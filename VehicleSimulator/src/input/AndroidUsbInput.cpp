#include "input/AndroidUsbInput.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

//---------------------------------------------------------------------------
//  Socket API shim. POSIX (Linux/macOS) is what this project is built and
//  tested on; the Winsock branch is provided so the same source compiles on
//  Windows, since nothing else about AndroidUsbInput is platform-specific.
//---------------------------------------------------------------------------
#if defined(_WIN32)
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    using socket_t = SOCKET;
    static constexpr socket_t kInvalidSocket = INVALID_SOCKET;
    static void closeSocketFd(socket_t s) { closesocket(s); }
    static int lastSocketError() { return WSAGetLastError(); }
    #define VSIM_EWOULDBLOCK WSAEWOULDBLOCK
    #define VSIM_ETIMEDOUT   WSAETIMEDOUT
#else
    #include <arpa/inet.h>
    #include <fcntl.h>
    #include <netinet/in.h>
    #include <netinet/tcp.h>
    #include <sys/select.h>
    #include <sys/socket.h>
    #include <unistd.h>
    using socket_t = int;
    static constexpr socket_t kInvalidSocket = -1;
    static void closeSocketFd(socket_t s) { ::close(s); }
    static int lastSocketError() { return errno; }
    #define VSIM_EWOULDBLOCK EWOULDBLOCK
    #define VSIM_ETIMEDOUT   ETIMEDOUT
#endif

namespace vsim {
namespace {

/// select()-based wait so accept()/recv() wake up periodically to re-check
/// the shutdown flag instead of blocking forever.
///
/// Returns 1 if the socket became readable, 0 on timeout, -1 on error.
int waitReadable(socket_t fd, int timeoutMs) {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    timeval tv{};
    tv.tv_sec  = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;
    return select(static_cast<int>(fd) + 1, &set, nullptr, nullptr, &tv);
}

constexpr int kPollTimeoutMs = 200;

} // namespace

//---------------------------------------------------------------------------
//  Lifecycle
//---------------------------------------------------------------------------

AndroidUsbInput::AndroidUsbInput(const AndroidInputConfig& config) : m_config(config) {
#if defined(_WIN32)
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    m_lastPacketTime = std::chrono::steady_clock::now();
    m_thread = std::thread(&AndroidUsbInput::threadMain, this);
}

AndroidUsbInput::~AndroidUsbInput() {
    m_running = false;
    // Wake up anything blocked in select()/accept()/recv() by closing the
    // fds out from under the worker thread, then wait for it to notice.
    closeClientSocket();
    closeListenSocket();
    if (m_thread.joinable()) m_thread.join();
#if defined(_WIN32)
    WSACleanup();
#endif
}

//---------------------------------------------------------------------------
//  Socket setup
//---------------------------------------------------------------------------

bool AndroidUsbInput::openListenSocket() {
    std::lock_guard<std::mutex> lock(m_socketMutex);

    m_listenSocket = static_cast<int>(socket(AF_INET, SOCK_STREAM, 0));
    if (m_listenSocket == static_cast<int>(kInvalidSocket)) {
        std::cerr << "[android] socket() failed: " << lastSocketError() << "\n";
        return false;
    }

    const int yes = 1;
    setsockopt(static_cast<socket_t>(m_listenSocket), SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&yes), sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(m_config.port));
    // Loopback only: this must match the far end of `adb reverse`, and there
    // is no reason for anything off-device to reach this port directly.
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (bind(static_cast<socket_t>(m_listenSocket), reinterpret_cast<sockaddr*>(&addr),
             sizeof(addr)) != 0) {
        std::cerr << "[android] bind() to 127.0.0.1:" << m_config.port
                  << " failed: " << lastSocketError()
                  << " (is another instance already running?)\n";
        closeSocketFd(static_cast<socket_t>(m_listenSocket));
        m_listenSocket = -1;
        return false;
    }

    if (listen(static_cast<socket_t>(m_listenSocket), 1) != 0) {
        std::cerr << "[android] listen() failed: " << lastSocketError() << "\n";
        closeSocketFd(static_cast<socket_t>(m_listenSocket));
        m_listenSocket = -1;
        return false;
    }

    std::cout << "[android] listening on 127.0.0.1:" << m_config.port
              << " for the controller app (run `adb reverse tcp:" << m_config.port
              << " tcp:" << m_config.port << "` if auto-reverse is disabled or ADB "
              << "was restarted)\n";
    return true;
}

void AndroidUsbInput::closeListenSocket() {
    std::lock_guard<std::mutex> lock(m_socketMutex);
    if (m_listenSocket >= 0) {
        closeSocketFd(static_cast<socket_t>(m_listenSocket));
        m_listenSocket = -1;
    }
}

void AndroidUsbInput::closeClientSocket() {
    std::lock_guard<std::mutex> lock(m_socketMutex);
    if (m_clientSocket >= 0) {
        closeSocketFd(static_cast<socket_t>(m_clientSocket));
        m_clientSocket = -1;
    }
    m_clientConnected = false;
}

void AndroidUsbInput::attemptAdbReverse() const {
    if (!m_config.autoAdbReverse) return;

    std::ostringstream cmd;
#if defined(_WIN32)
    cmd << "adb reverse tcp:" << m_config.port << " tcp:" << m_config.port
        << " >NUL 2>&1";
#else
    cmd << "adb reverse tcp:" << m_config.port << " tcp:" << m_config.port
        << " >/dev/null 2>&1";
#endif
    const int rc = std::system(cmd.str().c_str());
    if (rc == 0) {
        std::cout << "[android] adb reverse tcp:" << m_config.port << " tcp:"
                  << m_config.port << " OK\n";
    } else {
        std::cout << "[android] adb reverse failed (adb not installed, no "
                     "device attached, or USB debugging not authorised) - "
                     "run it manually once the phone is connected\n";
    }
}

//---------------------------------------------------------------------------
//  Worker thread
//---------------------------------------------------------------------------

void AndroidUsbInput::threadMain() {
    if (!openListenSocket()) return;
    attemptAdbReverse();

    while (m_running.load()) {
        if (!acceptClient()) break;  // shutdown requested while waiting
        std::cout << "[android] controller connected\n";
        readLoop();
        closeClientSocket();
        if (m_running.load()) std::cout << "[android] controller disconnected; "
                                            "waiting for reconnection\n";
    }

    closeListenSocket();
}

bool AndroidUsbInput::acceptClient() {
    while (m_running.load()) {
        int listenFd;
        {
            std::lock_guard<std::mutex> lock(m_socketMutex);
            listenFd = m_listenSocket;
        }
        if (listenFd < 0) return false;

        const int ready = waitReadable(static_cast<socket_t>(listenFd), kPollTimeoutMs);
        if (ready <= 0) continue;  // timeout or interrupted select; re-check m_running

        const socket_t client = accept(static_cast<socket_t>(listenFd), nullptr, nullptr);
        if (client == kInvalidSocket) continue;

        const int yes = 1;
        setsockopt(client, IPPROTO_TCP, TCP_NODELAY,
                   reinterpret_cast<const char*>(&yes), sizeof(yes));
#if defined(_WIN32)
        DWORD timeoutMs = kPollTimeoutMs;
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&timeoutMs), sizeof(timeoutMs));
#else
        timeval tv{};
        tv.tv_sec  = 0;
        tv.tv_usec = kPollTimeoutMs * 1000;
        setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

        std::lock_guard<std::mutex> lock(m_socketMutex);
        m_clientSocket = static_cast<int>(client);
        m_clientConnected = true;
        return true;
    }
    return false;
}

void AndroidUsbInput::readLoop() {
    std::string accumulator;
    char buffer[4096];

    while (m_running.load()) {
        int clientFd;
        {
            std::lock_guard<std::mutex> lock(m_socketMutex);
            clientFd = m_clientSocket;
        }
        if (clientFd < 0) return;

        const auto n = recv(static_cast<socket_t>(clientFd), buffer, sizeof(buffer), 0);

        if (n > 0) {
            accumulator.append(buffer, static_cast<std::size_t>(n));

            // Only the LAST complete line matters - this is what keeps the
            // receiver from ever working through a backlog. Everything
            // after the final newline is a partial line, kept for next time.
            const std::size_t lastNewline = accumulator.find_last_of('\n');
            if (lastNewline != std::string::npos) {
                const std::size_t priorNewline =
                    accumulator.find_last_of('\n', lastNewline > 0 ? lastNewline - 1
                                                                    : std::string::npos);
                const std::size_t lineStart =
                    (priorNewline == std::string::npos) ? 0 : priorNewline + 1;
                const std::string lastLine =
                    accumulator.substr(lineStart, lastNewline - lineStart);
                if (!lastLine.empty()) parseAndApply(lastLine);

                accumulator.erase(0, lastNewline + 1);
            }

            // Guard against an unterminated stream growing without bound.
            if (accumulator.size() > 8192) {
                accumulator.erase(0, accumulator.size() - 4096);
            }
            continue;
        }

        if (n == 0) return;  // peer closed the connection cleanly

        const int err = lastSocketError();
        if (err == VSIM_EWOULDBLOCK || err == VSIM_ETIMEDOUT || err == EINTR) {
            continue;  // recv() timeout - just re-check m_running
        }
        return;  // a real error: treat as disconnected
    }
}

//---------------------------------------------------------------------------
//  Packet parsing
//---------------------------------------------------------------------------

bool AndroidUsbInput::parseAndApply(const std::string& lineIn) {
    std::string line = lineIn;
    if (!line.empty() && line.back() == '\r') line.pop_back();  // tolerate CRLF

    std::vector<std::string> fields;
    fields.reserve(9);
    std::size_t start = 0;
    while (true) {
        const std::size_t comma = line.find(',', start);
        if (comma == std::string::npos) {
            fields.push_back(line.substr(start));
            break;
        }
        fields.push_back(line.substr(start, comma - start));
        start = comma + 1;
    }

    if (fields.size() != 9 || fields[0] != "VSIM2") {
        ++m_malformedPackets;
        return false;
    }

    auto parseDouble = [](const std::string& s, double& out) -> bool {
        if (s.empty()) return false;
        char* end = nullptr;
        errno = 0;
        const double v = std::strtod(s.c_str(), &end);
        if (end != s.c_str() + s.size() || errno == ERANGE || !std::isfinite(v)) return false;
        out = v;
        return true;
    };

    // shiftSeq is a wrapping counter, not a physical quantity - parsed as a
    // long rather than through parseDouble so a huge value can't round-trip
    // through a double and silently change which requests compare equal.
    auto parseLong = [](const std::string& s, long& out) -> bool {
        if (s.empty()) return false;
        char* end = nullptr;
        errno = 0;
        const long v = std::strtol(s.c_str(), &end, 10);
        if (end != s.c_str() + s.size() || errno == ERANGE) return false;
        out = v;
        return true;
    };

    double steering = 0.0, throttle = 0.0, brake = 0.0, clutch = 0.0;
    if (!parseDouble(fields[2], steering) || !parseDouble(fields[3], throttle) ||
        !parseDouble(fields[4], brake) || !parseDouble(fields[6], clutch)) {
        ++m_malformedPackets;
        return false;
    }

    bool handbrake;
    if (fields[5] == "0") {
        handbrake = false;
    } else if (fields[5] == "1") {
        handbrake = true;
    } else {
        ++m_malformedPackets;
        return false;
    }

    int shiftDir;
    if (fields[7] == "-1")      shiftDir = -1;
    else if (fields[7] == "0")  shiftDir = 0;
    else if (fields[7] == "1")  shiftDir = 1;
    else {
        ++m_malformedPackets;
        return false;
    }

    long shiftSeq = 0;
    if (!parseLong(fields[8], shiftSeq)) {
        ++m_malformedPackets;
        return false;
    }

    ControlInput c;
    c.steering = steering;
    c.throttle = throttle;
    c.brake    = brake;
    c.clutch   = clutch;
    // shiftRequest stays 0 - see AndroidUsbInput::controlInput() and
    // consumeShiftRequest() for why it is tracked separately below.
    c.clampToValidRange();

    {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        m_latest             = c;
        m_latestHandbrake    = handbrake;
        m_latestShiftDir     = shiftDir;
        m_latestShiftSeq     = static_cast<int>(shiftSeq);
        m_everReceivedPacket = true;
        m_lastPacketTime     = std::chrono::steady_clock::now();
    }
    ++m_packetsReceived;
    return true;
}

//---------------------------------------------------------------------------
//  Accessors
//---------------------------------------------------------------------------

bool AndroidUsbInput::isConnected() const {
    if (!m_clientConnected.load()) return false;
    std::lock_guard<std::mutex> lock(m_stateMutex);
    if (!m_everReceivedPacket) return false;
    const std::chrono::duration<double> age = std::chrono::steady_clock::now() - m_lastPacketTime;
    return age.count() < m_config.timeoutSeconds;
}

double AndroidUsbInput::secondsSinceLastPacket() const {
    std::lock_guard<std::mutex> lock(m_stateMutex);
    if (!m_everReceivedPacket) return std::numeric_limits<double>::infinity();
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - m_lastPacketTime)
        .count();
}

ControlInput AndroidUsbInput::controlInput() const {
    if (!isConnected()) return ControlInput{};  // safe: zero throttle/brake, centred steering
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return m_latest;
}

bool AndroidUsbInput::handbrake() const {
    if (!isConnected()) return false;  // safe state
    std::lock_guard<std::mutex> lock(m_stateMutex);
    return m_latestHandbrake;
}

int AndroidUsbInput::consumeShiftRequest() {
    if (!isConnected()) return 0;  // safe state: no phantom shifts on reconnect
    std::lock_guard<std::mutex> lock(m_stateMutex);
    if (m_haveConsumedShiftSeq && m_latestShiftSeq == m_lastConsumedShiftSeq) return 0;
    m_lastConsumedShiftSeq = m_latestShiftSeq;
    m_haveConsumedShiftSeq = true;
    return m_latestShiftDir;
}

} // namespace vsim
