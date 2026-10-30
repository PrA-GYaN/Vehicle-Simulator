#pragma once
//===========================================================================
//  AndroidUsbInput.h  --  TCP receiver for the Android USB/ADB controller.
//
//  Treat this exactly like Input: it produces a ControlInput plus a
//  handbrake flag, in the same normalised ranges, from a completely
//  different physical source. The vehicle/simulation layer never sees this
//  class - Application picks whichever input source is active and hands the
//  vehicle a plain ControlInput, so nothing downstream knows Android exists.
//
//  See AndroidController/PROTOCOL.md for the wire format this class speaks.
//
//  THREADING
//  ---------
//  A background thread owns the listening/accepted socket and does all
//  blocking I/O. The public accessors lock a small mutex and return a copy,
//  so the render thread never blocks on the network.
//
//  SAFETY
//  ------
//  controlInput()/handbrake() return the safe zero/centred state whenever
//  this class is not currently connected AND fresh (see isConnected()).
//  There is no code path that can hand out a stale non-zero value: the
//  caller does not need to remember to check isConnected() first, though it
//  should still do so to decide whether to fall back to another input source.
//===========================================================================

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include "config/Config.h"
#include "core/VehicleState.h"

namespace vsim {

class AndroidUsbInput {
public:
    explicit AndroidUsbInput(const AndroidInputConfig& config);
    ~AndroidUsbInput();

    AndroidUsbInput(const AndroidUsbInput&) = delete;
    AndroidUsbInput& operator=(const AndroidUsbInput&) = delete;

    /// Latest driving input (steering/throttle/brake/clutch). Safe (all-zero
    /// / centred) whenever the phone is not connected or the last valid
    /// packet is older than config.timeoutSeconds - see isConnected().
    ///
    /// shiftRequest is always 0 here - this accessor is side-effect-free and
    /// safe to call any number of times (the debug UI polls it just to
    /// display it). Gear changes are consumed separately - see
    /// consumeShiftRequest().
    ControlInput controlInput() const;

    /// Handbrake state. Safe (false) under the same conditions as above.
    bool handbrake() const;

    /// Turns the phone's latest gear-shift request into a one-shot -1/0/+1,
    /// exactly like Input::m_pendingShift does for the keyboard: calling
    /// this repeatedly for the SAME request returns 0 after the first call,
    /// no matter how many packets the phone sends still reporting it (see
    /// PROTOCOL.md - shiftDir is sticky, shiftSeq is what marks a NEW
    /// request). Safe (0) whenever the phone is not connected/fresh.
    ///
    /// Call this exactly once per simulation frame, from the one place that
    /// feeds the vehicle - never from a display/debug path, or that path
    /// would silently steal the edge before the real consumer sees it.
    int consumeShiftRequest();

    /// True when a client is connected AND a valid packet has arrived within
    /// the configured timeout. This is the single flag callers should use to
    /// decide whether to drive from this input source at all.
    bool isConnected() const;

    /// Seconds since the last valid packet, or +infinity if none has ever
    /// arrived. For the debug UI.
    double secondsSinceLastPacket() const;

    int  port() const { return m_config.port; }
    long packetsReceived() const { return m_packetsReceived.load(); }
    long malformedPackets() const { return m_malformedPackets.load(); }

private:
    void threadMain();
    bool openListenSocket();
    void closeListenSocket();
    void closeClientSocket();

    /// Blocks (with periodic wakeups to check m_running) until a client
    /// connects or shutdown is requested. Returns false on shutdown.
    bool acceptClient();

    /// Reads and applies packets from the connected client until it
    /// disconnects, errors, or shutdown is requested.
    void readLoop();

    /// Parse one line per the protocol in PROTOCOL.md. On success, updates
    /// the shared state and the last-packet clock. Returns false (and only
    /// bumps the malformed counter) on any validation failure.
    bool parseAndApply(const std::string& line);

    void attemptAdbReverse() const;

    AndroidInputConfig m_config;

    std::thread       m_thread;
    std::atomic<bool> m_running{true};
    std::atomic<bool> m_clientConnected{false};

    int m_listenSocket = -1;
    int m_clientSocket = -1;
    mutable std::mutex m_socketMutex;  ///< guards the two fds above

    mutable std::mutex       m_stateMutex;
    ControlInput              m_latest;             ///< shiftRequest always 0 - see controlInput()
    bool                      m_latestHandbrake = false;
    bool                      m_everReceivedPacket = false;
    std::chrono::steady_clock::time_point m_lastPacketTime;

    /// Raw gear-shift request as last reported by the phone (sticky - see
    /// PROTOCOL.md) and the bookkeeping consumeShiftRequest() uses to turn
    /// it into a one-shot edge.
    int  m_latestShiftDir = 0;
    int  m_latestShiftSeq = 0;
    int  m_lastConsumedShiftSeq = 0;
    bool m_haveConsumedShiftSeq = false;

    std::atomic<long> m_packetsReceived{0};
    std::atomic<long> m_malformedPackets{0};
};

} // namespace vsim
