#include "rendering/ApexVenueKit.h"

#include "core/Coordinates.h"

namespace vsim::venuekit {

//---------------------------------------------------------------------------
//  Dot font: digits, capitals and a dash. Seven rows of five bits each.
//---------------------------------------------------------------------------

namespace {

struct Glyph { char ch; unsigned char rows[7]; };

const Glyph kGlyphs[] = {
    {'A', {0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}},
    {'B', {0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110}},
    {'C', {0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110}},
    {'D', {0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110}},
    {'E', {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111}},
    {'F', {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000}},
    {'G', {0b01110, 0b10001, 0b10000, 0b10111, 0b10001, 0b10001, 0b01111}},
    {'H', {0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}},
    {'I', {0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}},
    {'J', {0b00111, 0b00010, 0b00010, 0b00010, 0b00010, 0b10010, 0b01100}},
    {'K', {0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001}},
    {'L', {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111}},
    {'M', {0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001}},
    {'N', {0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001}},
    {'O', {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}},
    {'P', {0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000}},
    {'Q', {0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101}},
    {'R', {0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001}},
    {'S', {0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110}},
    {'T', {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100}},
    {'U', {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}},
    {'V', {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100}},
    {'W', {0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b11011, 0b10001}},
    {'X', {0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001}},
    {'Y', {0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b00100}},
    {'Z', {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111}},
    {'0', {0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110}},
    {'1', {0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}},
    {'2', {0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111}},
    {'3', {0b11110, 0b00001, 0b00001, 0b01110, 0b00001, 0b00001, 0b11110}},
    {'4', {0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010}},
    {'5', {0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110}},
    {'6', {0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110}},
    {'7', {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000}},
    {'8', {0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110}},
    {'9', {0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100}},
    {'-', {0b00000, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000}},
};

const unsigned char kBlank[7] = {0, 0, 0, 0, 0, 0, 0};

} // namespace

const unsigned char* glyph(char ch) {
    for (const Glyph& g : kGlyphs) {
        if (g.ch == ch) return g.rows;
    }
    return kBlank;
}

void textQuads(MeshData& m, const vec3& origin, const vec3& dx, const vec3& dy,
               const vec3& normal, const std::string& text, float px, float py,
               const Brush& br) {
    for (std::size_t c = 0; c < text.size(); ++c) {
        const unsigned char* rows = glyph(text[c]);
        const float x0 = static_cast<float>(c) * 6.0f * px;
        for (int r = 0; r < 7; ++r) {
            // Merge each horizontal run of dots into one quad.
            int col = 0;
            while (col < 5) {
                if (!(rows[r] & (0x10 >> col))) { ++col; continue; }
                int end = col;
                while (end < 5 && (rows[r] & (0x10 >> end))) ++end;
                const float xa = x0 + static_cast<float>(col) * px;
                const float xb = x0 + static_cast<float>(end) * px;
                const float ya = static_cast<float>(6 - r) * py;
                const float yb = ya + py;
                quad(m, origin + dx * xa + dy * ya, origin + dx * xb + dy * ya,
                     origin + dx * xb + dy * yb, origin + dx * xa + dy * yb, normal, br);
                col = end;
            }
        }
    }
}

//---------------------------------------------------------------------------
//  Lap
//---------------------------------------------------------------------------

Lap::Lap(const Track& track) {
    hw = static_cast<float>(track.width() * 0.5);
    const auto& pts = track.centreline();
    f.reserve(pts.size());
    for (const TrackPoint& p : pts) {
        Frame fr;
        fr.pos = vec3(p.position);
        fr.pos.y = 0.0f;
        const double yaw = coords::yawFromDirection(p.tangent);
        fr.fwd   = vec3(coords::heading(yaw));
        fr.right = vec3(coords::right(yaw));
        fr.s     = static_cast<float>(p.distance);
        fr.curv  = static_cast<float>(p.curvature);
        f.push_back(fr);
    }
    if (!f.empty()) length = f.back().s + glm::length(f.front().pos - f.back().pos);
}

int Lap::locate(float s, float& u) const {
    s = wrapS(s);
    int lo = 0, hi = n() - 1;
    while (lo < hi) {
        const int mid = (lo + hi + 1) / 2;
        if (f[static_cast<std::size_t>(mid)].s <= s) lo = mid; else hi = mid - 1;
    }
    const float s0 = f[static_cast<std::size_t>(lo)].s;
    const float s1 = (lo + 1 < n()) ? f[static_cast<std::size_t>(lo + 1)].s : length;
    u = std::min(std::max((s - s0) / std::max(s1 - s0, 1e-4f), 0.0f), 1.0f);
    return lo;
}

Frame Lap::at(float s) const {
    float u = 0.0f;
    const int i = locate(s, u);
    const Frame& a = f[static_cast<std::size_t>(i)];
    const Frame& b = f[static_cast<std::size_t>(wrap(i + 1))];
    Frame r;
    r.pos   = glm::mix(a.pos, b.pos, u);
    r.fwd   = glm::normalize(glm::mix(a.fwd, b.fwd, u));
    r.right = glm::normalize(glm::mix(a.right, b.right, u));
    r.s     = wrapS(s);
    r.curv  = glm::mix(a.curv, b.curv, u);
    return r;
}

} // namespace vsim::venuekit
