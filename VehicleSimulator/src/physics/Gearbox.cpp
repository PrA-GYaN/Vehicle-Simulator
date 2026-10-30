#include "physics/Gearbox.h"

#include <algorithm>

#include "core/Units.h"

namespace vsim {

const char* shiftModeName(ShiftMode m) {
    return (m == ShiftMode::Manual) ? "Manual" : "Automatic";
}
const char* clutchModeName(ClutchMode m) {
    return (m == ClutchMode::Manual) ? "Manual" : "Automatic";
}

Gearbox::Gearbox(const GearboxParameters& params) : m_params(params) {
    m_params.validate();
}

double Gearbox::currentRatio() const {
    if (m_currentGear < 0 || m_currentGear >= static_cast<int>(m_params.gearRatios.size())) {
        return 0.0;
    }
    return m_params.gearRatios[static_cast<std::size_t>(m_currentGear)] * m_params.finalDrive;
}

int Gearbox::topGear() const {
    return static_cast<int>(m_params.gearRatios.size()) - 1;
}

bool Gearbox::shiftUp() {
    if (m_timeSinceShiftS < m_params.minShiftIntervalS) return false;
    if (m_currentGear >= topGear()) return false;
    ++m_currentGear;          // neutral (-1) steps up into 1st (0)
    m_timeSinceShiftS = 0.0;
    return true;
}

bool Gearbox::shiftDown() {
    if (m_timeSinceShiftS < m_params.minShiftIntervalS) return false;
    if (m_currentGear <= kNeutralGear) return false;
    --m_currentGear;          // 1st (0) steps down into neutral (-1)
    m_timeSinceShiftS = 0.0;
    return true;
}

void Gearbox::update(double wheelOmega, double redlineOmega, double dt) {
    m_timeSinceShiftS += dt;

    // Manual: the driver owns the gear. Only the debounce timer above is
    // still maintained, so a manual shift is rate-limited the same way an
    // automatic one is.
    if (m_params.shiftMode == ShiftMode::Manual) return;

    // An automatic never sits in neutral: if the driver selected it (the
    // manual override is honoured in either mode), the box picks 1st again
    // on its next evaluation. Without this it would be stuck - a zero ratio
    // makes implied engine speed zero, which reads as "far too low", and the
    // downshift branch below has nowhere further to go.
    if (m_currentGear <= kNeutralGear) {
        m_currentGear = 0;
        m_timeSinceShiftS = 0.0;
        return;
    }

    if (m_timeSinceShiftS < m_params.minShiftIntervalS) return;

    const int lastGear = topGear();
    if (lastGear < 0) return;

    const double w = std::max(0.0, sanitize(wheelOmega));
    const double impliedOmega = w * currentRatio();

    const double upThreshold   = redlineOmega * m_params.upshiftRedlineFraction;
    const double downThreshold = redlineOmega * m_params.downshiftRedlineFraction;

    if (impliedOmega > upThreshold && m_currentGear < lastGear) {
        ++m_currentGear;
        m_timeSinceShiftS = 0.0;
    } else if (impliedOmega < downThreshold && m_currentGear > 0) {
        --m_currentGear;
        m_timeSinceShiftS = 0.0;
    }
}

void Gearbox::reset() {
    m_currentGear = 0;
    m_timeSinceShiftS = 0.0;
}

} // namespace vsim
