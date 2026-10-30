#pragma once
//===========================================================================
//  Input.h  --  GLFW keyboard/mouse -> normalised control input + actions.
//
//  The input layer produces a ControlInput in the range [0,1] / [-1,+1] and
//  a set of edge-triggered actions. It knows nothing about the vehicle: the
//  mapping from "steering = 0.7" to a road wheel angle happens in the vehicle
//  layer, which is why an experiment script can substitute for the keyboard
//  with no other changes.
//===========================================================================

#include <array>

#include <glm/glm.hpp>

#include "core/VehicleState.h"

struct GLFWwindow;

namespace vsim {

/// One-shot commands, true only on the frame the key was pressed.
struct InputActions {
    bool reset            = false;  // R
    bool togglePause      = false;  // P
    bool toggleTelemetry  = false;  // T
    bool toggleDebugUI    = false;  // F1
    bool resetCamera      = false;  // C
    bool cycleCamera      = false;  // V
    int  selectCamera     = -1;     // 1..9, 0 -> index into cameraCycleOrder()
    bool toggleMute       = false;  // M
    bool exportTelemetry  = false;  // F2
    bool singleStep       = false;  // N
    bool cycleTractionControl  = false;  // G: level 1 -> 2 -> 3 -> 1
    bool toggleTractionControl = false;  // H: on / off
    bool cycleTireCompound     = false;  // Y: Standard -> Soft -> Medium -> Hard
    bool quit             = false;  // Esc
};

class Input {
public:
    /// Attach GLFW callbacks. Must be called after the window exists.
    void attach(GLFWwindow* window);

    /// Poll continuous state. Call once per frame, before using the results.
    void beginFrame(GLFWwindow* window, double frameTime);

    /// Continuous driving input, already normalised.
    ///
    /// NOTE: no rate limiting is applied here. This is the raw REQUEST. The
    /// vehicle applies the actuator rates, which keeps the "how fast can the
    /// steering move" question in one place - the vehicle - rather than
    /// duplicated across every input source.
    const ControlInput& controlInput() const { return m_control; }

    /// Edge-triggered actions for this frame.
    const InputActions& actions() const { return m_actions; }

    /// Space bar held.
    bool emergencyBrake() const { return m_emergencyBrake; }

    //-- mouse ------------------------------------------------------------
    glm::dvec2 mouseDelta() const { return m_mouseDelta; }
    double     scrollDelta() const { return m_scrollDelta; }
    bool       mouseDragging() const { return m_dragging; }

    /// Free-camera movement direction in camera-local axes (x right, y up,
    /// z forward), unnormalised, components in [-1, 1].
    glm::vec3 freeCameraMove() const { return m_freeMove; }

    /// When true, the UI has keyboard focus and driving input is suppressed
    /// so typing in a text field does not steer the car.
    void setUiCapturesKeyboard(bool v) { m_uiCapturesKeyboard = v; }
    void setUiCapturesMouse(bool v)    { m_uiCapturesMouse = v; }

private:
    static void keyCallback(GLFWwindow* w, int key, int scancode, int action, int mods);
    static void scrollCallback(GLFWwindow* w, double xoffset, double yoffset);
    static void mouseButtonCallback(GLFWwindow* w, int button, int action, int mods);

    ControlInput m_control;
    InputActions m_actions;

    bool m_emergencyBrake = false;

    glm::dvec2 m_mousePos{0.0};
    glm::dvec2 m_mouseDelta{0.0};
    bool m_haveMousePos = false;
    bool m_dragging = false;

    double m_scrollDelta = 0.0;
    double m_scrollAccum = 0.0;   // filled by the callback, drained per frame

    glm::vec3 m_freeMove{0.0f};

    bool m_uiCapturesKeyboard = false;
    bool m_uiCapturesMouse = false;

    /// Pending edge-triggered actions, set by the key callback and consumed
    /// on the next beginFrame(). Using the callback rather than polling means
    /// a fast tap is never missed between frames.
    InputActions m_pendingActions;

    /// Gear-change request captured by the key callback, drained once per
    /// frame into ControlInput::shiftRequest. Same reason as m_pendingActions:
    /// polling would miss a tap between frames.
    int m_pendingShift = 0;
};

} // namespace vsim
