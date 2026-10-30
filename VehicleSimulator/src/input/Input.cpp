#include "input/Input.h"

#include <GLFW/glfw3.h>

namespace vsim {
namespace {

/// GLFW callbacks are C function pointers with no user context beyond the
/// window pointer, so the active Input instance is stashed in the window's
/// user pointer at attach() time.
Input* instanceFor(GLFWwindow* w) {
    return static_cast<Input*>(glfwGetWindowUserPointer(w));
}

bool held(GLFWwindow* w, int key) {
    return glfwGetKey(w, key) == GLFW_PRESS;
}

} // namespace

void Input::attach(GLFWwindow* window) {
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, &Input::keyCallback);
    glfwSetScrollCallback(window, &Input::scrollCallback);
    glfwSetMouseButtonCallback(window, &Input::mouseButtonCallback);
}

//---------------------------------------------------------------------------
//  Callbacks
//---------------------------------------------------------------------------

void Input::keyCallback(GLFWwindow* w, int key, int, int action, int) {
    Input* self = instanceFor(w);
    if (!self) return;

    // ImGui owns the keyboard while a text field is focused. Forwarding keys
    // in that state would make typing "8" into the mass field also reset the
    // simulation, which is exactly the kind of thing that erodes trust in a
    // debug tool.
    if (self->m_uiCapturesKeyboard) return;

    if (action != GLFW_PRESS) return;

    switch (key) {
        case GLFW_KEY_R:      self->m_pendingActions.reset = true; break;
        case GLFW_KEY_P:      self->m_pendingActions.togglePause = true; break;
        case GLFW_KEY_T:      self->m_pendingActions.toggleTelemetry = true; break;
        case GLFW_KEY_F1:     self->m_pendingActions.toggleDebugUI = true; break;
        case GLFW_KEY_F2:     self->m_pendingActions.exportTelemetry = true; break;
        case GLFW_KEY_C:      self->m_pendingActions.resetCamera = true; break;
        case GLFW_KEY_V:      self->m_pendingActions.cycleCamera = true; break;
        case GLFW_KEY_N:      self->m_pendingActions.singleStep = true; break;
        case GLFW_KEY_M:      self->m_pendingActions.toggleMute = true; break;
        case GLFW_KEY_G:      self->m_pendingActions.cycleTractionControl = true; break;
        case GLFW_KEY_H:      self->m_pendingActions.toggleTractionControl = true; break;
        case GLFW_KEY_Y:      self->m_pendingActions.cycleTireCompound = true; break;
        case GLFW_KEY_1: case GLFW_KEY_2: case GLFW_KEY_3: case GLFW_KEY_4:
        case GLFW_KEY_5: case GLFW_KEY_6: case GLFW_KEY_7: case GLFW_KEY_8:
        case GLFW_KEY_9:
            self->m_pendingActions.selectCamera = key - GLFW_KEY_1; break;
        case GLFW_KEY_0:      self->m_pendingActions.selectCamera = 9; break;
        // Gear changes are EVENTS, so they go through the same edge-triggered
        // path as the one-shot actions rather than being polled: a held key
        // must shift once, not sweep the whole gearbox.
        case GLFW_KEY_X:      self->m_pendingShift = +1; break;
        case GLFW_KEY_Z:      self->m_pendingShift = -1; break;
        case GLFW_KEY_ESCAPE: self->m_pendingActions.quit = true; break;
        default: break;
    }
}

void Input::scrollCallback(GLFWwindow* w, double, double yoffset) {
    Input* self = instanceFor(w);
    if (!self || self->m_uiCapturesMouse) return;
    self->m_scrollAccum += yoffset;
}

void Input::mouseButtonCallback(GLFWwindow* w, int button, int action, int) {
    Input* self = instanceFor(w);
    if (!self) return;

    if (button == GLFW_MOUSE_BUTTON_LEFT || button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS && !self->m_uiCapturesMouse) {
            self->m_dragging = true;
        } else if (action == GLFW_RELEASE) {
            self->m_dragging = false;
        }
    }
}

//---------------------------------------------------------------------------
//  Per-frame poll
//---------------------------------------------------------------------------

void Input::beginFrame(GLFWwindow* window, double frameTime) {
    (void)frameTime;

    // Consume the edge-triggered actions collected since the last frame.
    m_actions = m_pendingActions;
    m_pendingActions = InputActions{};

    // Drain accumulated scroll.
    m_scrollDelta = m_scrollAccum;
    m_scrollAccum = 0.0;

    //-- mouse motion -----------------------------------------------------
    double mx = 0.0, my = 0.0;
    glfwGetCursorPos(window, &mx, &my);

    if (!m_haveMousePos) {
        m_mousePos = {mx, my};
        m_haveMousePos = true;
    }
    m_mouseDelta = m_dragging ? glm::dvec2(mx - m_mousePos.x, my - m_mousePos.y)
                              : glm::dvec2(0.0);
    m_mousePos = {mx, my};

    //-- driving input ----------------------------------------------------
    if (m_uiCapturesKeyboard) {
        // Release everything while the UI has focus, rather than freezing the
        // last input - a stuck throttle would be worse than none.
        m_control = ControlInput{};
        m_emergencyBrake = false;
        m_freeMove = glm::vec3(0.0f);
        m_pendingShift = 0;
        return;
    }

    const bool up    = held(window, GLFW_KEY_W) || held(window, GLFW_KEY_UP);
    const bool down  = held(window, GLFW_KEY_S) || held(window, GLFW_KEY_DOWN);
    const bool left  = held(window, GLFW_KEY_A) || held(window, GLFW_KEY_LEFT);
    const bool rightK= held(window, GLFW_KEY_D) || held(window, GLFW_KEY_RIGHT);

    m_control.throttle = up   ? 1.0 : 0.0;
    m_control.brake    = down ? 1.0 : 0.0;

    // Steering sign follows the convention in core/Coordinates.h:
    //     +1 = steer RIGHT (heading rotates from +X toward +Z)
    // Holding both keys cancels, which is the least surprising behaviour.
    m_control.steering = (rightK ? 1.0 : 0.0) - (left ? 1.0 : 0.0);

    // Clutch pedal. A keyboard only gives us "down" or "up"; the smooth
    // travel between them is applied as an actuator rate in the vehicle
    // layer, the same way throttle and brake are.
    m_control.clutch = held(window, GLFW_KEY_LEFT_SHIFT) ? 1.0 : 0.0;

    // Drain the edge-triggered shift request captured by the key callback.
    m_control.shiftRequest = m_pendingShift;
    m_pendingShift = 0;

    m_control.clampToValidRange();

    m_emergencyBrake = held(window, GLFW_KEY_SPACE);

    //-- free camera movement (only meaningful in free camera mode) --------
    //
    // Uses IJKL + Q/E so it never collides with the driving keys. The camera
    // reads this; in other modes it is ignored.
    m_freeMove = glm::vec3(0.0f);
    if (held(window, GLFW_KEY_I)) m_freeMove.z += 1.0f;
    if (held(window, GLFW_KEY_K)) m_freeMove.z -= 1.0f;
    if (held(window, GLFW_KEY_L)) m_freeMove.x += 1.0f;
    if (held(window, GLFW_KEY_J)) m_freeMove.x -= 1.0f;
    if (held(window, GLFW_KEY_E)) m_freeMove.y += 1.0f;
    if (held(window, GLFW_KEY_Q)) m_freeMove.y -= 1.0f;
}

} // namespace vsim
