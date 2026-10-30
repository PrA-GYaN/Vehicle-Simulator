#include "rendering/GLLoader.h"

#include <iostream>

#include <GLFW/glfw3.h>

namespace gl {

// Define the storage for every entry point declared in the header.
#define VSIM_GL_DEFINE(ret, name, args) PFN_##name name = nullptr;
VSIM_GL_FUNCTIONS(VSIM_GL_DEFINE)
#undef VSIM_GL_DEFINE

namespace {

const char* g_version  = "unknown";
const char* g_renderer = "unknown";

/// Resolve one entry point, recording failures rather than aborting on the
/// first one - it is far more useful to see the whole list of missing
/// functions at once than to fix them one context restart at a time.
template <typename Fn>
bool resolve(Fn& out, const char* name, int& failures) {
    out = reinterpret_cast<Fn>(glfwGetProcAddress(name));
    if (!out) {
        std::cerr << "[gl] missing entry point: gl" << name << "\n";
        ++failures;
        return false;
    }
    return true;
}

} // namespace

bool load() {
    int failures = 0;

#define VSIM_GL_RESOLVE(ret, name, args) resolve(name, "gl" #name, failures);
    VSIM_GL_FUNCTIONS(VSIM_GL_RESOLVE)
#undef VSIM_GL_RESOLVE

    if (failures > 0) {
        std::cerr << "[gl] " << failures
                  << " entry point(s) unavailable - is the context OpenGL 3.3 core?\n";
        return false;
    }

    if (const GLubyte* v = GetString(VERSION))  g_version  = reinterpret_cast<const char*>(v);
    if (const GLubyte* r = GetString(RENDERER)) g_renderer = reinterpret_cast<const char*>(r);

    return true;
}

const char* versionString()  { return g_version; }
const char* rendererString() { return g_renderer; }

} // namespace gl
