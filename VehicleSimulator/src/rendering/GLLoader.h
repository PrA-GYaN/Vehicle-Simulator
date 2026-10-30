#pragma once
//===========================================================================
//  GLLoader.h  --  minimal OpenGL 3.3 core loader.
//
//  WHY THIS EXISTS
//  ---------------
//  The renderer needs about thirty OpenGL entry points beyond version 1.1.
//  Pulling in GLEW or GLAD for thirty function pointers adds a build-time
//  dependency (and, for GLAD, a Python code-generation step) for very little
//  benefit. This header declares exactly what is used and nothing else.
//
//  Deliberately does NOT include <GL/gl.h>. All types and enum values are
//  defined here, so there is no chance of a conflict with the system GL
//  headers or with the loader that Dear ImGui's OpenGL3 backend carries
//  internally (that lives in its own translation unit).
//
//  Everything is loaded through glfwGetProcAddress, so a GL context must be
//  current before calling gl::load().
//===========================================================================

#include <cstddef>
#include <cstdint>

namespace gl {

//---------------------------------------------------------------------------
//  Types
//---------------------------------------------------------------------------
using GLenum     = unsigned int;
using GLboolean  = unsigned char;
using GLbitfield = unsigned int;
using GLvoid     = void;
using GLbyte     = signed char;
using GLubyte    = unsigned char;
using GLshort    = short;
using GLushort   = unsigned short;
using GLint      = int;
using GLuint     = unsigned int;
using GLsizei    = int;
using GLfloat    = float;
using GLdouble   = double;
using GLchar     = char;
using GLintptr   = std::intptr_t;
using GLsizeiptr = std::ptrdiff_t;

//---------------------------------------------------------------------------
//  Enum values actually used by the renderer
//---------------------------------------------------------------------------
constexpr GLenum FALSE_                  = 0;
constexpr GLenum TRUE_                   = 1;

constexpr GLbitfield COLOR_BUFFER_BIT    = 0x00004000;
constexpr GLbitfield DEPTH_BUFFER_BIT    = 0x00000100;

constexpr GLenum DEPTH_TEST              = 0x0B71;
constexpr GLenum BLEND                   = 0x0BE2;
constexpr GLenum CULL_FACE               = 0x0B44;
constexpr GLenum MULTISAMPLE             = 0x809D;
constexpr GLenum LINE_SMOOTH             = 0x0B20;
constexpr GLenum POLYGON_OFFSET_FILL     = 0x8037;

constexpr GLenum SRC_ALPHA               = 0x0302;
constexpr GLenum ONE_MINUS_SRC_ALPHA     = 0x0303;

constexpr GLenum LESS                    = 0x0201;
constexpr GLenum LEQUAL                  = 0x0203;

constexpr GLenum BACK                    = 0x0405;
constexpr GLenum FRONT_AND_BACK          = 0x0408;
constexpr GLenum CCW                     = 0x0901;
constexpr GLenum FILL                    = 0x1B02;
constexpr GLenum LINE                    = 0x1B01;

constexpr GLenum POINTS                  = 0x0000;
constexpr GLenum LINES                   = 0x0001;
constexpr GLenum LINE_STRIP              = 0x0003;
constexpr GLenum TRIANGLES               = 0x0004;
constexpr GLenum TRIANGLE_STRIP          = 0x0005;
constexpr GLenum TRIANGLE_FAN            = 0x0006;

constexpr GLenum ARRAY_BUFFER            = 0x8892;
constexpr GLenum ELEMENT_ARRAY_BUFFER    = 0x8893;
constexpr GLenum STATIC_DRAW             = 0x88E4;
constexpr GLenum DYNAMIC_DRAW            = 0x88E8;
constexpr GLenum STREAM_DRAW             = 0x88E0;

constexpr GLenum FLOAT_                  = 0x1406;
constexpr GLenum UNSIGNED_BYTE_          = 0x1401;
constexpr GLenum UNSIGNED_INT            = 0x1405;

//---------------------------------------------------------------------------
//  Textures - added for glTF model loading (src/rendering/GltfModel.h).
//  Everything else in the renderer is still vertex-colour only; textures
//  exist solely to reproduce an imported model's own material colours.
//---------------------------------------------------------------------------
constexpr GLenum TEXTURE_2D              = 0x0DE1;
constexpr GLenum TEXTURE0                = 0x84C0;
constexpr GLenum TEXTURE_MIN_FILTER      = 0x2801;
constexpr GLenum TEXTURE_MAG_FILTER      = 0x2800;
constexpr GLenum TEXTURE_WRAP_S          = 0x2802;
constexpr GLenum TEXTURE_WRAP_T          = 0x2803;
constexpr GLenum NEAREST                 = 0x2600;
constexpr GLenum LINEAR                  = 0x2601;
constexpr GLenum LINEAR_MIPMAP_LINEAR    = 0x2703;
constexpr GLenum REPEAT                  = 0x2901;
constexpr GLenum CLAMP_TO_EDGE           = 0x812F;
constexpr GLenum RGB                     = 0x1907;
constexpr GLenum RGBA                    = 0x1908;
constexpr GLenum UNPACK_ALIGNMENT        = 0x0CF5;
constexpr GLenum PACK_ALIGNMENT          = 0x0D05;

//---------------------------------------------------------------------------
//  Framebuffers - added for shadow mapping (src/rendering/ShadowMap.h). The
//  shadow pass renders scene depth into a texture from the light's point of
//  view; nothing else in the renderer draws off-screen.
//---------------------------------------------------------------------------
constexpr GLenum FRAMEBUFFER             = 0x8D40;
constexpr GLenum FRAMEBUFFER_COMPLETE    = 0x8CD5;
constexpr GLenum DEPTH_ATTACHMENT        = 0x8D00;
constexpr GLenum DEPTH_COMPONENT         = 0x1902;
constexpr GLenum DEPTH_COMPONENT24       = 0x81A6;
constexpr GLenum CLAMP_TO_BORDER         = 0x812D;
constexpr GLenum TEXTURE_2D_ARRAY        = 0x8C1A;
constexpr GLenum COMPRESSED_RGB_S3TC_DXT1 = 0x83F0;  // EXT_texture_compression_s3tc
constexpr GLenum TEXTURE_MAX_ANISOTROPY  = 0x84FE;   // EXT_texture_filter_anisotropic
constexpr GLenum SAMPLE_ALPHA_TO_COVERAGE = 0x809E;
constexpr GLenum TEXTURE_COMPARE_MODE    = 0x884C;
constexpr GLenum TEXTURE_COMPARE_FUNC    = 0x884D;
constexpr GLenum COMPARE_REF_TO_TEXTURE  = 0x884E;
constexpr GLenum TEXTURE_BORDER_COLOR    = 0x1004;
constexpr GLenum FRONT                   = 0x0404;
constexpr GLenum NONE_                   = 0;

constexpr GLenum VERTEX_SHADER           = 0x8B31;
constexpr GLenum FRAGMENT_SHADER         = 0x8B30;
constexpr GLenum COMPILE_STATUS          = 0x8B81;
constexpr GLenum LINK_STATUS             = 0x8B82;
constexpr GLenum INFO_LOG_LENGTH         = 0x8B84;

constexpr GLenum VENDOR                  = 0x1F00;
constexpr GLenum RENDERER                = 0x1F01;
constexpr GLenum VERSION                 = 0x1F02;

//---------------------------------------------------------------------------
//  Entry points.
//
//  X-macro list: one line per function, so adding a call site means adding
//  exactly one line here and nothing else.
//---------------------------------------------------------------------------
#define VSIM_GL_FUNCTIONS(X)                                                                    \
    X(void,   Clear,               (GLbitfield mask))                                           \
    X(void,   ClearColor,          (GLfloat r, GLfloat g, GLfloat b, GLfloat a))                \
    X(void,   Enable,              (GLenum cap))                                                \
    X(void,   Disable,             (GLenum cap))                                                \
    X(void,   DepthFunc,           (GLenum func))                                               \
    X(void,   DepthMask,           (GLboolean flag))                                            \
    X(void,   BlendFunc,           (GLenum src, GLenum dst))                                    \
    X(void,   CullFace,            (GLenum mode))                                               \
    X(void,   FrontFace,           (GLenum mode))                                               \
    X(void,   Viewport,            (GLint x, GLint y, GLsizei w, GLsizei h))                    \
    X(void,   LineWidth,           (GLfloat width))                                             \
    X(void,   PolygonMode,         (GLenum face, GLenum mode))                                  \
    X(void,   PolygonOffset,       (GLfloat factor, GLfloat units))                             \
    X(void,   DrawArrays,          (GLenum mode, GLint first, GLsizei count))                   \
    X(void,   DrawElements,        (GLenum mode, GLsizei count, GLenum type, const void* idx))  \
    X(const GLubyte*, GetString,   (GLenum name))                                               \
    X(GLenum, GetError,            (void))                                                      \
    X(void,   GenBuffers,          (GLsizei n, GLuint* buffers))                                \
    X(void,   BindBuffer,          (GLenum target, GLuint buffer))                              \
    X(void,   BufferData,          (GLenum target, GLsizeiptr size, const void* data, GLenum usage)) \
    X(void,   BufferSubData,       (GLenum target, GLintptr offset, GLsizeiptr size, const void* data)) \
    X(void,   DeleteBuffers,       (GLsizei n, const GLuint* buffers))                          \
    X(void,   GenVertexArrays,     (GLsizei n, GLuint* arrays))                                 \
    X(void,   BindVertexArray,     (GLuint array))                                              \
    X(void,   DeleteVertexArrays,  (GLsizei n, const GLuint* arrays))                           \
    X(void,   VertexAttribPointer, (GLuint index, GLint size, GLenum type, GLboolean norm, GLsizei stride, const void* ptr)) \
    X(void,   EnableVertexAttribArray, (GLuint index))                                          \
    X(GLuint, CreateShader,        (GLenum type))                                               \
    X(void,   ShaderSource,        (GLuint s, GLsizei count, const GLchar* const* str, const GLint* len)) \
    X(void,   CompileShader,       (GLuint s))                                                  \
    X(void,   GetShaderiv,         (GLuint s, GLenum pname, GLint* params))                     \
    X(void,   GetShaderInfoLog,    (GLuint s, GLsizei bufSize, GLsizei* length, GLchar* log))   \
    X(void,   DeleteShader,        (GLuint s))                                                  \
    X(GLuint, CreateProgram,       (void))                                                      \
    X(void,   AttachShader,        (GLuint prog, GLuint shader))                                \
    X(void,   LinkProgram,         (GLuint prog))                                               \
    X(void,   GetProgramiv,        (GLuint prog, GLenum pname, GLint* params))                  \
    X(void,   GetProgramInfoLog,   (GLuint prog, GLsizei bufSize, GLsizei* length, GLchar* log))\
    X(void,   DeleteProgram,       (GLuint prog))                                               \
    X(void,   UseProgram,          (GLuint prog))                                               \
    X(GLint,  GetUniformLocation,  (GLuint prog, const GLchar* name))                           \
    X(void,   Uniform1i,           (GLint loc, GLint v))                                        \
    X(void,   Uniform1f,           (GLint loc, GLfloat v))                                      \
    X(void,   Uniform3fv,          (GLint loc, GLsizei count, const GLfloat* v))                \
    X(void,   Uniform4fv,          (GLint loc, GLsizei count, const GLfloat* v))                \
    X(void,   UniformMatrix4fv,    (GLint loc, GLsizei count, GLboolean transpose, const GLfloat* v)) \
    X(void,   GenTextures,         (GLsizei n, GLuint* textures))                                \
    X(void,   DeleteTextures,      (GLsizei n, const GLuint* textures))                          \
    X(void,   BindTexture,         (GLenum target, GLuint texture))                              \
    X(void,   ActiveTexture,       (GLenum texture))                                             \
    X(void,   TexImage2D,          (GLenum target, GLint level, GLint internalFormat, GLsizei w, GLsizei h, GLint border, GLenum format, GLenum type, const void* pixels)) \
    X(void,   TexParameteri,       (GLenum target, GLenum pname, GLint param))                   \
    X(void,   GenerateMipmap,      (GLenum target))                                              \
    X(void,   PixelStorei,         (GLenum pname, GLint param))                                  \
    X(void,   TexParameterfv,      (GLenum target, GLenum pname, const GLfloat* params))         \
    X(void,   GenFramebuffers,     (GLsizei n, GLuint* framebuffers))                            \
    X(void,   DeleteFramebuffers,  (GLsizei n, const GLuint* framebuffers))                      \
    X(void,   BindFramebuffer,     (GLenum target, GLuint framebuffer))                          \
    X(void,   FramebufferTexture2D,(GLenum target, GLenum attachment, GLenum textarget, GLuint texture, GLint level)) \
    X(GLenum, CheckFramebufferStatus, (GLenum target))                                           \
    X(void,   DrawBuffer,          (GLenum mode))                                                \
    X(void,   ReadBuffer,          (GLenum mode))                                                \
    X(void,   DrawElementsInstanced, (GLenum mode, GLsizei count, GLenum type, const void* idx, GLsizei instances)) \
    X(void,   VertexAttribDivisor, (GLuint index, GLuint divisor))                               \
    X(void,   ReadPixels,          (GLint x, GLint y, GLsizei w, GLsizei h, GLenum format, GLenum type, void* data)) \
    X(void,   Finish,              (void))                                                       \
    X(void,   TexImage3D,          (GLenum target, GLint level, GLint internalFormat, GLsizei w, GLsizei h, GLsizei d, GLint border, GLenum format, GLenum type, const void* pixels)) \
    X(void,   TexSubImage3D,       (GLenum target, GLint level, GLint x, GLint y, GLint z, GLsizei w, GLsizei h, GLsizei d, GLenum format, GLenum type, const void* pixels))

// Declare a function-pointer typedef and an extern pointer for each entry.
#define VSIM_GL_DECLARE(ret, name, args) \
    using PFN_##name = ret (*) args;     \
    extern PFN_##name name;
VSIM_GL_FUNCTIONS(VSIM_GL_DECLARE)
#undef VSIM_GL_DECLARE

/// Resolve every entry point. A GL context must already be current.
/// Returns false if any function could not be resolved, and reports which.
bool load();

/// Human-readable driver info, valid after load().
const char* versionString();
const char* rendererString();

} // namespace gl
