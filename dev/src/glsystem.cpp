#include "stdafx.h"

#include "glinterface.h"

#include "glincludes.h"

#include "sdlincludes.h"

#ifdef WIN32
#define GLEXT(type, name) type name = NULL;
GLBASEEXTS GLEXTS
#undef GLEXT
#endif

float4x4 view2clip(1);
float4x4 object2view(1);
float4x4 view2object(1);

vector<Light> lights;

float4 curcolor = float4_0;

static BlendMode curblendmode = BLEND_NONE;
static CullMode curcullmode = CULL_NONE;
static bool curdepthtest = true;
static bool curdepthwrite = true;

int SetBlendMode(BlendMode mode)
{
    if (mode == curblendmode) return curblendmode;
    switch (mode)
    {
        case BLEND_NONE:     glDisable(GL_BLEND); break;
        case BLEND_ALPHA:    glEnable (GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); break; // alpha / interpolative
        case BLEND_ADD:      glEnable (GL_BLEND); glBlendFunc(GL_ONE,       GL_ONE                ); break; // additive (plain)
        case BLEND_ADDALPHA: glEnable (GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE                ); break; // additive (using src alpha)
        case BLEND_MUL:      glEnable (GL_BLEND); glBlendFunc(GL_DST_COLOR, GL_ZERO               ); break; // multiplicative / masking
    }
    int old = curblendmode;
    curblendmode = mode;
    return old;
}

int SetCullMode(CullMode mode)
{
    if (mode == curcullmode)
        return curcullmode;

    switch (mode)
    {
    case CULL_NONE:
        glDisable(GL_CULL_FACE);
        break;

    case CULL_FRONT:
        glEnable(GL_CULL_FACE);
        glCullFace(GL_FRONT);
        break;

    case CULL_BACK:
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        break;
    }

    int old = curcullmode;
    curcullmode = mode;
    return old;
}

bool SetDepthTest(bool test)
{
    bool old = curdepthtest;

    curdepthtest = test;

    if (curdepthtest)
        glDepthFunc(GL_LESS);
    else
        glDepthFunc(GL_ALWAYS);

    return old;
}

bool SetDepthWrite(bool write)
{
    bool old = curdepthwrite;

    curdepthwrite = write;

    if (curdepthwrite)
        glDepthMask(TRUE);
    else
        glDepthMask(FALSE);

    return old;
}

void ClearFrameBuffer(const float3 &c)
{
    glClearColor(c.x(), c.y(), c.z(), 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Set2DMode(const int2 &screensize)
{
    Set2DMode(float2(0.f, 0.f), float2((float)screensize.x(), (float)screensize.y()));
}

void Set2DMode(const float2 &topleft, const float2 &bottomright)
{
    glDisable(GL_CULL_FACE);  
    glDisable(GL_DEPTH_TEST);

    object2view = float4x4_1;
    view2object = float4x4_1;

    view2clip = orthoGL(topleft.x(), bottomright.x(), bottomright.y(), topleft.y(), 1, -1);
}

void Set3DMode(float fovy, float ratio, float znear, float zfar)
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);  

    object2view = float4x4_1;
    view2object = float4x4_1;

    view2clip = perspectiveFov(fovy, ratio, znear, zfar, 1);
    view2clip *= float4x4(float4(1, -1, 1, 1)); // FIXME?
}

void OpenGLFrameStart(const int2 &screensize)
{
    glViewport(0, 0, screensize.x(), screensize.y());

    // force initial set.
    curblendmode = BLEND_NONE;
    SetBlendMode(BLEND_ALPHA);

    curcullmode = CULL_NONE;
    SetCullMode(CULL_BACK);

    curdepthwrite = false;
    SetDepthWrite(true);

    curdepthtest = false;
    SetDepthTest(true);

    //

    curcolor = float4(1);

    lights.clear();
}

void OpenGLInit()
{
    #if !defined(__IOS__) && !defined(ANDROID)
    //auto vers = (char *)glGetString(GL_VERSION);
    auto exts = (char *)glGetString(GL_EXTENSIONS);

    if (!strstr(exts, "GL_ARB_vertex_buffer_object")) throw string("no VBOs!");
    if (!strstr(exts, "GL_ARB_multitexture")) throw string("no multitexture!");
    if (!strstr(exts, "GL_ARB_vertex_program") || !strstr(exts, "GL_ARB_fragment_program")) throw string("no shaders!");
    //if (!strstr(exts, "GL_ARB_shading_language_100") || !strstr(exts, "GL_ARB_shader_objects") || !strstr(exts, "GL_ARB_vertex_shader") || !strstr(exts, "GL_ARB_fragment_shader")) throw string("no GLSL!");
    #endif

    #ifdef WIN32
    #define GLEXT(type, name) name = (type)SDL_GL_GetProcAddress(#name); if (!name) throw string("no " #name);
    GLBASEEXTS GLEXTS
    #undef GLEXT
    #endif

    #if !defined(__IOS__) && !defined(ANDROID)
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_MULTISAMPLE);
    #endif
}

