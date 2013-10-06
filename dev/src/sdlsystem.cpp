#include "stdafx.h"

#include "sdlincludes.h"
#include "sdlinterface.h"

SDL_Window *_sdl_window = NULL;
SDL_GLContext _sdl_context = NULL;


/*
mouse1 mouse2 mouse3...
backspace tab clear return pause escape space delete
! " # $ & ' ( ) * + , - . / 0 1 2 3 4 5 6 7 8 9 : ; < = > ? @ [ \ ] ^ _ ` 
a b c d e f g h i j k l m n o p q r s t u v w x y z
[0] [1] [2] [3] [4] [5] [6] [7] [8] [9] [.] [/] [*] [-] [+] 
enter equals up down right left insert home end page up page down
f1 f2 f3 f4 f5 f6 f7 f8 f9 f10 f11 f12 f13 f14 f15
numlock caps lock scroll lock right shift left shift right ctrl left ctrl right alt left alt
right meta left meta left super right super alt gr compose help print screen sys req break
*/

struct KeyState
{
    bool isdown;
    bool wentdown;

    bool isdown_lastframe;
    bool wentdown_lastframe;

    KeyState() : isdown(false), wentdown(false), isdown_lastframe(false), wentdown_lastframe(false) {};
};

map<string, KeyState> keymap;

int mousewheeldelta = 0;

int skipmousemotion = 3;


int frametime = 0, lastmillis = 0, frames = 0, starttime = 0;


int screenscalefactor = 1;  // FIXME: remove this

bool fullscreen = false;
bool cursor = true;
bool landscape = true;
bool minimized = false;

void updatebutton(string &name, bool on)
{
    auto kmit = keymap.find(name);
    auto ks = &(kmit != keymap.end() ? kmit : keymap.insert(make_pair(name, KeyState())).first)->second;
    ks->isdown = on;
    if (on) ks->wentdown = true;
}

void updatemousebutton(int button, int finger, bool on)
{
    string name = "mouse";
    name += '0' + (char)button;
    if (finger) name += '0' + (char)finger;
    updatebutton(name, on);
}

    
struct Finger
{
    SDL_FingerID id;
    int2 mousepos;
    int2 mousedelta;
    
    Finger() : id(0), mousepos(-1), mousedelta(0) {};
};

const int MAXFINGERS = 10;
Finger fingers[MAXFINGERS];

void clearfingers(bool delta)
{
    for (auto &f : fingers) (delta ? f.mousedelta : f.mousepos) = int2(0);
}

int findfinger(SDL_FingerID id, bool remove)
{
    for (auto &f : fingers) if (f.id == id)
    {
        if (remove) f.id = 0; // would be more correct to clear mouse position here, but that doesn't work with delayed touch.. would have to delay it too
        return &f - fingers;
    }
    if (remove) return MAXFINGERS - 1; // FIXME: this is masking a bug...
    assert(!remove);
    for (auto &f : fingers) if (!f.id)
    {
        f.id = id;
        return &f - fingers;
    }
    assert(0);
    return 0;
}

const int2 &GetFinger(int i, bool delta)
{
    auto &f = fingers[max(min(i, MAXFINGERS - 1), 0)];
    return delta ? f.mousedelta : f.mousepos;
}

int updatedragpos(SDL_TouchFingerEvent &e, Uint32 et, const int2 &screensize)
{
    int numfingers = SDL_GetNumTouchFingers(e.touchId);
    //assert(numfingers && e.fingerId < numfingers);
    for (int i = 0; i < numfingers; i++)
    {
        auto finger = SDL_GetTouchFinger(e.touchId, i);
        if (finger->id == e.fingerId)
        {
            // this is a bit clumsy as SDL has a list of fingers and so do we, but they work a bit differently
            int j = findfinger(e.fingerId, et == SDL_FINGERUP);
            auto &f = fingers[j];
            auto ep = float2(e.x, e.y);
            auto ed = float2(e.dx, e.dy);
            auto xy = ep * float2(screensize);
           
            f.mousepos = int2(xy * float(screenscalefactor));  // FIXME: converting back to int coords even though touch theoretically may have higher res
            f.mousedelta += int2(ed * float2(screensize));
            return j;
        }
    }
    //assert(0);
    return 0;
}


string SDLError(const char *msg)
{
    string s = string(msg) + ": " + SDL_GetError();
    DebugLog(1, s.c_str());
    SDLShutdown();
    return s;
}

int SDLHandleAppEvents(void *userdata, SDL_Event *event)
{
    switch (event->type)
    {
        case SDL_APP_TERMINATING:
            /* Terminate the app.
             Shut everything down before returning from this function.
             */
            return 0;
        case SDL_APP_LOWMEMORY:
            /* You will get this when your app is paused and iOS wants more memory.
             Release as much memory as possible.
             */
            return 0;
        case SDL_APP_WILLENTERBACKGROUND:
            minimized = true;
            /* Prepare your app to go into the background.  Stop loops, etc.
             This gets called when the user hits the home button, or gets a call.
             */
            return 0;
        case SDL_APP_DIDENTERBACKGROUND:
            /* This will get called if the user accepted whatever sent your app to the background.
             If the user got a phone call and canceled it, you'll instead get an SDL_APP_DIDENTERFOREGROUND event and restart your loops.
             When you get this, you have 5 seconds to save all your state or the app will be terminated.
             Your app is NOT active at this point.
             */
            return 0;
        case SDL_APP_WILLENTERFOREGROUND:
            /* This call happens when your app is coming back to the foreground.
             Restore all your state here.
             */
            return 0;
        case SDL_APP_DIDENTERFOREGROUND:
            /* Restart your loops here.
             Your app is interactive and getting CPU again.
             */
            minimized = false;
            return 0;
        default:
            /* No special processing, add it to the event queue */
            return 1;
    }
}

string SDLInit(const char *title, int2 &screensize)
{
    if (SDL_Init(SDL_INIT_VIDEO/* | SDL_INIT_AUDIO*/ | SDL_INIT_NOPARACHUTE) < 0)
    {
        return SDLError("Unable to initialize SDL");
    }

    SDL_SetEventFilter(SDLHandleAppEvents, NULL);
    
    DebugLog(-1, "SDL initialized...");

    SDL_LogSetAllPriority(SDL_LOG_PRIORITY_WARN);

    // on demand now
    //extern bool sfxr_init();
    //if (!sfxr_init())
    //   return SDLError("Unable to initialize audio");
        
    #if defined(__IOS__) || defined(ANDROID)
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    #else
    //certain older Intel HD GPUs and also Nvidia Quadro 1000M don't support 3.1 ? the 1000M is supposed to support 4.2
    //SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    //SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    #ifndef WIN32
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
    #endif
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    #endif
    
    //SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 0);      // set this if we're in 2D mode for speed on mobile?
    SDL_GL_SetAttribute(SDL_GL_RETAINED_BACKING, 1);    // because we redraw the screen each frame

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    DebugLog(-1, "SDL about to figure out display mode...");

    #if defined(__IOS__) || defined(ANDROID)
    landscape = screensize.x() >= screensize.y();
    int modes = SDL_GetNumDisplayModes(0);
    screensize = int2(0); 
    for (int i = 0; i < modes; i++)
    {
        SDL_DisplayMode mode;
        SDL_GetDisplayMode(0, i, &mode);
        //printf("mode: %d %d\n", mode.w, mode.h);
        if (landscape ? mode.w > screensize.x() : mode.h > screensize.y())
        {
            screensize = int2(mode.w, mode.h);
        }
    }

    DebugLog(-1, inttoa(screensize.x()));
    DebugLog(-1, inttoa(screensize.y()));
    DebugLog(-1, "SDL about to create window...");

    _sdl_window = SDL_CreateWindow(title,
                                    0, 0,
                                    screensize.x(), screensize.y(),
                                    SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_BORDERLESS);

    DebugLog(-1, _sdl_window ? "SDL window passed..." : "SDL window FAILED...");

    if (landscape) SDL_SetHint("SDL_HINT_ORIENTATIONS", "LandscapeLeft LandscapeRight");
    
    int ax = 0, ay = 0;
    SDL_GetWindowSize(_sdl_window, &ax, &ay);
    int2 actualscreensize(ax, ay);
    //screenscalefactor = screensize.x / actualscreensize.x;  // should be 2 on retina
    #ifdef __IOS__
        assert(actualscreensize == screensize);
        screensize = actualscreensize;
    #else
        screensize = actualscreensize;  // ANDROID
        DebugLog(-1, inttoa(screensize.x));
        DebugLog(-1, inttoa(screensize.y));
    #endif
    #else
    _sdl_window = SDL_CreateWindow(title,
                                    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                    screensize.x(), screensize.y(),
                                    SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    #endif

    if (!_sdl_window) 
        return SDLError("Unable to create window");

    DebugLog(-1, "SDL window opened...");


    _sdl_context = SDL_GL_CreateContext(_sdl_window);
    DebugLog(-1, _sdl_context ? "SDL context passed..." : "SDL context FAILED...");
    if (!_sdl_context) return SDLError("Unable to create OpenGL context");

    DebugLog(-1, "SDL OpenGL context created...");

    /*
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    */

    #ifndef __IOS__
        SDL_GL_SetSwapInterval(1);  // vsync on
    #endif

    starttime = SDL_GetTicks();

    return "";
}

void SDLShutdown() 
{
    if (_sdl_context) /*SDL_GL_DeleteContext(_sdl_context);*/ _sdl_context = NULL;  // FIXME: SDL gives ERROR: wglMakeCurrent(): The handle is invalid. 
    if (_sdl_window)  SDL_DestroyWindow(_sdl_window);     _sdl_window = NULL;

    SDL_Quit();
}

bool SDLFrame(int2 &screensize)
{
    for (;;)
    {
        int millis = SDL_GetTicks();
        frametime = millis-lastmillis;
        if (frametime < 10) { SDL_Delay(1); continue; }
        lastmillis = millis;
        frames++;
        break;
    }

    for (auto &it : keymap)
    {
        it.second.isdown_lastframe = it.second.isdown;
        it.second.wentdown_lastframe = it.second.wentdown;
        it.second.wentdown = false;
    }

    mousewheeldelta = 0;
    clearfingers(true);

    if (minimized)
        SDL_Delay(10);  // save CPU/battery
    else
        SDL_GL_SwapWindow(_sdl_window);
    
    //SDL_Delay(1000);
    
    if (!cursor) clearfingers(false);

    bool closebutton = false;

    SDL_Event event;
    while(SDL_PollEvent(&event)) switch(event.type)
    {
        case SDL_QUIT:
            closebutton = true;   
            break;

        case SDL_KEYDOWN:
        case SDL_KEYUP:
        {
            const char *kn = SDL_GetKeyName(event.key.keysym.sym);
            if (!*kn) break;
            string name = kn;
            std::transform(name.begin(), name.end(), name.begin(), ::tolower);
            updatebutton(name, event.key.state==SDL_PRESSED);
            break;
        }

        #if defined(__IOS__) || defined(ANDROID)
            
        // FIXME: if we're in cursor==0 mode, only update delta, not position
        case SDL_FINGERDOWN:
        {
            int i = updatedragpos(event.tfinger, event.type, screensize);
            updatemousebutton(1, i, true);
            break;
        }
        case SDL_FINGERUP:
        {
            int i = findfinger(event.tfinger.fingerId, true);
            updatemousebutton(1, i, false);
            break;
        }
            
        case SDL_FINGERMOTION:
        {
            updatedragpos(event.tfinger, event.type, screensize);
            break;
        }

        #else
                    
        case SDL_MOUSEBUTTONDOWN:
        case SDL_MOUSEBUTTONUP:
        {
            updatemousebutton(event.button.button, 0, event.button.state != 0);
            if (cursor)
            {
                fingers[0].mousepos = int2(event.button.x, event.button.y) * screenscalefactor;
            }
            break;
        }

        case SDL_MOUSEMOTION:
            fingers[0].mousedelta += int2(event.motion.xrel, event.motion.yrel);
            if (cursor)
            {
                fingers[0].mousepos = int2(event.motion.x, event.motion.y) * screenscalefactor;
            }
            else
            {
                //if (skipmousemotion) { skipmousemotion--; break; }
                //if (event.motion.x == screensize.x / 2 && event.motion.y == screensize.y / 2) break;

                //auto delta = int3(event.motion.xrel, event.motion.yrel);
                //fingers[0].mousedelta += delta;

                //auto delta = int3(event.motion.x, event.motion.y) - screensize / 2;
                //fingers[0].mousepos -= delta;

                //SDL_WarpMouseInWindow(_sdl_window, screensize.x / 2, screensize.y / 2);
            }
            break;

        case SDL_MOUSEWHEEL:
            mousewheeldelta += event.wheel.y;
            break;
                    
        #endif

        case SDL_WINDOWEVENT:
            switch (event.window.event)
            {
                case SDL_WINDOWEVENT_RESIZED:
                    screensize = int2(event.window.data1, event.window.data2);
                    // reload and bind shaders/textures here
                    break;

                case SDL_WINDOWEVENT_LEAVE:
                    // never gets hit?
                    /*
                    for (int i = 1; i <= 5; i++)
                        updatemousebutton(i, false);
                    */
                    break;
            }
            break;
            
        case SDL_WINDOWEVENT_MINIMIZED:
            //minimized = true;
            break;
            
        case SDL_WINDOWEVENT_MAXIMIZED:
        case SDL_WINDOWEVENT_RESTORED:
            /*
            #ifdef __IOS__
                SDL_Delay(10);  // IOS crashes in SDL_GL_SwapWindow if we start rendering straight away
            #endif
            minimized = false;
            */
            break;
    }

    // simulate mouse up events, since SDL won't send any if the mouse leaves the window while down
    // doesn't work
    /*
    for (int i = 1; i <= 5; i++)
        if (!(SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(i)))
            updatemousebutton(i, false);
    */

    /*
    if (SDL_GetMouseFocus() != _sdl_window)
    {
        int A = 1;
    }
    */

    return closebutton;
}

bool GetKS(const char *name, bool delta)
{
    auto ks = keymap.find(name);
    if (ks == keymap.end()) return false;
    #if defined(__IOS__) || defined(ANDROID)    // delayed results by one frame, that way they get 1 frame over finger hovering over target, which makes gl_hit work correctly
        return delta ? ks->second.wentdown_lastframe : ks->second.isdown_lastframe;
    #else
        return delta ? ks->second.wentdown : ks->second.isdown;
    #endif
}

void SDLTitle(const char *title) { SDL_SetWindowTitle(_sdl_window, title); }

float SDLTime() { return (lastmillis-starttime)/1000.0f; }

float SDLDeltaTime() { return frametime/1000.0f; }
int SDLWheelDelta() { return mousewheeldelta; }
bool SDLIsMinimized() { return minimized; }

bool SDLCursor(bool on)
{
    if (on != cursor)
    {
        if ((cursor = !cursor))
        {
            if (fullscreen) SDL_SetWindowGrab(_sdl_window, SDL_FALSE);
            SDL_ShowCursor(1);
            SDL_SetRelativeMouseMode(SDL_FALSE);
        }
        else
        {
            if (fullscreen) SDL_SetWindowGrab(_sdl_window, SDL_TRUE);
            SDL_ShowCursor(0);
            SDL_SetRelativeMouseMode(SDL_TRUE);
            clearfingers(false);
        }
    }
    return cursor;
}

bool SDLGrab(bool on)
{
    SDL_SetWindowGrab(_sdl_window, on ? SDL_TRUE : SDL_FALSE);
    return SDL_GetWindowGrab(_sdl_window) == SDL_TRUE;
}
