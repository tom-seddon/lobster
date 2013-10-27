#pragma once

#ifdef WIN32
    #define _CRT_SECURE_NO_WARNINGS
    #define _CRTDBG_MAP_ALLOC
    #include <stdlib.h>
    #include <crtdbg.h>
    #ifdef _DEBUG
        // don't know why plain old #undef new doesn't do the trick...
        #define DEBUG_NEW new
        #define new DEBUG_NEW
    #endif
#endif

#include <stdio.h>
#include <ctype.h>
#include <assert.h>
#include <math.h>
#include <string.h>

#include <string>
#include <map>
#include <vector>
#include <set>
#include <algorithm>
#include <iterator>
#include <functional>

#include <sstream>

using namespace std;

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;

#ifdef NULL
#undef NULL
#endif
#define NULL nullptr

// our universally used headers
#include "platform.h"
#include "tools.h"
#include "slaballoc.h"
#include "geom.h"
