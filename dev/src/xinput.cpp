#include "stdafx.h"

#include "vmdata.h"
#include "natreg.h"

#include <windows.h>
#include <xinput.h>

// xinput.lib doesn't work with Windows 7 + VS2012. see, e.g.,
// http://stackoverflow.com/questions/12181796/xinput-does-not-work-on-windows-7-with-visual-studio-2012
#pragma comment(lib,"xinput9_1_0.lib")

static float Remap(int value,int a,int b)
{
	float scale=1.f;
	if(value<0)
	{
		value=-value;
		scale=-scale;
	}

	float result;
	if(value<a)
		result=0.f;
	else if(value>b)
		result=1.f;
	else
		result=(value-a)/(float)(b-a);

	result*=scale;
	return result;
}

static const int DZ_MIN_THUMB=5000;
static const int DZ_MAX_THUMB=28000;

static const int DZ_MIN_TRIGGER=30;
static const int DZ_MAX_TRIGGER=255;

/*
XINPUT_GAMEPAD_DPAD_UP:==0x0001
XINPUT_GAMEPAD_DPAD_DOWN:==0x0002
XINPUT_GAMEPAD_DPAD_LEFT:==0x0004
XINPUT_GAMEPAD_DPAD_RIGHT:==0x0008
XINPUT_GAMEPAD_START:==0x0010
XINPUT_GAMEPAD_BACK:==0x0020
XINPUT_GAMEPAD_LEFT_THUMB:==0x0040
XINPUT_GAMEPAD_RIGHT_THUMB:==0x0080
XINPUT_GAMEPAD_LEFT_SHOULDER:==0x0100
XINPUT_GAMEPAD_RIGHT_SHOULDER:==0x0200
XINPUT_GAMEPAD_A:==0x1000
XINPUT_GAMEPAD_B:==0x2000
XINPUT_GAMEPAD_X:==0x4000
XINPUT_GAMEPAD_Y:==0x8000
*/

void AddXInput()
{
	STARTDECL(xinput_getstate)(Value &index)
	{
		XINPUT_STATE xis;
		DWORD result=XInputGetState(index.ival,&xis);
		if(result!=ERROR_SUCCESS)
			memset(&xis,0,sizeof xis);

		LVector *lv=g_vm->NewVector(2,V_VECTOR);
		lv->push(Value(Remap(xis.Gamepad.sThumbLX,DZ_MIN_THUMB,DZ_MAX_THUMB)));
		lv->push(Value(Remap(xis.Gamepad.sThumbLY,DZ_MIN_THUMB,DZ_MAX_THUMB)));

		LVector *rv=g_vm->NewVector(2,V_VECTOR);
		rv->push(Value(Remap(xis.Gamepad.sThumbRX,DZ_MIN_THUMB,DZ_MAX_THUMB)));
		rv->push(Value(Remap(xis.Gamepad.sThumbRY,DZ_MIN_THUMB,DZ_MAX_THUMB)));

		LVector *tv=g_vm->NewVector(2,V_VECTOR);
		tv->push(Value(Remap(xis.Gamepad.bLeftTrigger,DZ_MIN_TRIGGER,DZ_MAX_TRIGGER)));
		tv->push(Value(Remap(xis.Gamepad.bRightTrigger,DZ_MIN_TRIGGER,DZ_MAX_TRIGGER)));

		LVector *v=g_vm->NewVector(4,V_VECTOR);
		v->push(Value(lv));
		v->push(Value(rv));
		v->push(Value(tv));
		v->push(Value(xis.Gamepad.wButtons));

		return Value(v);
	}
	ENDDECL1(xinput_getstate,"index","I","V",
		"get XInput state for given joypad: [[lx,ly],[rx,ry],[lt,rt],buttons]");
}

AutoRegister __axi("xinput",AddXInput);
// minitags::::func.regexp=^[ \t]+STARTDECL\(([A-Za-z0-9_]+)\)
