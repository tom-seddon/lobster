#include "stdafx.h"

#include "btBulletDynamicsCommon.h"

#include "vmdata.h"
#include "natreg.h"
#include <stddef.h>

void AddBulletPhysics()
{
    __nop();
}

AutoRegister __abp("bulletphysics",AddBulletPhysics);
