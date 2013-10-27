#include "stdafx.h"

#include "btBulletDynamicsCommon.h"

#include "vmdata.h"
#include "natreg.h"

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

struct WorldData
{
    btDefaultCollisionConfiguration colli_conf;
    btCollisionDispatcher dispatcher;
    btDbvtBroadphase overlapping_pair_cache;
    btSequentialImpulseConstraintSolver solver;
    btDiscreteDynamicsWorld world;

    IntResourceManager<btCollisionShape> shapes;
    IntResourceManager<btRigidBody> bodies;

    WorldData();
};

WorldData::WorldData():
dispatcher(&colli_conf),
world(&dispatcher,&overlapping_pair_cache,&solver,&colli_conf)
{
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static WorldData *g_bt_world;

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static float GetFloat(const Value &v)
{
    if(v.type==V_INT)
        return (float)v.ival;
    else if(v.type==V_FLOAT)
        return v.fval;

    g_vm->BuiltinError("can't convert object of type "+std::string(g_vm->ProperTypeName(v))+" to float");
    return 0.f;
}

static float GetFloatDEC(Value &v)
{
    float f=GetFloat(v);
    v.DEC();
    return f;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static btVector3 GetBtVector3FromValue(const Value &v_)
{
    if(v_.type==V_VECTOR)
    {
        if(v_.vval->len==3)
        {
            btVector3 v;

            v.setX(GetFloat(v_.vval->at(0)));
            v.setY(GetFloat(v_.vval->at(1)));
            v.setZ(GetFloat(v_.vval->at(2)));

            return v;
        }
        else
        {
            g_vm->BuiltinError("can't convert vector of length "+std::string(inttoa(v_.vval->len))+" to btVector3");
            return btVector3(0.f, 0.f, 0.f);
        }
    }
    else
    {
        g_vm->BuiltinError("can't convert object of type "+std::string(g_vm->ProperTypeName(v_))+" to b2Vec2");
        return btVector3(0.f, 0.f, 0.f);
    }
}

static btVector3 GetBtVector3FromValueDEC(Value &v_)
{
    const btVector3 &v=GetBtVector3FromValue(v_);
    v_.DEC();
    return v;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static Value GetValueFromBtVector3(const btVector3 &v)
{
    LVector *lv=g_vm->NewVector(3,V_VECTOR);

    lv->push(Value(v.getX()));
    lv->push(Value(v.getY()));
    lv->push(Value(v.getZ()));

    return Value(lv);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

template<class T>
static void Delete(T **p)
{
    delete *p;
    *p=nullptr;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void AddBulletPhysics()
{
    STARTDECL(bt_createworld)()
    {
        if(g_bt_world)
            g_vm->BuiltinError("can only have one Bullet world at a time.");

        g_bt_world=new WorldData;

        return Value();
    }
    ENDDECL0(bt_createworld,"","","","create world");

    STARTDECL(bt_destroyworld)()
    {
        Delete(&g_bt_world);

        return Value();
    }
    ENDDECL0(bt_destroyworld,"","","","destroy world");

    STARTDECL(bt_setgravity)(Value &g_)
    {
        const btVector3 &g=GetBtVector3FromValueDEC(g_);
        g_bt_world->world.setGravity(g);

        return Value();
    }
    ENDDECL1(bt_setgravity,"g","V","","set world gravity");

    STARTDECL(bt_frame)(Value &timestep_)
    {
        g_bt_world->world.stepSimulation(timestep_.fval,1);

        return Value();
    }
    ENDDECL1(bt_frame,"timestep","F","","advances world by TIMESTEP seconds");
}

AutoRegister __abp("bulletphysics",AddBulletPhysics);
