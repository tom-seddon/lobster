#include "stdafx.h"

#include "btBulletDynamicsCommon.h"

#include "vmdata.h"
#include "natreg.h"
#include "glinterface.h"
#include "glincludes.h"

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static uchar GetByteColourFromFloat(float f)
{
    if(f<0.f)
        return 0;
    else if(f>1.f)
        return 255;
    else
        return (uchar)(f*255.f);
}

static byte4 GetByte4ColourFromBtVector3(const btVector3 &colour)
{
    float rf=colour.getX();
    float rg=colour.getY();
    float rb=colour.getZ();

    uchar r=GetByteColourFromFloat(rf);
    uchar g=GetByteColourFromFloat(rg);
    uchar b=GetByteColourFromFloat(rb);

    return byte4(r,g,b,255);
}

class BulletDebugDraw:
    public btIDebugDraw
{
    struct Vertex
    {
        float3 p;
        byte4 c;

        Vertex(const btVector3 &p_,byte4 c_):
            p(p_.getX(),p_.getY(),p_.getZ()),
            c(c_)
        {
        }
    };
public:
    explicit BulletDebugDraw(Shader *shader):
    m_shader(shader),
        m_debug_mode(0)
    {
        this->Reset();

    }

    void Reset()
    {
        m_vertices.clear();
        m_contact_colour=btVector3(rand()/(float)RAND_MAX,rand()/(float)RAND_MAX,rand()/(float)RAND_MAX);
    }

    void drawLine(const btVector3 &from,const btVector3 &to,const btVector3 &color)
    {
        byte4 c=GetByte4ColourFromBtVector3(color);
        m_vertices.push_back(Vertex(from,c));
        m_vertices.push_back(Vertex(to,c));
    }

    void drawContactPoint(const btVector3 &PointOnB,const btVector3 &normalOnB,btScalar distance,int lifeTime,const btVector3 &color)
    {
        btVector3 n=normalOnB.normalized()*.5f;
        this->drawLine(PointOnB,PointOnB+n,m_contact_colour);
    }

    void reportErrorWarning(const char *warningString)
    {
        // hardly used.
    }

    void draw3dText(const btVector3 &location,const char *textString)
    {
        // hardly used.
    }

    void setDebugMode(int debugMode)
    {
        m_debug_mode=debugMode;
    }

    int getDebugMode() const
    {
        return m_debug_mode;
    }

    void DoDraw()
    {
        int old_blend_mode=SetBlendMode(BLEND_ALPHA);

        if(!m_vertices.empty())
            RenderArray(m_shader,PRIM_LINES,m_vertices.size(),"PC",sizeof m_vertices[0],&m_vertices[0]);

        SetBlendMode((BlendMode)old_blend_mode);
    }
protected:
private:
    Shader *m_shader;
    std::vector<Vertex> m_vertices;
    btVector3 m_contact_colour;
    int m_debug_mode;
};

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

class BulletData
{
public:
    btDefaultCollisionConfiguration colli_conf;
    btCollisionDispatcher dispatcher;
    btDbvtBroadphase overlapping_pair_cache;
    btSequentialImpulseConstraintSolver solver;
    btDiscreteDynamicsWorld world;

    IntResourceManager<btCollisionShape> shapes;
    IntResourceManager<btRigidBody> bodies;

    BulletDebugDraw *debug_draw;

    BulletData();
    ~BulletData();
protected:
private:
    BulletData(const BulletData &);
    BulletData &operator=(const BulletData &);
};

BulletData::BulletData():
    dispatcher(&colli_conf),
    world(&dispatcher,&overlapping_pair_cache,&solver,&colli_conf),
    debug_draw(nullptr)
{
}

BulletData::~BulletData()
{
    // bullet not being terribly helpful here...
    for(size_t i=0;i<this->bodies.Range();++i)
    {
        if(btRigidBody *body=this->bodies.Get(i))
        {
            this->world.removeRigidBody(body);
        }
    }

    delete this->debug_draw;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static BulletData *g_bt;

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static void TestBullet()
{
    if(!g_bt)
        g_vm->BuiltinError("Bullet must be initialised");
}

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
        g_vm->BuiltinError("can't convert object of type "+std::string(g_vm->ProperTypeName(v_))+" to btVector3");
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

static btQuaternion GetBtQuaternionFromValue(const Value &v_)
{
    if(v_.type==V_VECTOR)
    {
        if(v_.vval->len==4)
        {
            btQuaternion v;

            v.setValue(GetFloat(v_.vval->at(0)),GetFloat(v_.vval->at(1)),GetFloat(v_.vval->at(2)),GetFloat(v_.vval->at(3)));

            return v;
        }
        else
        {
            g_vm->BuiltinError("can't convert vector of length "+std::string(inttoa(v_.vval->len))+" to btQuaternion");
            return btQuaternion(0.f,0.f,0.f,1.f);
        }
    }
    else
    {
        g_vm->BuiltinError("can't convert object of type "+std::string(g_vm->ProperTypeName(v_))+" to btQuaternion");
        return btQuaternion(0.f,0.f,0.f,1.f);
    }
}

static btQuaternion GetBtQuaternionFromValueDEC(Value &v_)
{
    const btQuaternion &v=GetBtQuaternionFromValue(v_);
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

static Value GetValueFromBtQuaternion(const btQuaternion &v)
{
    LVector *lv=g_vm->NewVector(4,V_VECTOR);

    lv->push(Value(v.getX()));
    lv->push(Value(v.getY()));
    lv->push(Value(v.getZ()));
    lv->push(Value(v.getW()));

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

template<class ObjectType,class MFnObjectType>
static Value DoFloatAction(Value &id_,Value &value_,const IntResourceManager<ObjectType> &resman,void (MFnObjectType::*set_mfn)(float))
{
    ObjectType *object=resman.Get(id_.ival);
    (object->*set_mfn)(value_.fval);
    return Value();
}

template<class ObjectType,class MFnResultType,class MFnObjectType>
static Value DoQuaternionFunc(Value &id_,const IntResourceManager<ObjectType> &resman,MFnResultType (MFnObjectType::*get_mfn)() const)
{
    ObjectType *object=resman.Get(id_.ival);
    const btQuaternion &value=(object->*get_mfn)();
    return GetValueFromBtQuaternion(value);
}

template<class ObjectType,class MFnResultType,class MFnObjectType>
static Value DoVec3Func(Value &id_,const IntResourceManager<ObjectType> &resman,MFnResultType (MFnObjectType::*get_mfn)() const)
{
    ObjectType *object=resman.Get(id_.ival);
    const btVector3 &value=(object->*get_mfn)();
    return GetValueFromBtVector3(value);
}

template<class ObjectType,class MFnObjectType,class MFnArgType>
static Value DoVec3Action(Value &id_,Value &value_,const IntResourceManager<ObjectType> &resman,void (MFnObjectType::*set_mfn)(MFnArgType))
{
    ObjectType *object=resman.Get(id_.ival);
    const btVector3 &value=GetBtVector3FromValueDEC(value_);
    (object->*set_mfn)(value);
    return Value();
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

void AddBulletPhysics()
{
    //////////////////////////////////////////////////////////////////////////
    // world create/destroy
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(bt_createworld)()
    {
        if(g_bt)
            g_vm->BuiltinError("can only have one Bullet world at a time.");

        g_bt=new BulletData;

        return Value();
    }
    ENDDECL0(bt_createworld,"","","",
        "create world");

    STARTDECL(bt_destroyworld)()
    {
        Delete(&g_bt);

        return Value();
    }
    ENDDECL0(bt_destroyworld,"","","",
        "destroy world");

    //////////////////////////////////////////////////////////////////////////
    // world configuration
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(bt_setgravity)(Value &g_)
    {
        TestBullet();

        const btVector3 &g=GetBtVector3FromValueDEC(g_);
        g_bt->world.setGravity(g);

        return Value();
    }
    ENDDECL1(bt_setgravity,"g","V","",
        "set world gravity");

    //////////////////////////////////////////////////////////////////////////
    // frame step
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(bt_frame)(Value &timestep_)
    {
        TestBullet();

        g_bt->world.stepSimulation(timestep_.fval,1);

        return Value();
    }
    ENDDECL1(bt_frame,"timestep","F","",
        "advances world by TIMESTEP seconds");

    //////////////////////////////////////////////////////////////////////////
    // debug draw
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(bt_debugdraw)(Value &flags_)
    {
        TestBullet();

        if(!g_bt->debug_draw)
        {
            g_bt->debug_draw=new BulletDebugDraw(LookupShader("vertexcolor"));
            g_bt->world.setDebugDrawer(g_bt->debug_draw);
        }

        int flags=0;

        for(const char *p=flags_.sval->str();*p!=0;++p)
        {
            char c=tolower(*p);

            if(c=='w')
                flags|=btIDebugDraw::DBG_DrawWireframe;
            else if(c=='a')
                flags|=btIDebugDraw::DBG_DrawAabb;
            else if(c=='p')
                flags|=btIDebugDraw::DBG_DrawContactPoints;
            //             else if(c=='')
            //                 flags|=DBG_NoDeactivation;
            else if(c=='c')
                flags|=btIDebugDraw::DBG_DrawConstraints;
            else if(c=='l')
                flags|=btIDebugDraw::DBG_DrawConstraintLimits;
            else if(c=='n')
                flags|=btIDebugDraw::DBG_DrawNormals;
        }

        g_bt->debug_draw->Reset();
        g_bt->debug_draw->setDebugMode(flags);
        g_bt->world.setDebugDrawer(g_bt->debug_draw);
        g_bt->world.debugDrawWorld();
        g_bt->world.setDebugDrawer(nullptr);

        g_bt->debug_draw->DoDraw();

        flags_.DEC();

        return Value();
    }
    ENDDECL1(bt_debugdraw,"flags","S","",
        "debug draw world. FLAGS should include chars indicating what to draw - (W)ireframe, (A)ABBs, Contact (P)oints, (C)onstraints, Constraint (L)imits, (N)ormals");

    //////////////////////////////////////////////////////////////////////////
    // shape create/destroy
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(bt_createsphereshape)(Value &radius_)
    {
        TestBullet();

        auto shape=new btSphereShape(radius_.fval);
        size_t id=g_bt->shapes.Add(shape);

        return Value((int)id);
    }
    ENDDECL1(bt_createsphereshape,"size","F","I",
        "create sphere shape with given radius. returns integer id");

    STARTDECL(bt_createboxshape)(Value &size_)
    {
        TestBullet();

        const btVector3 &size=GetBtVector3FromValueDEC(size_);

        auto shape=new btBoxShape(size*.5f);
        size_t id=g_bt->shapes.Add(shape);

        return Value((int)id);
    }
    ENDDECL1(bt_createboxshape,"size","V","I",
        "create box shape with given size. returns integer id");

    STARTDECL(bt_createstaticplaneshape)(Value &n_,Value &d_)
    {
        TestBullet();

        const btVector3 &n=GetBtVector3FromValueDEC(n_);

        auto shape=new btStaticPlaneShape(n,d_.fval);
        size_t id=g_bt->shapes.Add(shape);

        return Value((int)id);
    }
    ENDDECL2(bt_createstaticplaneshape,"n,d","VF","I",
        "create static plane shape with normal N and plane constant D. returns integer id");

    STARTDECL(bt_createconvexhullshape)(Value &pts_)
    {
        TestBullet();

        auto shape=new btConvexHullShape();

        for(int i=0;i<pts_.vval->len;++i)
        {
            const btVector3 &pt=GetBtVector3FromValue(pts_.vval->at(i));
            shape->addPoint(pt);
        }

        size_t id=g_bt->shapes.Add(shape);

        pts_.DEC();

        return Value((int)id);
    }
    ENDDECL1(bt_createconvexhullshape,"pts","V","I",
        "create convex hull shape with points from PTS, an array of float3");

    //////////////////////////////////////////////////////////////////////////
    // body create/destroy
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(bt_createbody)(Value &pos_,Value &orient_,Value &shape_,Value &mass_)
    {
        TestBullet();

        const btVector3 &pos=GetBtVector3FromValueDEC(pos_);
        const btQuaternion &orient=GetBtQuaternionFromValueDEC(orient_);
        btCollisionShape *shape=g_bt->shapes.Get(shape_.ival);

        btVector3 inertia;
        shape->calculateLocalInertia(mass_.fval,inertia);

        btRigidBody::btRigidBodyConstructionInfo ci(mass_.fval,nullptr,shape,inertia);

        ci.m_startWorldTransform.setOrigin(pos);
        ci.m_startWorldTransform.setRotation(orient);

        auto body=new btRigidBody(ci);

        //body->setCollisionFlags();

        g_bt->world.addRigidBody(body);

        size_t id=g_bt->bodies.Add(body);

        return Value((int)id);
    }
    ENDDECL4(bt_createbody,"pos,orient,shape,mass","VVIF","I",
        "create rigid body. initial position is POS, orientation ORIENT (a quat), shape id SHAPE and mass MASS. returns integer id");

    STARTDECL(bt_makebodykinematic)(Value &body_)
    {
        TestBullet();

        btRigidBody *body=g_bt->bodies.Get(body_.ival);

        int cf=body->getCollisionFlags();

        if(cf&btRigidBody::CF_STATIC_OBJECT)
        {
            cf&=~btRigidBody::CF_STATIC_OBJECT;
            cf|=btRigidBody::CF_KINEMATIC_OBJECT;

            body->setCollisionFlags(cf);

            body->setActivationState(DISABLE_DEACTIVATION);
        }

        return Value();
    }
    ENDDECL1(bt_makebodykinematic,"body","I","",
        "make body BODY kinematic");

    STARTDECL(bt_destroybody)(Value &body_)
    {
        TestBullet();

        btRigidBody *body=g_bt->bodies.Get(body_.ival);

        g_bt->world.removeRigidBody(body);

        g_bt->bodies.Delete(body_.ival);

        return Value();
    }
    ENDDECL1(bt_destroybody,"id","I","",
        "destroy rigid body with id ID");

    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(bt_applytorqueimpulse)(Value &body_,Value &impulse_)
    {
        return DoVec3Action(body_,impulse_,g_bt->bodies,&btRigidBody::applyTorqueImpulse);
    }
    ENDDECL2(bt_applytorqueimpulse,"body,impulse","IV","",
        "apply torque impulse to body");

    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(bt_getbodyorientation)(Value &body_)
    {
        return DoQuaternionFunc(body_,g_bt->bodies,&btRigidBody::getOrientation);
    }
    ENDDECL1(bt_getbodyorientation,"id","I","V",
        "get orientation of body ID as a quaternion");

    STARTDECL(bt_setbodyorientation)(Value &body_,Value &orient_)
    {
        TestBullet();

        btRigidBody *body=g_bt->bodies.Get(body_.ival);
        const btQuaternion &orient=GetBtQuaternionFromValueDEC(orient_);

        btTransform *t=&body->getWorldTransform();
        t->setRotation(orient);

        return Value();
    }
    ENDDECL2(bt_setbodyorientation,"id,orient","IV","",
        "set orientation of body ID as a quaternion");

    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(bt_getbodyvel)(Value &body_)
    {
        return DoVec3Func(body_,g_bt->bodies,&btRigidBody::getLinearVelocity);
    }
    ENDDECL1(bt_getbodyvel,"id","I","V",
        "get linear velocity of body ID");

    STARTDECL(bt_setbodyvel)(Value &body_,Value &vel_)
    {
        return DoVec3Action(body_,vel_,g_bt->bodies,&btRigidBody::setLinearVelocity);
    }
    ENDDECL2(bt_setbodyvel,"id,vel","IV","",
        "set linear velocity of body ID");

    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(bt_setbodygravity)(Value &body_,Value &gravity_)
    {
        return DoVec3Action(body_,gravity_,g_bt->bodies,&btRigidBody::setGravity);
    }
    ENDDECL2(bt_setbodygravity,"id,g","IV","",
        "set gravity for body ID to G");

    STARTDECL(bt_setbodyrestitution)(Value &body_,Value &restitution_)
    {
        return DoFloatAction(body_,restitution_,g_bt->bodies,&btRigidBody::setRestitution);
    }
    ENDDECL2(bt_setbodyrestitution,"id,restitution","IF","",
        "set coefficient of restitution for body ID to RESTITUTION");
}

AutoRegister __abp("bulletphysics",AddBulletPhysics);

// minitags::::func.regexp=^[ \t]+STARTDECL\(([A-Za-z0-9_]+)\)
