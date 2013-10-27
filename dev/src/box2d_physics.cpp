#include "stdafx.h"

#include "vmdata.h"
#include "natreg.h"

#include "glinterface.h"
#include "glincludes.h"

#include <Box2D/Box2D.h>

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

template <typename T> class IntResourceManager2
{
    struct Elem
    {
        T *t;
        size_t nextfree;

        Elem() : t(NULL), nextfree(size_t(-1)) {}
    };

    std::vector<Elem> elems;
    size_t firstfree;
    size_t numitems;

public:

    // there isn't actually any particularly nice way of doing this.
    void (*deleter)(T *p, void *context);
    void *deleter_context;

    IntResourceManager2(void (*deleter_)(T *, void *), void *deleter_context_) : firstfree(size_t(-1)), numitems(0), deleter(deleter_), deleter_context(deleter_context_)
    {
        elems.push_back(Elem());    // a NULL item at index 0 that can never be allocated/deleted
    }

    ~IntResourceManager2()
    {
        for (auto &e : elems)
            if (e.t)
                (*this->deleter)(e.t, this->deleter_context);
    }

    size_t Add(T *t)
    {
        assert(t);  // we can't store NULL pointers as elements, because we wouldn't be able to distinguish them from unallocated slots
        size_t i = elems.size();
        if (firstfree < i)
        {
            i = firstfree;
            firstfree = elems[i].nextfree;
        }
        else
        {
            elems.push_back(Elem());
        }
        ++numitems;
        elems[i].t = t;
        return i;
    }

    T *Get(size_t i)
    {
        return i < elems.size() ? elems[i].t : NULL;
    }

    void Delete(size_t i)
    {
        T *e = Get(i);

        if (e)
        {
            (*this->deleter)(elems[i].t, this->deleter_context);
            elems[i].t = NULL;
            elems[i].nextfree = firstfree;
            firstfree = i;
            --numitems;
        }
    }

    size_t Range() { return elems.size(); }     // if you wanted to iterate over all elements

    size_t NumItems() { return numitems; }
};

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static uint8_t GetByteColour(float f)
{
    if(f<0.f)
        return 0;
    else if(f>1.f)
        return 255;
    else
        return (uint8_t)(f*255.f);
}

static byte4 GetByte4Colour(const b2Color &c,uint8_t a)
{
    return byte4(GetByteColour(c.r),GetByteColour(c.g),GetByteColour(c.b),a);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

#define GL(X) (X)

class DebugDraw:
    public b2Draw
{
    struct Vertex
    {
        float3 p;
        byte4 c;

        Vertex(float x,float y,float z,byte4 c_):
            p(x,y,z),
            c(c_)
        {
        }

        Vertex(const b2Vec2 &p_,float z,byte4 c_):
            p(p_.x,p_.y,z),
            c(c_)
        {
        }
    };
public:
    explicit DebugDraw(Shader *shader):
    m_shader(shader)
    {
    }

    ~DebugDraw()
    {
    }

    void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
    {
        byte4 c=GetByte4Colour(color,255);
        float z=0.f;

        int32 i=0;
        const b2Vec2 *vi=vertices+i;

        int32 j=vertexCount-1;
        const b2Vec2 *vj=vertices+j;

        while(i<vertexCount)
        {
            m_lines.push_back(Vertex(*vi,z,c));
            m_lines.push_back(Vertex(*vj,z,c));

            j=i++;
            vj=vi++;
        }
    }

    /// Draw a solid closed polygon provided in CCW order.
    void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
    {
        byte4 c=GetByte4Colour(color,255)/2;
        float z=0.f;

        const b2Vec2 *v0=vertices;
        const b2Vec2 *vi=vertices+1;
        const b2Vec2 *vj=vertices+2;

        for(int j=2;j<vertexCount;++j)
        {
            m_lines.push_back(Vertex(*v0,z,c));
            m_lines.push_back(Vertex(*vi,z,c));
            m_lines.push_back(Vertex(*vj,z,c));

            ++vi;
            ++vj;
        }

        this->DrawPolygon(vertices,vertexCount,color);
    }

    /// Draw a circle.
    void DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color)
    {
        byte4 c=GetByte4Colour(color,255);
        float z=0.f;

        float x1=center.x+radius;
        float y1=center.y;

        float theta=DTHETA;

        for(int i=0;i<NUM_SEGMENTS;++i)
        {
            float x2=center.x+cosf(theta)*radius;
            float y2=center.y+sinf(theta)*radius;

            m_lines.push_back(Vertex(x1,y1,z,c));
            m_lines.push_back(Vertex(x2,y2,z,c));

            theta+=DTHETA;

            x1=x2;
            y1=y2;
        }
    }

    /// Draw a solid circle.
    void DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color)
    {
        byte4 c=GetByte4Colour(color,255)/2;
        float z=0.f;

        float x1=center.x+radius;
        float y1=center.y;

        float theta=DTHETA;

        for(int i=0;i<NUM_SEGMENTS;++i)
        {
            float x2=center.x+cosf(theta)*radius;
            float y2=center.y+sinf(theta)*radius;

            m_tris.push_back(Vertex(x1,y1,z,c));
            m_tris.push_back(Vertex(x2,y2,z,c));
            m_tris.push_back(Vertex(center.x,center.y,z,c));

            theta+=DTHETA;

            x1=x2;
            y1=y2;
        }

        this->DrawCircle(center,radius,color);

        c=GetByte4Colour(color,255);

        m_lines.push_back(Vertex(center,z,c));
        m_lines.push_back(Vertex(center+radius*axis,z,c));
    }

    /// Draw a line segment.
    void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color)
    {
        byte4 c=GetByte4Colour(color,255);
        float z=0.f;

        m_lines.push_back(Vertex(p1,z,c));
        m_lines.push_back(Vertex(p2,z,c));
    }

    /// Draw a transform. Choose your own length scale.
    /// @param xf a transform.
    void DrawTransform(const b2Transform& xf)
    {
        this->DrawSegment(xf.p,xf.p+AXIS_SCALE*xf.q.GetXAxis(),b2Color(1.f,0.f,0.f));
        this->DrawSegment(xf.p,xf.p+AXIS_SCALE*xf.q.GetYAxis(),b2Color(0.f,1.f,0.f));
    }

    void Reset()
    {
        m_tris.clear();
        m_lines.clear();
    }

    void Draw()
    {
        SetBlendMode(BLEND_ALPHA);

        if(!m_tris.empty())
            RenderArray(m_shader,PRIM_TRIS,m_tris.size(),"PC",sizeof(Vertex),&m_tris[0]);

        if(!m_lines.empty())
            RenderArray(m_shader,PRIM_LINES,m_lines.size(),"PC",sizeof(Vertex),&m_lines[0]);
    }
protected:
private:
    std::vector<Vertex> m_tris,m_lines;
    Shader *m_shader;

    static const int NUM_SEGMENTS=16;
    static const float DTHETA;
    static const float AXIS_SCALE;
};

const float DebugDraw::DTHETA=2.f*PI/NUM_SEGMENTS;
const float DebugDraw::AXIS_SCALE=.4f;

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

static b2Vec2 Getb2Vec2(const Value &v)
{
    if(v.type==V_VECTOR)
    {
        if(v.vval->len==2)
        {
            b2Vec2 result;

            result.x=GetFloat(v.vval->at(0));
            result.y=GetFloat(v.vval->at(1));

            return result;
        }
    }

    g_vm->BuiltinError("can't convert object of type "+std::string(g_vm->ProperTypeName(v))+" to b2Vec2");
    return b2Vec2_zero;
}

static b2Vec2 Getb2Vec2DEC(Value &v)
{
    const b2Vec2 &vec=Getb2Vec2(v);
    v.DEC();
    return vec;
}

static Value GetValue(const b2Vec2 &v)
{
    LVector *lv=g_vm->NewVector(2,V_VECTOR);

    lv->push(Value(v.x));
    lv->push(Value(v.y));

    return Value(lv);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static void Getb2AABB(b2AABB *aabb,const Value &v)
{
    if(v.type!=V_VECTOR||
        v.vval->len!=2)
    {
        g_vm->BuiltinError("can't convert object of type "+std::string(g_vm->ProperTypeName(v))+" to b2AABB");
    }

    const b2Vec2 &a=Getb2Vec2(v.vval->at(0));
    const b2Vec2 &b=Getb2Vec2(v.vval->at(1));

    aabb->lowerBound.Set(std::min(a.x,b.x),std::min(a.y,b.y));
    aabb->upperBound.Set(std::max(a.x,b.x),std::max(a.y,b.y));
}

static void Getb2AABBDEC(b2AABB *aabb,Value &v)
{
    Getb2AABB(aabb,v);
    v.DEC();
}

static Value GetValue(const b2AABB &aabb)
{
    LVector *lv=g_vm->NewVector(2,V_VECTOR);

    lv->push(GetValue(aabb.lowerBound));
    lv->push(GetValue(aabb.upperBound));

    return Value(lv);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static float GetIFor1kgShape(const b2Shape *shape)
{
    if(shape->m_type==b2Shape::e_circle)
    {
        auto c=(const b2CircleShape *)shape;

        return c->m_radius*c->m_radius;
    }
    else if(shape->m_type==b2Shape::e_polygon)
    {
        auto p=(const b2PolygonShape *)shape;

        float I=0.f;

        for(int i=0;i<p->m_count;++i)
            I+=p->m_vertices[i].LengthSquared()/p->m_count;

        return I;
    }

    g_vm->BuiltinError("unsupported shape type.");
    return 0.f;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static IntResourceManager2<b2Shape> *g_b2_shapes;
static IntResourceManager2<b2Body> *g_b2_bodies;
static b2World *g_b2_world;
static DebugDraw *g_b2_debug_draw;

template<class T>
static void ScalarDeleter(T *p, void *context)
{
    delete p;
}

static void BodyDeleter(b2Body *body, void *context)
{
    auto world=(b2World *)context;

    world->DestroyBody(body);
}

template<class ContType,class ObjType>
static Value AllocateID(std::vector<ContType> *cont,ObjType *obj)
{
    cont->push_back(obj);

    int id=(int)(cont->size()-1);
    return Value(id);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// body->vector
static Value DoB_V(Value &body_,const b2Vec2 &(b2Body::*get_mfn)() const)
{
    b2Body *body=g_b2_bodies->Get(body_.ival);

    const b2Vec2 &v=(body->*get_mfn)();
    return GetValue(v);
}

// body->float
static Value DoB_F(Value &body_,float (b2Body::*get_mfn)() const)
{
    b2Body *body=g_b2_bodies->Get(body_.ival);

    float f=(body->*get_mfn)();
    return Value(f);
}

// body,vector->void
static Value DoBV_(Value &body_,Value &v0_,void (b2Body::*mfn)(const b2Vec2 &))
{
    b2Body *body=g_b2_bodies->Get(body_.ival);
    const b2Vec2 &v0=Getb2Vec2DEC(v0_);

    (body->*mfn)(v0);
}

// body,float->void
static Value DoBV_(Value &body_,Value &f0_,void (b2Body::*mfn)(float))
{
    b2Body *body=g_b2_bodies->Get(body_.ival);

    (body->*mfn)(f0_.fval);
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

static const int NUM_VELOCITY_ITERATIONS=8;
static const int NUM_POSITION_ITERATIONS=8;

void AddBox2DPhysics()
{
    __nop();

    STARTDECL(b2_createworld)(Value &g)
    {
        if(g_b2_world)
            g_vm->BuiltinError("can only have one Box2D world at a time.");

        g_b2_world=new b2World(Getb2Vec2DEC(g));

        g_b2_bodies=new IntResourceManager2<b2Body>(&BodyDeleter, g_b2_world);
        g_b2_shapes=new IntResourceManager2<b2Shape>(&ScalarDeleter, 0);

        return Value();
    }
    ENDDECL1(b2_createworld,"g","V","","create world with given gravity.");

    STARTDECL(b2_frame)(Value &timestep_)
    {
        g_b2_world->Step(timestep_.fval,NUM_VELOCITY_ITERATIONS,NUM_POSITION_ITERATIONS);
        return Value();
    }
    ENDDECL1(b2_frame,"timestep","F","","advances world by TIMESTEP seconds.");

    STARTDECL(b2_destroyworld)()
    {
        delete g_b2_bodies;
        g_b2_bodies=nullptr;

        delete g_b2_shapes;
        g_b2_shapes=nullptr;

        g_b2_world->SetDebugDraw(nullptr);

        delete g_b2_debug_draw;
        g_b2_debug_draw=nullptr;

        delete g_b2_world;
        g_b2_world=nullptr;

        return Value();
    }
    ENDDECL0(b2_destroyworld,"","","","destroy world.");

    STARTDECL(b2_debugdraw)(Value &flags_)
    {
        if(!g_b2_debug_draw)
        {
            g_b2_debug_draw=new DebugDraw(LookupShader("vertexcolor"));

            g_b2_world->SetDebugDraw(g_b2_debug_draw);
        }

        g_b2_debug_draw->Reset();

        uint32 flags=0;

        for(const char *p=flags_.sval->str();*p!=0;++p)
        {
            char c=tolower(*p);

            if(c=='s')
                flags|=b2Draw::e_shapeBit;
            else if(c=='j')
                flags|=b2Draw::e_jointBit;
            else if(c=='a')
                flags|=b2Draw::e_aabbBit;
            else if(c=='p')
                flags|=b2Draw::e_pairBit;
            else if(c=='c')
                flags|=b2Draw::e_centerOfMassBit;
        }

        g_b2_debug_draw->SetFlags(flags);

        g_b2_world->DrawDebugData();

        g_b2_debug_draw->Draw();

        flags_.DEC();

        return Value();
    }
    ENDDECL1(b2_debugdraw,"flags","S","","debug draw world. FLAGS should include chars indicating what to draw - (S)hapes, (J)oints, (A)ABBs, (P)airs or (C)enters of mass. (GL must be initialised.)");

    STARTDECL(b2_createcircleshape)(Value &r_)
    {
        float r=GetFloatDEC(r_);

        b2CircleShape *shape=new b2CircleShape;

        shape->m_radius=r;

        return Value((int)g_b2_shapes->Add(shape));
    }
    ENDDECL1(b2_createcircleshape,"r","A","I","create circle shape with given radius. returns integer id.");

    STARTDECL(b2_createpolygonshape)(Value &pts_)
    {
        b2Vec2 vertices[b2_maxPolygonVertices];
        int numvertices=0;

        if(pts_.vval->len>b2_maxPolygonVertices)
            g_vm->BuiltinError("too many vertices supplied to b2_createpolygonshape");

        for(int i=0;i<pts_.vval->len;++i)
            vertices[numvertices++]=Getb2Vec2(pts_.vval->at(i));

        b2PolygonShape *shape=new b2PolygonShape;

        shape->Set(vertices,numvertices);

        pts_.DEC();

        int id=(int)g_b2_shapes->Add(shape);
        return Value(id);
    }
    ENDDECL1(b2_createpolygonshape,"pts","V","I","create polygon shape with given points. returns integer id.");

    STARTDECL(b2_createboxshape)(Value &size_)
    {
        const b2Vec2 &size=Getb2Vec2DEC(size_);

        b2PolygonShape *shape=new b2PolygonShape;
        shape->SetAsBox(size.x*.5f,size.y*.5f);

        int id=(int)g_b2_shapes->Add(shape);
        return Value(id);
    }
    ENDDECL1(b2_createboxshape,"size","V","I","create axis-aligned box shape with given size. returns integer id.");

    STARTDECL(b2_createbody)(Value &pos_,Value &angle_,Value &shape_,Value &mass_)
    {
        b2BodyDef body_def;

        const float angle=angle_.fval;
        const float mass=mass_.fval;

        if(mass==0.f)
            body_def.type=b2_staticBody;
        else
            body_def.type=b2_dynamicBody;

        body_def.position=Getb2Vec2DEC(pos_);
        body_def.angle=GetFloatDEC(angle_);

        b2Body *body=g_b2_world->CreateBody(&body_def);

        b2FixtureDef fixture_def;

        fixture_def.shape=g_b2_shapes->Get((size_t)shape_.ival);

        body->CreateFixture(&fixture_def);

        b2MassData mass_data;

        mass_data.center=b2Vec2_zero;
        mass_data.mass=mass;
        mass_data.I=GetIFor1kgShape(fixture_def.shape)*mass_data.mass;

        body->SetMassData(&mass_data);

        int id=(int)g_b2_bodies->Add(body);
        return Value(id);

    }
    ENDDECL4(b2_createbody,"initial_pos,initial_angle,shape_id,mass_kg","VFIF","I","create body with given properties. Center of mass is at (0,0), and it is assigned a single fixture holding the shape (assumed to be of uniform density). Returns integer id.");

    STARTDECL(b2_destroybody)(Value &body_)
    {
        g_b2_bodies->Delete(body_.ival);

        return Value();
    }
    ENDDECL1(b2_destroybody,"body","I","","delete given body.");

    STARTDECL(b2_getIfor1kgshape)(Value &shape_)
    {
        b2Shape *shape=g_b2_shapes->Get((size_t)shape_.ival);

        float I=GetIFor1kgShape(shape);
        return Value(I);
    }
    ENDDECL1(b2_getIfor1kgshape,"shape","I","F","return moment of inertia for 1kg shape.");

    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(b2_getposition)(Value &body_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        b2Vec2 result=body->GetPosition();
        return GetValue(result);
    }
    ENDDECL1(b2_getposition,"body","I","V","get body position.");

    STARTDECL(b2_getworldcenter)(Value &body_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        b2Vec2 result=body->GetWorldCenter();
        return GetValue(result);
    }
    ENDDECL1(b2_getworldcenter,"body","I","V","get body world center.");

    STARTDECL(b2_getlocalcenter)(Value &body_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        b2Vec2 result=body->GetLocalCenter();
        return GetValue(result);
    }
    ENDDECL1(b2_getlocalcenter,"body","I","V","get body local center.");

    STARTDECL(b2_getangle)(Value &body_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        
        float angle=body->GetAngle();

        return Value(angle);
    }
    ENDDECL1(b2_getangle,"body","I","F","get body angle, in radians.");

    STARTDECL(b2_settransform)(Value &body_,Value &pos_,Value &angle_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        const b2Vec2 &pos=Getb2Vec2DEC(pos_);
        float angle=angle_.fval;

        body->SetTransform(pos,angle);

        return Value();
    }
    ENDDECL3(b2_settransform,"body,worldpos,angle","IVF","","set body transform. Angle is in radians.");

//     STARTDECL(b2_setangle)(Value &body_,Value &angle_)
//     {
//         b2Body *body=g_bodies->Get(body_.ival);
//         body->SetAngle(angle_.fval);
//     }
//     ENDDECL1(b2_setangle,"body","I","F","set body angle.");

    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(b2_getangularvelocity)(Value &body_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);

        float result=body->GetAngularVelocity();

        return Value(result);
    }
    ENDDECL1(b2_getangularvelocity,"body","I","F","get body angular velocity.");

    STARTDECL(b2_setangularvelocity)(Value &body_,Value &angularvelocity_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        body->SetAngularVelocity(angularvelocity_.fval);
        return Value();
    }
    ENDDECL2(b2_setangularvelocity,"body,angularvelocity","IF","F","set body angular velocity.");

    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

       STARTDECL(b2_getlinearvelocity)(Value &body_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        b2Vec2 result=body->GetLinearVelocity();
        return GetValue(result);
    }
    ENDDECL1(b2_getlinearvelocity,"body","I","V","get body linear velocity.");

    STARTDECL(b2_setlinearvelocity)(Value &body_,Value &linearvelocity_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        const b2Vec2 &linearvelocity=Getb2Vec2DEC(linearvelocity_);
        body->SetLinearVelocity(linearvelocity);
        return Value();
    }
    ENDDECL2(b2_setlinearvelocity,"body,linearvelocity","IV","","set body linear velocity.");

    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(b2_wake)(Value &body_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        body->SetAwake(true);
        return Value();
    }
    ENDDECL1(b2_wake,"body","I","","wake body up, if it's sleeping.");

    STARTDECL(b2_applyforce)(Value &body_,Value &worldforce_,Value &worldpt_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        const b2Vec2 &worldforce=Getb2Vec2DEC(worldforce_);
        const b2Vec2 &worldpt=Getb2Vec2DEC(worldpt_);
        body->ApplyForce(worldforce,worldpt,false);
        return Value();
    }
    ENDDECL3(b2_applyforce,"body,worldforce,worldpt","IVV","","apply force to body.");

    STARTDECL(b2_applyforcetocenter)(Value &body_,Value &worldforce_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        const b2Vec2 &worldforce=Getb2Vec2DEC(worldforce_);
        body->ApplyForceToCenter(worldforce,false);
        return Value();
    }
    ENDDECL2(b2_applyforcetocenter,"body,worldforce","IV","","apply force to body's center of mass.");

    STARTDECL(b2_applytorque)(Value &body_,Value &torque_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        body->ApplyTorque(torque_.fval,false);
        return Value();
    }
    ENDDECL2(b2_applytorque,"body,torque","IF","","apply torque to body.");

    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(b2_getlinearvelocityfromworldpoint)(Value &body_,Value &worldpt_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        const b2Vec2 &worldpt=Getb2Vec2DEC(worldpt_);
        const b2Vec2 &linearvelocity=body->GetLinearVelocityFromWorldPoint(worldpt);
        return GetValue(linearvelocity);
    }
    ENDDECL2(b2_getlinearvelocityfromworldpoint,"body,worldpt","IV","V","get world space linear velocity from world space position.");

    STARTDECL(b2_getlinearvelocityfromlocalpoint)(Value &body_,Value &localpt_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);
        const b2Vec2 &localpt=Getb2Vec2DEC(localpt_);
        const b2Vec2 &linearvelocity=body->GetLinearVelocityFromLocalPoint(localpt);
        return GetValue(linearvelocity);
    }
    ENDDECL2(b2_getlinearvelocityfromlocalpoint,"body,localpt","IV","V","get world space linear velocity from local space position.");

    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////

    STARTDECL(b2_getaabb)(Value &body_)
    {
        b2Body *body=g_b2_bodies->Get(body_.ival);

        b2AABB total_aabb;
        total_aabb.lowerBound.Set(FLT_MAX,FLT_MAX);
        total_aabb.upperBound.Set(-FLT_MAX,-FLT_MAX);

        const b2Transform *t=&body->GetTransform();

        for(b2Fixture *fixture=body->GetFixtureList();fixture;fixture=fixture->GetNext())
        {
            b2Shape *shape=fixture->GetShape();

            for(int32 i=0;i<shape->GetChildCount();++i)
            {
                b2AABB aabb;
                shape->ComputeAABB(&aabb,*t,i);

                total_aabb.Combine(aabb);
            }
        }

        if(!total_aabb.IsValid())
        {
            total_aabb.lowerBound=t->p;
            total_aabb.upperBound=t->p;
        }

        return GetValue(total_aabb);
    }
    ENDDECL1(b2_getaabb,"body","I","V","get world space AABB of body.");

    STARTDECL(b2_aabbsintersect)(Value &a_,Value &b_)
    {
        b2AABB a,b;
        Getb2AABBDEC(&a,a_);
        Getb2AABBDEC(&b,b_);

        bool intersect=true;

        if(a.upperBound.x<b.lowerBound.x||b.upperBound.x<a.lowerBound.x)
            intersect=false;//X disjoint
        else if(a.upperBound.y<b.lowerBound.y||b.upperBound.y<a.lowerBound.y)
            intersect=false;//Y disjoint

        return Value(intersect);
    }
    ENDDECL2(b2_aabbsintersect,"a,b","VV","I","returns true if AABBs intersect.");

    STARTDECL(b2_aabbcontainsaabb)(Value &outer_,Value &inner_)
    {
        b2AABB outer,inner;
        Getb2AABBDEC(&outer,outer_);
        Getb2AABBDEC(&inner,inner_);

        bool contains=outer.Contains(inner);
        return Value(contains);
    }
    ENDDECL2(b2_aabbcontainsaabb, "outer,inner", "VV", "I", "returns true if inner AABB is wholly inside outer AABB.");

    STARTDECL(b2_debuginfo)()
    {
        LVector *lv=g_vm->NewVector(2,V_VECTOR);

        lv->push(Value((int)g_b2_bodies->NumItems()));
        lv->push(Value((int)g_b2_shapes->NumItems()));

        return Value(lv);
    }
    ENDDECL0(b2_debuginfo, "", "" ,"S", "return some box2D debug info.");
}

AutoRegister __ab2p("box2dphysics",AddBox2DPhysics);

// minitags::::func.regexp=^[ \t]+STARTDECL\(([A-Za-z0-9_]+)\)
