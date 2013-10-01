#include "stdafx.h"

#include "vmdata.h"
#include "natreg.h"

#include "glinterface.h"
#include "glincludes.h"

#include <Box2D/Box2D.h>

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

    vector<Elem> elems;
    size_t firstfree;

    public:

    // there isn't actually any particularly nice way of doing this.
    void (*deleter)(T *p, void *context);
    void *deleter_context;

    IntResourceManager2(void (*deleter_)(T *, void *), void *deleter_context_) : firstfree(size_t(-1)), deleter(deleter_), deleter_context(deleter_context_)
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
        }
    }

    size_t Range() { return elems.size(); }     // if you wanted to iterate over all elements
};

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

static IntResourceManager2<b2Shape> *g_shapes;
static IntResourceManager2<b2Body> *g_bodies;
static b2World *g_world;
static DebugDraw *g_debug_draw;

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

static const int NUM_VELOCITY_ITERATIONS=8;
static const int NUM_POSITION_ITERATIONS=8;

void AddPhysics()
{
    __nop();

    STARTDECL(b2_createworld)(Value &g)
    {
        if(g_world)
            g_vm->BuiltinError("can only have one world at a time.");

        g_world=new b2World(Getb2Vec2DEC(g));

        g_bodies=new IntResourceManager2<b2Body>(&BodyDeleter, g_world);
        g_shapes=new IntResourceManager2<b2Shape>(&ScalarDeleter, 0);

        return Value();
    }
    ENDDECL1(b2_createworld,"g","V","","create world with given gravity.");

    STARTDECL(b2_frame)(Value &timestep_)
    {
        g_world->Step(timestep_.fval,NUM_VELOCITY_ITERATIONS,NUM_POSITION_ITERATIONS);
        return Value();
    }
    ENDDECL1(b2_frame,"timestep","F","","advances world by TIMESTEP seconds.");

    STARTDECL(b2_destroyworld)()
    {
        delete g_bodies;
        g_bodies=nullptr;

        delete g_shapes;
        g_shapes=nullptr;

        g_world->SetDebugDraw(nullptr);

        delete g_debug_draw;
        g_debug_draw=nullptr;

        delete g_world;
        g_world=nullptr;

        return Value();
    }
    ENDDECL0(b2_destroyworld,"","","","destroy world.");

    STARTDECL(b2_debugdraw)(Value &flags_)
    {
        if(!g_debug_draw)
        {
            g_debug_draw=new DebugDraw(LookupShader("vertexcolor"));

            g_world->SetDebugDraw(g_debug_draw);
        }

        g_debug_draw->Reset();

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

        g_debug_draw->SetFlags(flags);

        g_world->DrawDebugData();

        g_debug_draw->Draw();

        flags_.DEC();

        return Value();
    }
    ENDDECL1(b2_debugdraw,"flags","S","","debug draw world. FLAGS should include chars indicating what to draw - (S)hapes, (J)oints, (A)ABBs, (P)airs or (C)enters of mass. (GL must be initialised.)");

    STARTDECL(b2_createcircleshape)(Value &r_)
    {
        float r=GetFloatDEC(r_);

        b2CircleShape *shape=new b2CircleShape;

        shape->m_radius=r;

        return Value((int)g_shapes->Add(shape));
    }
    ENDDECL1(b2_createcircleshape,"r","A","I","create circle shape with given radius. returns integer id.");

    STARTDECL(b2_createbody)(Value &pos_,Value &angle_,Value &shape_,Value &mass_)
    {
        b2BodyDef body_def;

        float mass=GetFloatDEC(mass_);
        if(mass==0.f)
            body_def.type=b2_staticBody;
        else
            body_def.type=b2_dynamicBody;

        body_def.position=Getb2Vec2DEC(pos_);
        body_def.angle=GetFloatDEC(angle_);

        b2Body *body=g_world->CreateBody(&body_def);

        b2FixtureDef fixture_def;

        fixture_def.shape=g_shapes->Get((size_t)shape_.ival);

        body->CreateFixture(&fixture_def);

        b2MassData mass_data;

        mass_data.center=b2Vec2_zero;
        mass_data.mass=mass;
        mass_data.I=GetIFor1kgShape(fixture_def.shape)*mass_data.mass;

        body->SetMassData(&mass_data);

        int id=(int)g_bodies->Add(body);
        return Value(id);

    }
    ENDDECL4(b2_createbody,"initial_pos,initial_angle,shape_id,mass_kg","VAIA","I","create body with given properties. Center of mass is at (0,0) and shape is assumed to be of uniform density. Returns integer id.");

    STARTDECL(b2_destroybody)(Value &body_)
    {
        g_bodies->Delete(body_.ival);

        return Value();
    }
    ENDDECL1(b2_destroybody,"body","I","","delete given body.");

    STARTDECL(b2_getIfor1kgshape)(Value &shape_)
    {
        b2Shape *shape=g_shapes->Get((size_t)shape_.ival);

        float I=GetIFor1kgShape(shape);
        return Value(I);
    }
    ENDDECL1(b2_getIfor1kgshape,"shape","I","F","return moment of inertia for 1kg shape.");
}

AutoRegister __ap("physics",AddPhysics);
