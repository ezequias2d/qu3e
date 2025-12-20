//------------------------------------------------------------------------------
/**
@file	q3Body.cpp

@author Randy Gaul, Ezequias Silva
@date   19/12/2025
Copyright (c) 2014 Randy Gaul http://www.randygaul.net
Copyright (c) 2025 Ezequias Silva https://github.com/ezequias2d

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
    1. The origin of this software must not be misrepresented; you must not
      claim that you wrote the original software. If you use this software
      in a product, an acknowledgment in the product documentation would be
      appreciated but is not required.
    2. Altered source versions must be plainly marked as such, and must not
      be misrepresented as being the original software.
    3. This notice may not be removed or altered from any source distribution.
*/
//------------------------------------------------------------------------------

#include "q3Body.h"
#include "../broadphase/q3BroadPhase.h"
#include "../collision/q3Box.h"
#include "../collision/q3Capsule.h"
#include "../collision/q3Sphere.h"
#include "../scene/q3Scene.h"
#include "q3Contact.h"

//------------------------------------------------------------------------------
// q3Body
//------------------------------------------------------------------------------
q3Body::q3Body(const q3BodyDef &def, q3Scene *scene)
{
    m_linearVelocity  = def.linearVelocity;
    m_angularVelocity = def.angularVelocity;
    q3Identity(m_force);
    q3Identity(m_torque);
    m_q.Set(q3Normalize(def.axis), def.angle);
    m_tx.rotation    = m_q.ToMat3();
    m_tx.position    = def.position;
    m_sleepTime      = r32(0.0);
    m_gravityScale   = def.gravityScale;
    m_layers         = def.layers;
    m_userData       = def.userData;
    m_scene          = scene;
    m_flags          = 0;
    m_linearDamping  = def.linearDamping;
    m_angularDamping = def.angularDamping;

    if (def.bodyType == eDynamicBody)
        m_flags |= q3Body::eDynamic;

    else
    {
        if (def.bodyType == eStaticBody)
        {
            m_flags |= q3Body::eStatic;
            q3Identity(m_linearVelocity);
            q3Identity(m_angularVelocity);
            q3Identity(m_force);
            q3Identity(m_torque);
        }

        else if (def.bodyType == eKinematicBody)
            m_flags |= q3Body::eKinematic;
    }

    if (def.allowSleep)
        m_flags |= eAllowSleep;

    if (def.awake)
        m_flags |= eAwake;

    if (def.active)
        m_flags |= eActive;

    if (def.lockAxisX)
        m_flags |= eLockAxisX;

    if (def.lockAxisY)
        m_flags |= eLockAxisY;

    if (def.lockAxisZ)
        m_flags |= eLockAxisZ;

    m_shapes      = NULL;
    m_contactList = NULL;
}

//------------------------------------------------------------------------------
const q3Box *q3Body::AddBox(const q3BoxDef &def)
{
    q3AABB aabb;
    q3Shape *shape = (q3Shape *)m_scene->m_heap.Allocate(sizeof(q3Box));
    q3Box *box     = (q3Box *)shape;
    new (box) q3Box();
    box->m_type = eBox;
    box->local  = def.m_tx;
    box->e      = def.m_e;
    box->next   = m_shapes;
    m_shapes    = box;
    box->ComputeAABB(m_tx, &aabb);

    box->body        = this;
    box->friction    = def.m_friction;
    box->restitution = def.m_restitution;
    box->density     = def.m_density;
    box->sensor      = def.m_sensor;

    CalculateMassData();

    m_scene->m_contactManager.m_broadphase.InsertShape(box, aabb);
    m_scene->m_newShape = true;

    return box;
}

//------------------------------------------------------------------------------
const q3Sphere *q3Body::AddSphere(const q3SphereDef &def)
{
    q3AABB aabb;
    q3Shape *shape   = (q3Shape *)m_scene->m_heap.Allocate(sizeof(q3Sphere));
    q3Sphere *sphere = (q3Sphere *)shape;
    new (sphere) q3Sphere();
    sphere->m_type = eSphere;
    sphere->local  = def.local;
    sphere->radius = def.radius;
    sphere->next   = m_shapes;
    m_shapes       = sphere;
    sphere->ComputeAABB(m_tx, &aabb);

    sphere->body        = this;
    sphere->friction    = def.friction;
    sphere->restitution = def.restitution;
    sphere->density     = def.density;
    sphere->sensor      = def.sensor;

    CalculateMassData();

    m_scene->m_contactManager.m_broadphase.InsertShape(sphere, aabb);
    m_scene->m_newShape = true;

    return sphere;
}

//------------------------------------------------------------------------------
const q3Capsule *q3Body::AddCapsule(const q3CapsuleDef &def)
{
    q3AABB aabb;
    q3Shape *shape     = (q3Shape *)m_scene->m_heap.Allocate(sizeof(q3Capsule));
    q3Capsule *capsule = (q3Capsule *)shape;
    new (capsule) q3Capsule();
    capsule->m_type = eCapsule;
    capsule->local  = def.local;
    capsule->radius = def.radius;
    capsule->height = def.height;
    capsule->next   = m_shapes;
    m_shapes        = capsule;
    capsule->ComputeAABB(m_tx, &aabb);

    capsule->body        = this;
    capsule->friction    = def.friction;
    capsule->restitution = def.restitution;
    capsule->density     = def.density;
    capsule->sensor      = def.sensor;

    CalculateMassData();

    m_scene->m_contactManager.m_broadphase.InsertShape(capsule, aabb);
    m_scene->m_newShape = true;

    return capsule;
}

//------------------------------------------------------------------------------
void q3Body::RemoveShape(q3Shape *shape)
{
    assert(shape);
    assert(shape->body == this);

    q3Shape *node = m_shapes;

    bool found = false;
    if (node == shape)
    {
        m_shapes = node->next;
        found    = true;
    }

    else
    {
        while (node)
        {
            if (node->next == shape)
            {
                node->next = shape->next;
                found      = true;
                break;
            }

            node = node->next;
        }
    }

    // This shape was not connected to this body.
    assert(found);

    // Remove all contacts associated with this shape
    q3ContactEdge *edge = m_contactList;
    while (edge)
    {
        q3ContactConstraint *contact = edge->constraint;
        edge                         = edge->next;

        q3Shape *A = contact->A;
        q3Shape *B = contact->B;

        if (shape == A || shape == B)
            m_scene->m_contactManager.RemoveContact(contact);
    }

    m_scene->m_contactManager.m_broadphase.RemoveShape(shape);

    CalculateMassData();

    m_scene->m_heap.Free((void *)shape);
}

//------------------------------------------------------------------------------
void q3Body::RemoveAllShapes()
{
    while (m_shapes)
    {
        q3Shape *next = m_shapes->next;

        m_scene->m_contactManager.m_broadphase.RemoveShape(m_shapes);
        m_scene->m_heap.Free((void *)m_shapes);

        m_shapes = next;
    }

    m_scene->m_contactManager.RemoveContactsFromBody(this);
}

//------------------------------------------------------------------------------
void q3Body::ApplyLinearForce(const q3Vec3 &force)
{
    m_force += force * m_mass;

    SetToAwake();
}

//------------------------------------------------------------------------------
void q3Body::ApplyForceAtWorldPoint(const q3Vec3 &force, const q3Vec3 &point)
{
    m_force += force * m_mass;
    m_torque += q3Cross(point - m_worldCenter, force);

    SetToAwake();
}

//------------------------------------------------------------------------------
void q3Body::ApplyLinearImpulse(const q3Vec3 &impulse)
{
    m_linearVelocity += impulse * m_invMass;

    SetToAwake();
}

//------------------------------------------------------------------------------
void q3Body::ApplyLinearImpulseAtWorldPoint(const q3Vec3 &impulse,
                                            const q3Vec3 &point)
{
    m_linearVelocity += impulse * m_invMass;
    m_angularVelocity +=
        m_invInertiaWorld * q3Cross(point - m_worldCenter, impulse);

    SetToAwake();
}

//------------------------------------------------------------------------------
void q3Body::ApplyTorque(const q3Vec3 &torque) { m_torque += torque; }

//------------------------------------------------------------------------------
void q3Body::SetToAwake()
{
    if (!(m_flags & eAwake))
    {
        m_flags |= eAwake;
        m_sleepTime = r32(0.0);
    }
}

//------------------------------------------------------------------------------
void q3Body::SetToSleep()
{
    m_flags &= ~eAwake;
    m_sleepTime = r32(0.0);
    q3Identity(m_linearVelocity);
    q3Identity(m_angularVelocity);
    q3Identity(m_force);
    q3Identity(m_torque);
}

//------------------------------------------------------------------------------
bool q3Body::IsAwake() const { return m_flags & eAwake ? true : false; }

//------------------------------------------------------------------------------
r32 q3Body::GetMass() const { return m_mass; }

//------------------------------------------------------------------------------
r32 q3Body::GetInvMass() const { return m_invMass; }

//------------------------------------------------------------------------------
r32 q3Body::GetGravityScale() const { return m_gravityScale; }

//------------------------------------------------------------------------------
void q3Body::SetGravityScale(r32 scale) { m_gravityScale = scale; }

//------------------------------------------------------------------------------
const q3Vec3 q3Body::GetLocalPoint(const q3Vec3 &p) const
{
    return q3MulT(m_tx, p);
}

//------------------------------------------------------------------------------
const q3Vec3 q3Body::GetLocalVector(const q3Vec3 &v) const
{
    return q3MulT(m_tx.rotation, v);
}

//------------------------------------------------------------------------------
const q3Vec3 q3Body::GetWorldPoint(const q3Vec3 &p) const
{
    return q3Mul(m_tx, p);
}

//------------------------------------------------------------------------------
const q3Vec3 q3Body::GetWorldVector(const q3Vec3 &v) const
{
    return q3Mul(m_tx.rotation, v);
}

//------------------------------------------------------------------------------
const q3Vec3 q3Body::GetLinearVelocity() const { return m_linearVelocity; }

//------------------------------------------------------------------------------
const q3Vec3 q3Body::GetVelocityAtWorldPoint(const q3Vec3 &p) const
{
    q3Vec3 directionToPoint   = p - m_worldCenter;
    q3Vec3 relativeAngularVel = q3Cross(m_angularVelocity, directionToPoint);

    return m_linearVelocity + relativeAngularVel;
}

//------------------------------------------------------------------------------
void q3Body::SetLinearVelocity(const q3Vec3 &v)
{
    // Velocity of static bodies cannot be adjusted
    if (m_flags & eStatic)
        assert(false);

    if (q3Dot(v, v) > r32(0.0))
    {
        SetToAwake();
    }

    m_linearVelocity = v;
}

//------------------------------------------------------------------------------
const q3Vec3 q3Body::GetAngularVelocity() const { return m_angularVelocity; }

//------------------------------------------------------------------------------
void q3Body::SetAngularVelocity(const q3Vec3 v)
{
    // Velocity of static bodies cannot be adjusted
    if (m_flags & eStatic)
        assert(false);

    if (q3Dot(v, v) > r32(0.0))
    {
        SetToAwake();
    }

    m_angularVelocity = v;
}

//------------------------------------------------------------------------------
bool q3Body::CanCollide(const q3Body *other) const
{
    if (this == other)
        return false;

    // Every collision must have at least one dynamic body involved
    if (!(m_flags & eDynamic) && !(other->m_flags & eDynamic))
        return false;

    if (!(m_layers & other->m_layers))
        return false;

    return true;
}

//------------------------------------------------------------------------------
const q3Transform q3Body::GetTransform() const { return m_tx; }

//------------------------------------------------------------------------------
void q3Body::SetTransform(const q3Vec3 &position)
{
    m_worldCenter = position;

    SynchronizeProxies();
}

//------------------------------------------------------------------------------
void q3Body::SetTransform(const q3Vec3 &position, const q3Vec3 &axis, r32 angle)
{
    m_worldCenter = position;
    m_q.Set(axis, angle);
    m_tx.rotation = m_q.ToMat3();

    SynchronizeProxies();
}

//------------------------------------------------------------------------------
i32 q3Body::GetFlags() const { return m_flags; }

//------------------------------------------------------------------------------
void q3Body::SetLayers(i32 layers) { m_layers = layers; }

//------------------------------------------------------------------------------
i32 q3Body::GetLayers() const { return m_layers; }

//------------------------------------------------------------------------------
const q3Quaternion q3Body::GetQuaternion() const { return m_q; }

//------------------------------------------------------------------------------
void *q3Body::GetUserData() const { return m_userData; }

//------------------------------------------------------------------------------
void q3Body::SetLinearDamping(r32 damping) { m_linearDamping = damping; }

//------------------------------------------------------------------------------
r32 q3Body::GetLinearDamping(r32 damping) const { return m_linearDamping; }

//------------------------------------------------------------------------------
void q3Body::SetAngularDamping(r32 damping) { m_angularDamping = damping; }

//------------------------------------------------------------------------------
r32 q3Body::GetAngularDamping(r32 damping) const { return m_angularDamping; }

//------------------------------------------------------------------------------
void q3Body::Render(q3Render *render) const
{
    bool awake     = IsAwake();
    q3Shape *shape = m_shapes;

    while (shape)
    {
        shape->Render(m_tx, awake, render);
        shape = shape->next;
    }
}

//------------------------------------------------------------------------------
void q3Body::Dump(FILE *file, i32 index) const
{
    fprintf(file, "{\n");
    fprintf(file, "\tq3BodyDef bd;\n");

    switch (m_flags & (eStatic | eDynamic | eKinematic))
    {
    case eStatic:
        fprintf(file, "\tbd.bodyType = q3BodyType( %d );\n", eStaticBody);
        break;

    case eDynamic:
        fprintf(file, "\tbd.bodyType = q3BodyType( %d );\n", eDynamicBody);
        break;

    case eKinematic:
        fprintf(file, "\tbd.bodyType = q3BodyType( %d );\n", eKinematicBody);
        break;
    }

    fprintf(
        file,
        "\tbd.position.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n",
        m_tx.position.x,
        m_tx.position.y,
        m_tx.position.z);
    q3Vec3 axis;
    r32 angle;
    m_q.ToAxisAngle(&axis, &angle);
    fprintf(file,
            "\tbd.axis.Set( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) );\n",
            axis.x,
            axis.y,
            axis.z);
    fprintf(file, "\tbd.angle = r32( %.15lf );\n", angle);
    fprintf(file,
            "\tbd.linearVelocity.Set( r32( %.15lf ), r32( %.15lf ), r32( "
            "%.15lf ) );\n",
            m_linearVelocity.x,
            m_linearVelocity.y,
            m_linearVelocity.z);
    fprintf(file,
            "\tbd.angularVelocity.Set( r32( %.15lf ), r32( %.15lf ), r32( "
            "%.15lf ) );\n",
            m_angularVelocity.x,
            m_angularVelocity.y,
            m_angularVelocity.z);
    fprintf(file, "\tbd.gravityScale = r32( %.15lf );\n", m_gravityScale);
    fprintf(file, "\tbd.layers = %d;\n", m_layers);
    fprintf(file, "\tbd.allowSleep = bool( %d );\n", m_flags & eAllowSleep);
    fprintf(file, "\tbd.awake = bool( %d );\n", m_flags & eAwake);
    fprintf(file, "\tbd.awake = bool( %d );\n", m_flags & eAwake);
    fprintf(file, "\tbd.lockAxisX = bool( %d );\n", m_flags & eLockAxisX);
    fprintf(file, "\tbd.lockAxisY = bool( %d );\n", m_flags & eLockAxisY);
    fprintf(file, "\tbd.lockAxisZ = bool( %d );\n", m_flags & eLockAxisZ);
    fprintf(file, "\tbodies[ %d ] = scene.CreateBody( bd );\n\n", index);

    q3Shape *shape = m_shapes;

    while (shape)
    {
        if (shape->m_type == eBox)
        {
            q3Box *box = (q3Box *)shape;
            fprintf(file, "\t{\n");
            fprintf(file, "\t\tq3BoxDef sd;\n");
            fprintf(
                file, "\t\tsd.SetFriction( r32( %.15lf ) );\n", box->friction);
            fprintf(file,
                    "\t\tsd.SetRestitution( r32( %.15lf ) );\n",
                    box->restitution);
            fprintf(
                file, "\t\tsd.SetDensity( r32( %.15lf ) );\n", box->density);
            i32 sensor = (int)box->sensor;
            fprintf(file, "\t\tsd.SetSensor( bool( %d ) );\n", sensor);
            fprintf(file, "\t\tq3Transform boxTx;\n");
            q3Transform boxTx = box->local;
            q3Vec3 xAxis      = boxTx.rotation.col0;
            q3Vec3 yAxis      = boxTx.rotation.col1;
            q3Vec3 zAxis      = boxTx.rotation.col2;
            fprintf(
                file,
                "\t\tq3Vec3 xAxis( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) "
                ");\n",
                xAxis.x,
                xAxis.y,
                xAxis.z);
            fprintf(
                file,
                "\t\tq3Vec3 yAxis( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) "
                ");\n",
                yAxis.x,
                yAxis.y,
                yAxis.z);
            fprintf(
                file,
                "\t\tq3Vec3 zAxis( r32( %.15lf ), r32( %.15lf ), r32( %.15lf ) "
                ");\n",
                zAxis.x,
                zAxis.y,
                zAxis.z);
            fprintf(file,
                    "\t\tboxTx.rotation.SetColumns( xAxis, yAxis, zAxis );\n");
            fprintf(
                file,
                "\t\tboxTx.position.Set( r32( %.15lf ), r32( %.15lf ), r32( "
                "%.15lf ) );\n",
                boxTx.position.x,
                boxTx.position.y,
                boxTx.position.z);
            fprintf(
                file,
                "\t\tsd.Set( boxTx, q3Vec3( r32( %.15lf ), r32( %.15lf ), r32( "
                "%.15lf ) ) );\n",
                box->e.x * 2.0f,
                box->e.y * 2.0f,
                box->e.z * 2.0f);
            fprintf(file, "\t\tbodies[ %d ]->AddBox( sd );\n", index);
            fprintf(file, "\t}\n");
        }
        else if (shape->m_type == eSphere)
        {
            q3Sphere *sphere = (q3Sphere *)shape;
            fprintf(file, "\t{\n");
            fprintf(file, "\t\tq3SphereDef sd;\n");
            fprintf(file,
                    "\t\tsd.SetFriction( r32( %.15lf ) );\n",
                    sphere->friction);
            fprintf(file,
                    "\t\tsd.SetRestitution( r32( %.15lf ) );\n",
                    sphere->restitution);
            fprintf(
                file, "\t\tsd.SetDensity( r32( %.15lf ) );\n", sphere->density);
            i32 sensor = (int)sphere->sensor;
            fprintf(file, "\t\tsd.SetSensor( bool( %d ) );\n", sensor);
            fprintf(file, "\t\tq3Transform sphereTx;\n");
            q3Transform sphereTx = sphere->local;
            q3Vec3 xAxis         = sphereTx.rotation.col0;
            q3Vec3 yAxis         = sphereTx.rotation.col1;
            q3Vec3 zAxis         = sphereTx.rotation.col2;
            fprintf(file,
                    "\t\tq3Vec3 xAxis( r32( %.15lf ), r32( %.15lf ), r32( "
                    "%.15lf ) );\n",
                    xAxis.x,
                    xAxis.y,
                    xAxis.z);
            fprintf(file,
                    "\t\tq3Vec3 yAxis( r32( %.15lf ), r32( %.15lf ), r32( "
                    "%.15lf ) );\n",
                    yAxis.x,
                    yAxis.y,
                    yAxis.z);
            fprintf(file,
                    "\t\tq3Vec3 zAxis( r32( %.15lf ), r32( %.15lf ), r32( "
                    "%.15lf ) );\n",
                    zAxis.x,
                    zAxis.y,
                    zAxis.z);
            fprintf(
                file,
                "\t\tsphereTx.rotation.SetColumns( xAxis, yAxis, zAxis );\n");
            fprintf(file,
                    "\t\tsphereTx.position.Set( r32( %.15lf ), r32( %.15lf ), "
                    "r32( %.15lf ) );\n",
                    sphereTx.position.x,
                    sphereTx.position.y,
                    sphereTx.position.z);
            fprintf(file,
                    "\t\tsd.Set( sphereTx, r32( %.15lf ) );\n",
                    sphere->radius);
            fprintf(file, "\t\tbodies[ %d ]->AddSphere( sd );\n", index);
            fprintf(file, "\t}\n");
        }
        else if (shape->m_type == eCapsule)
        {
            q3Capsule *capsule = (q3Capsule *)shape;
            fprintf(file, "\t{\n");
            fprintf(file, "\t\tq3CapsuleDef sd;\n");
            fprintf(file,
                    "\t\tsd.SetFriction( r32( %.15lf ) );\n",
                    capsule->friction);
            fprintf(file,
                    "\t\tsd.SetRestitution( r32( %.15lf ) );\n",
                    capsule->restitution);
            fprintf(file,
                    "\t\tsd.SetDensity( r32( %.15lf ) );\n",
                    capsule->density);
            i32 sensor = (int)capsule->sensor;
            fprintf(file, "\t\tsd.SetSensor( bool( %d ) );\n", sensor);
            fprintf(file, "\t\tq3Transform capsuleTx;\n");
            q3Transform capsuleTx = capsule->local;
            q3Vec3 xAxis          = capsuleTx.rotation.col0;
            q3Vec3 yAxis          = capsuleTx.rotation.col1;
            q3Vec3 zAxis          = capsuleTx.rotation.col2;
            fprintf(file,
                    "\t\tq3Vec3 xAxis( r32( %.15lf ), r32( %.15lf ), r32( "
                    "%.15lf ) );\n",
                    xAxis.x,
                    xAxis.y,
                    xAxis.z);
            fprintf(file,
                    "\t\tq3Vec3 yAxis( r32( %.15lf ), r32( %.15lf ), r32( "
                    "%.15lf ) );\n",
                    yAxis.x,
                    yAxis.y,
                    yAxis.z);
            fprintf(file,
                    "\t\tq3Vec3 zAxis( r32( %.15lf ), r32( %.15lf ), r32( "
                    "%.15lf ) );\n",
                    zAxis.x,
                    zAxis.y,
                    zAxis.z);
            fprintf(
                file,
                "\t\tcapsuleTx.rotation.SetColumns( xAxis, yAxis, zAxis );\n");
            fprintf(file,
                    "\t\tcapsuleTx.position.Set( r32( %.15lf ), r32( %.15lf ), "
                    "r32( %.15lf ) );\n",
                    capsuleTx.position.x,
                    capsuleTx.position.y,
                    capsuleTx.position.z);
            fprintf(file,
                    "\t\tsd.Set( capsuleTx, r32( %.15lf ), r32( %.15lf ) );\n",
                    capsule->height,
                    capsule->radius);
            fprintf(file, "\t\tbodies[ %d ]->AddCapsule( sd );\n", index);
            fprintf(file, "\t}\n");
        }
        shape = shape->next;
    }

    fprintf(file, "}\n\n");
}

//------------------------------------------------------------------------------
void q3Body::CalculateMassData()
{
    q3Mat3 inertia    = q3Diagonal(r32(0.0));
    m_invInertiaModel = q3Diagonal(r32(0.0));
    m_invInertiaWorld = q3Diagonal(r32(0.0));
    m_invMass         = r32(0.0);
    m_mass            = r32(0.0);
    r32 mass          = r32(0.0);

    if (m_flags & eStatic || m_flags & eKinematic)
    {
        q3Identity(m_localCenter);
        m_worldCenter = m_tx.position;
        return;
    }

    q3Vec3 lc;
    q3Identity(lc);

    for (q3Shape *shape = m_shapes; shape; shape = shape->next)
    {
        if (shape->density == r32(0.0))
            continue;

        q3MassData md;
        shape->ComputeMass(&md);
        mass += md.mass;
        inertia += md.inertia;
        lc += md.center * md.mass;
    }

    if (mass > r32(0.0))
    {
        m_mass    = mass;
        m_invMass = r32(1.0) / mass;
        lc *= m_invMass;
        q3Mat3 identity;
        q3Identity(identity);
        inertia -= (identity * q3Dot(lc, lc) - q3OuterProduct(lc, lc)) * mass;
        m_invInertiaModel = q3Inverse(inertia);

        if (m_flags & eLockAxisX)
            q3Identity(m_invInertiaModel.col0);

        if (m_flags & eLockAxisY)
            q3Identity(m_invInertiaModel.col1);

        if (m_flags & eLockAxisZ)
            q3Identity(m_invInertiaModel.col2);
    }
    else
    {
        // Force all dynamic bodies to have some mass
        m_invMass         = r32(1.0);
        m_invInertiaModel = q3Diagonal(r32(0.0));
        m_invInertiaWorld = q3Diagonal(r32(0.0));
    }

    m_localCenter = lc;
    m_worldCenter = q3Mul(m_tx, lc);
}

//------------------------------------------------------------------------------
void q3Body::SynchronizeProxies()
{
    q3BroadPhase *broadphase = &m_scene->m_contactManager.m_broadphase;

    m_tx.position = m_worldCenter - q3Mul(m_tx.rotation, m_localCenter);

    q3AABB aabb;
    q3Transform tx = m_tx;

    q3Shape *shape = m_shapes;
    while (shape)
    {
        shape->ComputeAABB(tx, &aabb);
        broadphase->Update(shape->broadPhaseIndex, aabb);
        shape = shape->next;
    }
}
