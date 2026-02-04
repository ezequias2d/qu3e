#include "../dynamics/q3Body.h"
#include "../dynamics/q3Contact.h"
#include "q3Box.h"
#include "q3Capsule.h"
#include "q3Collide.h"
#include "q3Sphere.h"

#include "q3Collide.inl"

//------------------------------------------------------------------------------
void q3SphereToSphere(q3Manifold *m, q3Sphere *a, q3Sphere *b)
{
    q3Transform atx = a->body->GetTransform();
    q3Transform btx = b->body->GetTransform();
    q3Transform aL  = a->local;
    q3Transform bL  = b->local;
    atx             = q3Mul(atx, aL);
    btx             = q3Mul(btx, bL);

    q3Vec3 d = btx.position - atx.position;
    r32 d2   = q3Dot(d, d);
    r32 r    = a->radius + b->radius;

    if (d2 > r * r)
        return;

    r32 l = sqrt(d2);

    m->contactCount = 1;

    q3Vec3 n;

    const r32 epsilon = r32(1.0e-8);
    if (l > epsilon)
    {
        n = d / l;
    }
    else
    {
        n.Set(r32(0.0), r32(1.0), r32(0.0));
    }

    m->normal = n; // Normal points from A to B

    q3Contact *c   = m->contacts;
    c->penetration = r - l;
    c->position    = atx.position + n * a->radius;
    c->fp.key      = 0;
}

//------------------------------------------------------------------------------
void q3SphereToBox(q3Manifold *m, q3Sphere *a, q3Box *b)
{
    m->A      = a;
    m->B      = b;
    m->sensor = a->sensor || b->sensor;

    q3Transform atx = a->body->GetTransform();
    q3Transform btx = b->body->GetTransform();
    q3Transform aL  = a->local;
    q3Transform bL  = b->local;
    atx             = q3Mul(atx, aL);
    btx             = q3Mul(btx, bL);

    // Transform sphere center into box's local space
    q3Vec3 relCenter = q3MulT(btx, atx.position);

    // Clamp to box extents
    q3Vec3 closest = relCenter;
    closest.x      = q3Clamp(-b->e.x, b->e.x, closest.x);
    closest.y      = q3Clamp(-b->e.y, b->e.y, closest.y);
    closest.z      = q3Clamp(-b->e.z, b->e.z, closest.z);

    // Check distance
    q3Vec3 d = relCenter - closest;
    r32 d2   = q3Dot(d, d);

    if (d2 > a->radius * a->radius)
        return;

    // Collision detected
    m->contactCount = 1;
    q3Contact *c    = m->contacts;

    const r32 epsilon = r32(1.0e-8);

    if (d2 > epsilon)
    {
        r32 l     = sqrt(d2);
        q3Vec3 n  = d / l;
        m->normal = -q3Mul(btx.rotation, n);

        c->penetration = a->radius - l;
        c->position    = q3Mul(btx, closest);
        c->fp.key      = 0;
    }

    else
    {
        // Sphere center is inside the box
        r32 dx = b->e.x - q3Abs(relCenter.x);
        r32 dy = b->e.y - q3Abs(relCenter.y);
        r32 dz = b->e.z - q3Abs(relCenter.z);

        if (dx < dy && dx < dz)
        {
            r32 sign  = q3Sign(relCenter.x);
            m->normal = -q3Mul(btx.rotation, q3Vec3(sign, r32(0.0), r32(0.0)));
            c->penetration = a->radius + dx;
            closest.x      = sign * b->e.x;
        }

        else if (dy < dz)
        {
            r32 sign  = q3Sign(relCenter.y);
            m->normal = -q3Mul(btx.rotation, q3Vec3(r32(0.0), sign, r32(0.0)));
            c->penetration = a->radius + dy;
            closest.y      = sign * b->e.y;
        }

        else
        {
            r32 sign  = q3Sign(relCenter.z);
            m->normal = -q3Mul(btx.rotation, q3Vec3(r32(0.0), r32(0.0), sign));
            c->penetration = a->radius + dz;
            closest.z      = sign * b->e.z;
        }

        c->position = q3Mul(btx, closest);
        c->fp.key   = 0;
    }
}