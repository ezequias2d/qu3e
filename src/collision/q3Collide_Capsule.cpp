#include "../dynamics/q3Body.h"
#include "../dynamics/q3Contact.h"
#include "q3Box.h"
#include "q3Capsule.h"
#include "q3Collide.h"
#include "q3Sphere.h"

#include "q3Collide.inl"

//------------------------------------------------------------------------------
void q3CapsuleToCapsule(q3Manifold *m, q3Capsule *a, q3Capsule *b)
{
    q3Transform atx = a->body->GetTransform();
    q3Transform btx = b->body->GetTransform();
    q3Transform aL  = a->local;
    q3Transform bL  = b->local;
    atx             = q3Mul(atx, aL);
    btx             = q3Mul(btx, bL);

    // Get capsule axes in world space
    // Y-axis is the capsule axis
    q3Vec3 aY = atx.rotation.col1;
    q3Vec3 bY = btx.rotation.col1;

    q3Vec3 pA = atx.position;
    q3Vec3 pB = btx.position;

    // Segment A: pA - aY * (hA/2) to pA + aY * (hA/2)
    // Segment B: pB - bY * (hB/2) to pB + bY * (hB/2)
    r32 hA = a->height * 0.5f;
    r32 hB = b->height * 0.5f;

    q3Vec3 termA = aY * hA;
    q3Vec3 termB = bY * hB;

    q3Vec3 pA1 = pA - termA;
    q3Vec3 pA2 = pA + termA;
    q3Vec3 pB1 = pB - termB;
    q3Vec3 pB2 = pB + termB;

    // Find closest points between two segments
    q3Vec3 cA, cB;
    q3EdgesContact(&cA, &cB, pA1, pA2, pB1, pB2);

    q3Vec3 d = cB - cA;
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
        // Coincident centers or very close
        n.Set(r32(0.0), r32(1.0), r32(0.0));
    }

    m->normal = n;

    q3Contact *c   = m->contacts;
    c->penetration = r - l;

    // Contact point is usually midpoint of the overlap or surface of A
    // Let's use surface of A
    c->position = cA + n * a->radius;
    c->fp.key   = 0;
}

//------------------------------------------------------------------------------
void q3CapsuleToBox(q3Manifold *m, q3Capsule *a, q3Box *b)
{
    q3Transform atx = a->body->GetTransform();
    q3Transform btx = b->body->GetTransform();
    q3Transform aL  = a->local;
    q3Transform bL  = b->local;
    atx             = q3Mul(atx, aL);
    btx             = q3Mul(btx, bL);

    // Transform Capsule A into Box B's local space
    q3Transform tx = q3MulT(btx, atx);

    // Capsule points in Box space
    r32 h        = a->height * 0.5f;
    q3Vec3 axisA = tx.rotation.col1; // Capsule axis is Y
    q3Vec3 posA  = tx.position;
    q3Vec3 p1    = posA - axisA * h;
    q3Vec3 p2    = posA + axisA * h;
    r32 radius   = a->radius;
    q3Vec3 e     = b->e;

    // SAT Phase 1: Box Face Axes (x, y, z)
    int axis       = -1;
    r32 separation = -Q3_R32_MAX;
    q3Vec3 normal(0, 0, 0);

    for (int i = 0; i < 3; ++i)
    {
        // Project capsule endpoints onto axis i
        r32 p1v = i == 0 ? p1.x : (i == 1 ? p1.y : p1.z);
        r32 p2v = i == 0 ? p2.x : (i == 1 ? p2.y : p2.z);

        r32 minP = q3Min(p1v, p2v);
        r32 maxP = q3Max(p1v, p2v);

        // Separation along axis i:
        // Sep = (minP - r) - e[i] (Capsule on Positive side)
        r32 sepPos = minP - radius - e[i];

        // Sep = -e[i] - (maxP + radius) (Capsule on Negative side)
        r32 sepNeg = -e[i] - (maxP + radius);

        r32 sep = q3Max(sepPos, sepNeg);

        if (sep > r32(0.0))
            return; // Separating axis found

        if (sep > separation)
        {
            separation = sep;
            axis       = i;
            // Normal points from Capsule to Box (A -> B)
            if (sepPos > sepNeg)
            {
                normal.Set(0, 0, 0);
                if (i == 0)
                    normal.x = r32(-1.0);
                else if (i == 1)
                    normal.y = r32(-1.0);
                else
                    normal.z = r32(-1.0);
            }
            else
            {
                normal.Set(0, 0, 0);
                if (i == 0)
                    normal.x = r32(1.0);
                else if (i == 1)
                    normal.y = r32(1.0);
                else
                    normal.z = r32(1.0);
            }
        }
    }

    // SAT Phase 2: Edge-Edge Axes
    q3Vec3 d = axisA;

    for (int i = 0; i < 3; ++i)
    {
        q3Vec3 axisBox(0, 0, 0);
        if (i == 0)
            axisBox.x = r32(1.0);
        else if (i == 1)
            axisBox.y = r32(1.0);
        else
            axisBox.z = r32(1.0);

        q3Vec3 n = q3Cross(axisBox, d);
        r32 len  = q3Length(n);

        if (len < 1.0e-5f)
            continue; // Parallel
        n /= len;

        // Project Box onto n (radius)
        r32 rBox = e.x * q3Abs(n.x) + e.y * q3Abs(n.y) + e.z * q3Abs(n.z);

        // Project Capsule center onto n
        r32 dist = q3Abs(q3Dot(n, posA));
        r32 sep  = dist - (rBox + radius);

        if (sep > r32(0.0))
            return;

        if (sep > separation)
        {
            separation = sep;
            axis       = 3 + i;

            // Ensure normal points A -> B (Capsule -> Box)
            if (q3Dot(n, posA) > r32(0.0))
                n = -n;
            normal = n;
        }
    }

    m->normal       = q3Mul(btx.rotation, normal);
    m->contactCount = 0;

    if (axis < 3)
    {
        // Face Interaction
        r32 planeDist = -e[axis];

        // Check P1
        r32 d1 = q3Dot(normal, p1) - planeDist;

        q3Vec3 p1World = atx.position - atx.rotation.col1 * h;
        q3Vec3 p2World = atx.position + atx.rotation.col1 * h;

        if (d1 >= -radius)
        {
            m->contacts[m->contactCount].position =
                p1World + m->normal * radius;
            m->contacts[m->contactCount].penetration = radius + d1;
            m->contacts[m->contactCount].fp.key      = 0;
            m->contactCount++;
        }

        r32 d2 = q3Dot(normal, p2) - planeDist;
        if (d2 >= -radius)
        {
            m->contacts[m->contactCount].position =
                p2World + m->normal * radius;
            m->contacts[m->contactCount].penetration = radius + d2;
            m->contacts[m->contactCount].fp.key      = 1;
            m->contactCount++;
        }
    }
    else
    {
        // Edge-Edge Interaction
        q3Vec3 PA, QA;
        PA = atx.position - atx.rotation.col1 * h;
        QA = atx.position + atx.rotation.col1 * h;

        q3Vec3 PB, QB;
        q3SupportEdge(btx, e, -m->normal, &PB, &QB);

        q3Vec3 CA, CB;
        q3EdgesContact(&CA, &CB, PA, QA, PB, QB);

        m->contacts[0].penetration = -separation;
        m->contacts[0].position    = CB;
        m->contacts[0].fp.key      = 0;
        m->contactCount            = 1;
    }
}

//------------------------------------------------------------------------------
void q3CapsuleToSphere(q3Manifold *m, q3Capsule *a, q3Sphere *b)
{
    q3Transform atx = a->body->GetTransform();
    q3Transform btx = b->body->GetTransform();
    q3Transform aL  = a->local;
    q3Transform bL  = b->local;
    atx             = q3Mul(atx, aL);
    btx             = q3Mul(btx, bL);

    // Transform sphere center into capsule's local space
    q3Vec3 spherePosLocal = q3MulT(atx, btx.position);

    // Capsule axis is Y. Extents are [-h/2, h/2] on Y.
    r32 h = a->height * 0.5f;
    r32 y = spherePosLocal.y;

    // Clamp y to capsule segment
    y = q3Clamp(-h, h, y);

    q3Vec3 closestOnSegmentLocal(r32(0.0), y, r32(0.0));
    q3Vec3 closestOnSegmentWorld = q3Mul(atx, closestOnSegmentLocal);

    q3Vec3 d = btx.position - closestOnSegmentWorld;
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

    m->normal = n; // From A (capsule) to B (sphere)

    q3Contact *c   = m->contacts;
    c->penetration = r - l;
    c->position    = closestOnSegmentWorld + n * a->radius;
    c->fp.key      = 0;
}