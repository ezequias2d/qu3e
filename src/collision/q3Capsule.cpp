//------------------------------------------------------------------------------
/**
@file	q3Capsule.cpp

@author Ezequias Silva
@date   19/12/2025
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

#include "q3Capsule.h"
#include "../dynamics/q3Body.h"
#include <cmath>

//------------------------------------------------------------------------------
bool q3Capsule::TestPoint(const q3Transform &tx, const q3Vec3 &p) const
{
    // Optimized: pLocal = local^T * (tx^T * p)
    q3Vec3 localP = q3MulT(tx, p);
    localP        = q3MulT(local, localP);

    // Project onto Y axis (capsule axis)
    r32 y = localP.y;
    r32 h = height * 0.5f;

    // Clamp to segment
    y = q3Clamp(-h, h, y);

    // Distance from axis
    q3Vec3 closest(0.0f, y, 0.0f);
    q3Vec3 d = localP - closest;

    return q3Dot(d, d) <= radius * radius;
}

//------------------------------------------------------------------------------
bool q3Capsule::Raycast(const q3Transform &tx, q3RaycastData *raycast) const
{
    // Transform ray into local space
    q3Vec3 p1 = q3MulT(tx, raycast->start);
    p1        = q3MulT(local, p1);

    q3Vec3 d1 = q3MulT(tx.rotation, raycast->dir);
    d1        = q3MulT(local.rotation, d1);

    // Test against cylinder
    r32 tMin = raycast->t;
    bool hit = false;
    q3Vec3 normal(0.0f, 0.0f, 0.0f);

    r32 A = d1.x * d1.x + d1.z * d1.z;
    if (A > r32(1.0e-8))
    {
        r32 B   = r32(2.0) * (p1.x * d1.x + p1.z * d1.z);
        r32 C   = p1.x * p1.x + p1.z * p1.z - radius * radius;
        r32 det = B * B - r32(4.0) * A * C;

        if (det >= r32(0.0))
        {
            r32 sqrtDet = sqrt(det);
            r32 t1      = (-B - sqrtDet) / (r32(2.0) * A);
            if (t1 >= r32(0.0) && t1 < tMin)
            {
                // Check height bounds
                r32 y = p1.y + t1 * d1.y;
                r32 h = height * r32(0.5);

                if (y >= -h && y <= h)
                {
                    tMin = t1;
                    hit  = true;
                    // Normal at intersection (local space)
                    // Gradient of x^2 + z^2 is (2x, 0, 2z) => (x, 0, z)
                    normal.Set(p1.x + t1 * d1.x, r32(0.0), p1.z + t1 * d1.z);
                    normal = q3Normalize(normal);
                }
            }
        }
    }

    // Test against top hemisphere
    // Center (0, h/2, 0)
    r32 h = height * r32(0.5);
    q3Vec3 sphereCenter(0.0f, h, 0.0f);
    q3Vec3 m = p1 - sphereCenter;
    r32 b    = q3Dot(m, d1);
    r32 c    = q3Dot(m, m) - radius * radius;

    // Ray origin outside sphere (c > 0) and pointing away (b > 0) -> no
    // intersection
    if (c > r32(0.0) && b > r32(0.0))
    {
        // No intersection with top sphere
    }
    else
    {
        r32 discr = b * b - c;
        if (discr >= r32(0.0))
        {
            r32 t = -b - sqrt(discr);
            if (t >= r32(0.0) && t < tMin)
            {
                // Ensure we are on the upper hemisphere (y >= h)
                r32 y = p1.y + t * d1.y;
                if (y >= h)
                {
                    tMin   = t;
                    hit    = true;
                    normal = (p1 + d1 * t) - sphereCenter;
                    normal = q3Normalize(normal);
                }
            }
        }
    }

    // Test against bottom hemisphere
    // Center (0, -h/2, 0)
    sphereCenter.Set(0.0f, -h, 0.0f);
    m = p1 - sphereCenter;
    b = q3Dot(m, d1);
    c = q3Dot(m, m) - radius * radius;

    if (c > r32(0.0) && b > r32(0.0))
    {
        // No intersection with bottom sphere
    }
    else
    {
        r32 discr = b * b - c;
        if (discr >= r32(0.0))
        {
            r32 t = -b - sqrt(discr);
            if (t >= r32(0.0) && t < tMin)
            {
                // Ensure we are on the lower hemisphere (y <= -h)
                r32 y = p1.y + t * d1.y;
                if (y <= -h)
                {
                    tMin   = t;
                    hit    = true;
                    normal = (p1 + d1 * t) - sphereCenter;
                    normal = q3Normalize(normal);
                }
            }
        }
    }

    if (hit)
    {
        raycast->toi = tMin;
        // Transform normal to world space
        q3Vec3 worldNormal = q3Mul(local.rotation, normal);
        worldNormal        = q3Mul(tx.rotation, worldNormal);

        raycast->normal = worldNormal;
        return true;
    }

    return false;
}

//------------------------------------------------------------------------------
void q3Capsule::ComputeAABB(const q3Transform &tx, q3AABB *aabb) const
{
    q3Vec3 axis = q3Mul(tx.rotation, local.rotation.col1);

    q3Vec3 pos = q3Mul(tx, local.position);

    q3Vec3 p1 = pos + axis * (height * r32(0.5));
    q3Vec3 p2 = pos - axis * (height * r32(0.5));

    q3Vec3 min = q3Min(p1, p2);
    q3Vec3 max = q3Max(p1, p2);

    q3Vec3 r(radius, radius, radius);
    aabb->min = min - r;
    aabb->max = max + r;
}

//------------------------------------------------------------------------------
void q3Capsule::ComputeMass(q3MassData *md) const
{
    // V = pi * r^2 * (h + 4/3 * r)
    r32 r2   = radius * radius;
    r32 vol  = r32::pi() * r2 * (height + (r32(4.0) / r32(3.0)) * radius);
    r32 mass = vol * density;

    md->mass   = mass;
    md->center = q3Mul(local, q3Vec3(0, 0, 0));

    // Inertia Tensor
    // Cylinder approximation: H = h + 2r
    r32 totalH = height + r32(2.0) * radius;

    // I_y = 0.5 * m * r^2
    // I_x = I_z = 1/12 * m * (3*r^2 + H^2)
    r32 Iy = r32(0.5) * mass * r2;
    r32 Ix =
        (mass * (r32(3.0) * r2 + totalH * totalH)) * (r32(1.0) / r32(12.0));
    r32 Iz = Ix;

    md->inertia.Set(Ix, 0, 0, 0, Iy, 0, 0, 0, Iz);

    // Rotate inertia tensor into local space
    // Similarity transform: I' = R * I * R^T
    md->inertia = local.rotation * md->inertia * q3Transpose(local.rotation);

    // Shift inertia to body origin
    // I_new = I_cm + m * ( |p|^2 * I - p * p^T )
    // We can do this element-wise to avoid matrix ops
    q3Vec3 p = local.position;
    r32 x    = p.x;
    r32 y    = p.y;
    r32 z    = p.z;
    r32 m    = mass;

    // Diagonal terms: I_xx += m * (y^2 + z^2), etc.
    md->inertia.col0.x += m * (y * y + z * z);
    md->inertia.col1.y += m * (x * x + z * z);
    md->inertia.col2.z += m * (x * x + y * y);

    // Off-diagonal terms: I_xy -= m * x * y, etc.
    r32 mxy = m * x * y;
    r32 mxz = m * x * z;
    r32 myz = m * y * z;

    md->inertia.col0.y -= mxy;
    md->inertia.col1.x -= mxy;

    md->inertia.col0.z -= mxz;
    md->inertia.col2.x -= mxz;

    md->inertia.col1.z -= myz;
    md->inertia.col2.y -= myz;
}

//------------------------------------------------------------------------------
void q3Capsule::Render(const q3Transform &tx,
                       bool awake,
                       q3Render *render) const
{
    q3Vec3 axis = q3Mul(tx.rotation, local.rotation.col1);

    q3Vec3 pos = q3Mul(tx, local.position);

    r32 h = height * f32(0.5);

    q3Vec3 p1 = pos - axis * h;
    q3Vec3 p2 = pos + axis * h;

    if (awake)
        render->SetPenColor(f32(0.2), f32(0.4), f32(0.7));
    else
        render->SetPenColor(f32(0.5), f32(0.5), f32(0.5));

    render->SetPenPosition(pos.x, pos.y, pos.z);
    render->SetScale(f32(1.0), f32(1.0), f32(1.0));

    render->Capsule(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, radius);
}
