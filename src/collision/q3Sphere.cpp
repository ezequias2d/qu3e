//------------------------------------------------------------------------------
/**
@file	q3Sphere.cpp

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

#include "q3Sphere.h"
#include "../common/q3Geometry.h"
#include "../dynamics/q3Body.h"

//------------------------------------------------------------------------------
bool q3Sphere::TestPoint(const q3Transform &tx, const q3Vec3 &p) const
{
    q3Transform world = q3Mul(tx, local);
    q3Vec3 d          = p - world.position;

    return q3Dot(d, d) <= radius * radius;
}

//------------------------------------------------------------------------------
bool q3Sphere::Raycast(const q3Transform &tx, q3RaycastData *raycast) const
{
    q3Transform world = q3Mul(tx, local);
    q3Vec3 d          = raycast->start - world.position;
    r32 a             = q3Dot(raycast->dir, raycast->dir);
    r32 b             = r32(2.0) * q3Dot(d, raycast->dir);
    r32 c             = q3Dot(d, d) - radius * radius;

    r32 discriminant = b * b - r32(4.0) * a * c;

    if (discriminant < r32(0.0))
        return false;

    discriminant = sqrt(discriminant);

    r32 t0 = (-b - discriminant) / (r32(2.0) * a);

    if (t0 > raycast->t || t0 < r32(0.0))
        return false;

    raycast->toi    = t0;
    raycast->normal = q3Normalize(d + raycast->dir * t0);

    return true;
}

//------------------------------------------------------------------------------
void q3Sphere::ComputeAABB(const q3Transform &tx, q3AABB *aabb) const
{
    q3Transform world = q3Mul(tx, local);
    q3Vec3 r(radius, radius, radius);
    aabb->min = world.position - r;
    aabb->max = world.position + r;
}

//------------------------------------------------------------------------------
void q3Sphere::ComputeMass(q3MassData *md) const
{
    // I = (2/5) * m * r^2
    r32 volume = (r32(4.0) / r32(3.0)) * r32::pi() * radius * radius * radius;
    r32 mass   = density * volume;
    r32 I      = (r32(2.0) / r32(5.0)) * mass * radius * radius;

    md->inertia = q3Diagonal(I);
    md->center  = local.position;
    md->mass    = mass;
}

//------------------------------------------------------------------------------
void q3Sphere::Render(const q3Transform &tx, bool awake, q3Render *render) const
{
    q3Transform world = q3Mul(tx, local);

    if (awake)
        render->SetPenColor(0.2f, 0.5f, 1.0f);
    else
        render->SetPenColor(0.2f, 0.2f, 0.2f);

    render->SetPenPosition(
        world.position.x, world.position.y, world.position.z);
    render->SetScale(radius, radius, radius);
    render->Sphere();
    render->SetScale(1.0f, 1.0f, 1.0f);
}
