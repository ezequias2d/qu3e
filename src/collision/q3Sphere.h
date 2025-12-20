//------------------------------------------------------------------------------
/**
@file	q3Sphere.h

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

#ifndef Q3SPHERE_H
#define Q3SPHERE_H

#include "../debug/q3Render.h"
#include "../math/q3Transform.h"
#include "q3Shape.h"

//------------------------------------------------------------------------------
// q3Sphere
//------------------------------------------------------------------------------
struct q3Sphere : public q3Shape
{
    q3Transform local;
    r32 radius;

    bool TestPoint(const q3Transform &tx, const q3Vec3 &p) const override final;
    bool Raycast(const q3Transform &tx,
                 q3RaycastData *raycast) const override final;
    void ComputeAABB(const q3Transform &tx, q3AABB *aabb) const override final;
    void ComputeMass(q3MassData *md) const override final;
    void Render(const q3Transform &tx,
                bool awake,
                q3Render *render) const override final;
};

//------------------------------------------------------------------------------
struct q3SphereDef
{
    q3SphereDef()
    {
        friction    = r32(0.4);
        restitution = r32(0.2);
        density     = r32(1.0);
        sensor      = false;
        radius      = r32(1.0);
        local.position.SetAll(0.0f);
        local.rotation.Set(
            1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    }

    void Set(r32 r) { radius = r; }

    void Set(const q3Transform &tx, r32 r)
    {
        local  = tx;
        radius = r;
    }

    void SetFriction(r32 f) { friction = f; }

    void SetRestitution(r32 r) { restitution = r; }

    void SetDensity(r32 d) { density = d; }

    void SetSensor(bool isSensor) { sensor = isSensor; }

    q3Transform local;
    r32 radius;
    r32 friction;
    r32 restitution;
    r32 density;
    bool sensor;
};

#endif // Q3SPHERE_H
