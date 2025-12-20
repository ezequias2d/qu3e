//------------------------------------------------------------------------------
/**
@file	q3Shape.h

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

#ifndef Q3SHAPE_H
#define Q3SHAPE_H

#include "../common/q3Types.h"
#include "../math/q3Transform.h"

struct q3AABB;
struct q3RaycastData;
class q3Render;

//------------------------------------------------------------------------------
// q3MassData
//------------------------------------------------------------------------------
struct q3MassData
{
    q3Mat3 inertia;
    q3Vec3 center;
    r32 mass;
};

//------------------------------------------------------------------------------
// q3Shape
//------------------------------------------------------------------------------
struct q3Shape
{
    q3ShapeType m_type;
    class q3Body *body;
    q3Shape *next;

    r32 friction;
    r32 restitution;
    r32 density;
    i32 broadPhaseIndex;
    mutable void *userData;
    mutable bool sensor;

    void SetUserdata(void *data) const { userData = data; };
    void *GetUserdata() const { return userData; };
    void SetSensor(bool isSensor) { sensor = isSensor; };

    virtual bool TestPoint(const q3Transform &tx, const q3Vec3 &p) const = 0;
    virtual bool Raycast(const q3Transform &tx,
                         q3RaycastData *raycast) const                   = 0;
    virtual void ComputeAABB(const q3Transform &tx, q3AABB *aabb) const  = 0;
    virtual void ComputeMass(q3MassData *md) const                       = 0;
    virtual void
    Render(const q3Transform &tx, bool awake, q3Render *render) const = 0;

    virtual ~q3Shape() {}
};

#endif // Q3SHAPE_H
