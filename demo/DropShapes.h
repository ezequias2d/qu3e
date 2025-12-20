//------------------------------------------------------------------------------
/**
@file	DropShapes.h

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

#ifndef DROPSHAPES_H
#define DROPSHAPES_H

#include "../src/collision/q3Box.h"
#include "../src/collision/q3Capsule.h"
#include "../src/collision/q3Sphere.h"
#include "../src/common/q3Geometry.h"
#include "../src/dynamics/q3Body.h"
#include "../src/math/q3Math.h"
#include "Demo.h"

struct DropShapes : public Demo
{
    virtual void Init()
    {
        acc = 0;

        // Create the floor
        q3BodyDef bodyDef;
        q3Body *body = scene.CreateBody(bodyDef);

        q3BoxDef boxDef;
        boxDef.SetRestitution(0);
        q3Transform tx;
        q3Identity(tx);
        boxDef.Set(tx, q3Vec3(50.0f, 1.0f, 50.0f));
        body->AddBox(boxDef);
    }

    virtual void Update()
    {
        acc += dt;

        if (acc > 0.5f)
        {
            acc = 0;

            q3BodyDef bodyDef;
            bodyDef.position = {
                q3RandomFloat(-5.0f, 5.0f), 10.0f, q3RandomFloat(-5.0f, 5.0f)};
            bodyDef.axis            = {q3RandomFloat(-1, 1),
                                       q3RandomFloat(-1, 1),
                                       q3RandomFloat(-1, 1)};
            bodyDef.angle           = r32::pi() * q3RandomFloat(-1, 1);
            bodyDef.bodyType        = eDynamicBody;
            bodyDef.angularVelocity = {q3RandomFloat(1, 3.0f),
                                       q3RandomFloat(1, 3.0f),
                                       q3RandomFloat(1, 3.0f)};
            bodyDef.angularVelocity *= q3Sign(q3RandomFloat(-1.0f, 1.0f));
            bodyDef.linearVelocity = {q3RandomFloat(1.0f, 3.0f),
                                      q3RandomFloat(1.0f, 3.0f),
                                      q3RandomFloat(1.0f, 3.0f)};
            bodyDef.linearVelocity *= q3Sign(q3RandomFloat(-1.0f, 1.0f));
            q3Body *body = scene.CreateBody(bodyDef);

            r32 r = q3RandomFloat(r32(0.0), r32(1.0));
            if (r < 0.33f)
            {
                q3Transform tx;
                q3Identity(tx);
                q3BoxDef boxDef;
                boxDef.Set(tx, q3Vec3(1.0f, 1.0f, 1.0f));
                body->AddBox(boxDef);
            }
            else if (r < 0.66f)
            {
                q3SphereDef sphereDef;
                sphereDef.radius = 1.0f;
                sphereDef.SetDensity(1.0f);
                body->AddSphere(sphereDef);
            }
            else
            {
                q3CapsuleDef capsuleDef;
                capsuleDef.radius = 0.5f;
                capsuleDef.height = 2.0f;
                capsuleDef.SetDensity(1.0f);
                body->AddCapsule(capsuleDef);
            }
        }
    }

    virtual void Shutdown() { scene.RemoveAllBodies(); }

    float acc;
};

#endif // DROPSHAPES_H
