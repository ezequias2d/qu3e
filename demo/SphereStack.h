//------------------------------------------------------------------------------
/**
@file	SphereStack.h

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

#ifndef SPHERESTACK_H
#define SPHERESTACK_H

#include "Demo.h"

struct SphereStack : public Demo
{
    virtual void Init()
    {
        // Create the floor
        q3BodyDef bodyDef;
        q3Body *body = scene.CreateBody(bodyDef);

        q3BoxDef boxDef;
        boxDef.SetRestitution(0);
        q3Transform tx;
        q3Identity(tx);
        boxDef.Set(tx, q3Vec3(50.0f, 1.0f, 50.0f));
        body->AddBox(boxDef);

        bodyDef.bodyType = eDynamicBody;

        q3SphereDef sphereDef;
        sphereDef.radius = 1.0f;
        sphereDef.SetRestitution(0.5f);
        sphereDef.SetDensity(1.0f);

        for (i32 i = 0; i < 3; ++i)
        {
            for (i32 j = 0; j < 3; ++j)
            {
                for (i32 k = 0; k < 3; ++k)
                {
                    bodyDef.position = {r32(2.0) + r32(1.0) * j,
                                        r32(1.0) * r32(i) + r32(5.0),
                                        r32(2.0) + r32(1.0) * r32(k)};
                    body             = scene.CreateBody(bodyDef);
                    body->AddSphere(sphereDef);
                }
            }
        }
    }

    virtual void Shutdown() { scene.RemoveAllBodies(); }
};

#endif // SPHERESTACK_H
