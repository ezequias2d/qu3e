//------------------------------------------------------------------------------
/**
@file	q3Collide.h

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

#ifndef Q3COLLIDE_H
#define Q3COLLIDE_H

#include "q3Box.h"
#include "q3Capsule.h"
#include "q3Sphere.h"

//------------------------------------------------------------------------------
// q3Collide
//------------------------------------------------------------------------------
struct q3Manifold;

void q3BoxtoBox(q3Manifold *m, q3Box *a, q3Box *b);
void q3SphereToSphere(q3Manifold *m, q3Sphere *a, q3Sphere *b);
void q3SphereToBox(q3Manifold *m, q3Sphere *a, q3Box *b);
void q3BoxToSphere(q3Manifold *m, q3Box *a, q3Sphere *b);
void q3CapsuleToCapsule(q3Manifold *m, q3Capsule *a, q3Capsule *b);
void q3CapsuleToBox(q3Manifold *m, q3Capsule *a, q3Box *b);
void q3BoxToCapsule(q3Manifold *m, q3Box *a, q3Capsule *b);
void q3CapsuleToSphere(q3Manifold *m, q3Capsule *a, q3Sphere *b);
void q3SphereToCapsule(q3Manifold *m, q3Sphere *a, q3Capsule *b);

#endif // Q3COLLIDE_H
