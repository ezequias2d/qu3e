//------------------------------------------------------------------------------
/**
@file	q3Contact.cpp

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

#include "q3Contact.h"
#include "../collision/q3Shape.h"

//------------------------------------------------------------------------------
// q3Contact
//------------------------------------------------------------------------------
void q3Manifold::SetPair(q3Shape *a, q3Shape *b)
{
    A = a;
    B = b;

    sensor = A->sensor || B->sensor;
}

// Generate contact information
void q3ContactConstraint::SolveCollision(void)
{
    manifold.contactCount = 0;

    q3ShapeType typeA = A->m_type;
    q3ShapeType typeB = B->m_type;

    if (typeA == eBox && typeB == eBox)
    {
        q3BoxtoBox(&manifold, (q3Box *)A, (q3Box *)B);
    }
    else if (typeA == eSphere && typeB == eSphere)
    {
        q3SphereToSphere(&manifold, (q3Sphere *)A, (q3Sphere *)B);
    }
    else if (typeA == eSphere && typeB == eBox)
    {
        q3SphereToBox(&manifold, (q3Sphere *)A, (q3Box *)B);
    }
    else if (typeA == eBox && typeB == eSphere)
    {
        q3BoxToSphere(&manifold, (q3Box *)A, (q3Sphere *)B);
    }
    else if (typeA == eCapsule && typeB == eCapsule)
    {
        q3CapsuleToCapsule(&manifold, (q3Capsule *)A, (q3Capsule *)B);
    }
    else if (typeA == eCapsule && typeB == eBox)
    {
        q3CapsuleToBox(&manifold, (q3Capsule *)A, (q3Box *)B);
    }
    else if (typeA == eBox && typeB == eCapsule)
    {
        q3BoxToCapsule(&manifold, (q3Box *)A, (q3Capsule *)B);
    }
    else if (typeA == eCapsule && typeB == eSphere)
    {
        q3CapsuleToSphere(&manifold, (q3Capsule *)A, (q3Sphere *)B);
    }
    else if (typeA == eSphere && typeB == eCapsule)
    {
        q3SphereToCapsule(&manifold, (q3Sphere *)A, (q3Capsule *)B);
    }

    if (manifold.contactCount > 0)
    {
        if (m_flags & eColliding)
            m_flags |= eWasColliding;

        else
            m_flags |= eColliding;
    }

    else
    {
        if (m_flags & eColliding)
        {
            m_flags &= ~eColliding;
            m_flags |= eWasColliding;
        }

        else
            m_flags &= ~eWasColliding;
    }
}
