//------------------------------------------------------------------------------
/**
@file	q3Mat3.inl

@author	Randy Gaul
@date	10/10/2014
Copyright (c) 2014 Randy Gaul http://www.randygaul.net

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

//------------------------------------------------------------------------------
// q3Mat3
//------------------------------------------------------------------------------
inline void q3Identity(q3Mat3 &m)
{
    m.Set(r32(1.0),
          r32(0.0),
          r32(0.0),
          r32(0.0),
          r32(1.0),
          r32(0.0),
          r32(0.0),
          r32(0.0),
          r32(1.0));
}

//------------------------------------------------------------------------------
inline const q3Mat3 q3Rotate(const q3Vec3 &x, const q3Vec3 &y, const q3Vec3 &z)
{
    return q3Mat3(x, y, z);
}

//------------------------------------------------------------------------------
inline const q3Mat3 q3Transpose(const q3Mat3 &m)
{
    // Transpose: Swapping rows and columns
    return q3Mat3(m.col0.x,
                  m.col1.x,
                  m.col2.x,
                  m.col0.y,
                  m.col1.y,
                  m.col2.y,
                  m.col0.z,
                  m.col1.z,
                  m.col2.z);
}

//------------------------------------------------------------------------------
inline void q3Zero(q3Mat3 &m) { memset(&m, 0, sizeof(r32) * 9); }

//------------------------------------------------------------------------------
inline const q3Mat3 q3Diagonal(r32 a)
{
    return q3Mat3(r32(a),
                  r32(0.0),
                  r32(0.0),
                  r32(0.0),
                  r32(a),
                  r32(0.0),
                  r32(0.0),
                  r32(0.0),
                  r32(a));
}

//------------------------------------------------------------------------------
inline const q3Mat3 q3Diagonal(r32 a, r32 b, r32 c)
{
    return q3Mat3(r32(a),
                  r32(0.0),
                  r32(0.0),
                  r32(0.0),
                  r32(b),
                  r32(0.0),
                  r32(0.0),
                  r32(0.0),
                  r32(c));
}

//------------------------------------------------------------------------------
inline const q3Mat3 q3OuterProduct(const q3Vec3 &u, const q3Vec3 &v)
{
    // u * v^T
    // [ u.x * v.x   u.x * v.y   u.x * v.z ]
    // [ u.y * v.x   u.y * v.y   u.y * v.z ]
    // [ u.z * v.x   u.z * v.y   u.z * v.z ]

    // In column-major constructor:
    // q3Mat3(col0, col1, col2)
    // col0 = u * v.x
    // col1 = u * v.y
    // col2 = u * v.z

    q3Vec3 a = u * v.x;
    q3Vec3 b = u * v.y;
    q3Vec3 c = u * v.z;

    return q3Mat3(a, b, c);
}

//------------------------------------------------------------------------------
inline const q3Mat3 q3Covariance(q3Vec3 *points, u32 numPoints)
{
    r32 invNumPoints = r32(1.0) / r32(numPoints);
    q3Vec3 c         = q3Vec3(r32(0.0), r32(0.0), r32(0.0));

    for (u32 i = 0; i < numPoints; ++i)
        c += points[i];

    c /= r32(numPoints);

    r32 m00, m11, m22, m01, m02, m12;
    m00 = m11 = m22 = m01 = m02 = m12 = r32(0.0);

    for (u32 i = 0; i < numPoints; ++i)
    {
        q3Vec3 p = points[i] - c;

        m00 += p.x * p.x;
        m11 += p.y * p.y;
        m22 += p.z * p.z;
        m01 += p.x * p.y;
        m02 += p.x * p.z;
        m12 += p.y * p.z;
    }

    r32 m01inv = m01 * invNumPoints;
    r32 m02inv = m02 * invNumPoints;
    r32 m12inv = m12 * invNumPoints;

    return q3Mat3(m00 * invNumPoints,
                  m01inv,
                  m02inv,
                  m01inv,
                  m11 * invNumPoints,
                  m12inv,
                  m02inv,
                  m12inv,
                  m22 * invNumPoints);
};

//------------------------------------------------------------------------------
inline const q3Mat3 q3Inverse(const q3Mat3 &m)
{
    // Det = dot(col0, cross(col1, col2))
    q3Vec3 tmp0, tmp1, tmp2;
    r32 detinv;

    tmp0 = q3Cross(m.col1, m.col2);
    tmp1 = q3Cross(m.col2, m.col0);
    tmp2 = q3Cross(m.col0, m.col1);

    detinv = r32(1.0) / q3Dot(m.col2, tmp2);

    // Inverse = Transpose(Cofactors) / Det
    // In Column major, Cofactors are columns of the cofactor matrix.
    // We already computed cross products which are related to cofactors.

    return q3Mat3(tmp0.x * detinv,
                  tmp1.x * detinv,
                  tmp2.x * detinv,
                  tmp0.y * detinv,
                  tmp1.y * detinv,
                  tmp2.y * detinv,
                  tmp0.z * detinv,
                  tmp1.z * detinv,
                  tmp2.z * detinv);
}
