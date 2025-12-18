//------------------------------------------------------------------------------
/**
@file	q3Mat3.cpp

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

#include "q3Mat3.h"
#include <cassert>

//------------------------------------------------------------------------------
// q3Mat3
//------------------------------------------------------------------------------
q3Mat3::q3Mat3() {}

//------------------------------------------------------------------------------
q3Mat3::q3Mat3(r32 a, r32 b, r32 c, r32 d, r32 e, r32 f, r32 g, r32 h, r32 i)
    : col0(a, b, c), col1(d, e, f), col2(g, h, i)
{
}

//------------------------------------------------------------------------------
q3Mat3::q3Mat3(const q3Vec3 &_col0, const q3Vec3 &_col1, const q3Vec3 &_col2)
    : col0(_col0), col1(_col1), col2(_col2)
{
}

//------------------------------------------------------------------------------
void q3Mat3::Set(r32 a, r32 b, r32 c, r32 d, r32 e, r32 f, r32 g, r32 h, r32 i)
{
    col0.Set(a, b, c);
    col1.Set(d, e, f);
    col2.Set(g, h, i);
}

//------------------------------------------------------------------------------
void q3Mat3::Set(const q3Vec3 &axis, r32 angle)
{
    r32 s  = sin(angle);
    r32 c  = cos(angle);
    r32 x  = axis.x;
    r32 y  = axis.y;
    r32 z  = axis.z;
    r32 xy = x * y;
    r32 yz = y * z;
    r32 zx = z * x;
    r32 t  = r32(1.0) - c;

    Set(x * x * t + c,
        xy * t + z * s,
        zx * t - y * s,
        xy * t - z * s,
        y * y * t + c,
        yz * t + x * s,
        zx * t + y * s,
        yz * t - x * s,
        z * z * t + c);
}

//------------------------------------------------------------------------------
void q3Mat3::SetColumns(const q3Vec3 &x, const q3Vec3 &y, const q3Vec3 &z)
{
    col0 = x;
    col1 = y;
    col2 = z;
}

//------------------------------------------------------------------------------
void q3Mat3::SetRows(const q3Vec3 &x, const q3Vec3 &y, const q3Vec3 &z)
{
    col0.Set(x.x, y.x, z.x);
    col1.Set(x.y, y.y, z.y);
    col2.Set(x.z, y.z, z.z);
}

//------------------------------------------------------------------------------
q3Mat3 &q3Mat3::operator=(const q3Mat3 &rhs)
{
    col0 = rhs.col0;
    col1 = rhs.col1;
    col2 = rhs.col2;

    return *this;
}

//------------------------------------------------------------------------------
q3Mat3 &q3Mat3::operator*=(const q3Mat3 &rhs)
{
    *this = *this * rhs;

    return *this;
}

//------------------------------------------------------------------------------
q3Mat3 &q3Mat3::operator*=(r32 f)
{
    col0 *= f;
    col1 *= f;
    col2 *= f;

    return *this;
}

//------------------------------------------------------------------------------
q3Mat3 &q3Mat3::operator+=(const q3Mat3 &rhs)
{
    col0 += rhs.col0;
    col1 += rhs.col1;
    col2 += rhs.col2;

    return *this;
}

//------------------------------------------------------------------------------
q3Mat3 &q3Mat3::operator-=(const q3Mat3 &rhs)
{
    col0 -= rhs.col0;
    col1 -= rhs.col1;
    col2 -= rhs.col2;

    return *this;
}

//------------------------------------------------------------------------------
q3Vec3 &q3Mat3::operator[](u32 index)
{
    switch (index)
    {
    case 0:
        return col0;
    case 1:
        return col1;
    case 2:
        return col2;
    default:
        assert(false);
        return col0;
    }
}

//------------------------------------------------------------------------------
const q3Vec3 &q3Mat3::operator[](u32 index) const
{
    switch (index)
    {
    case 0:
        return col0;
    case 1:
        return col1;
    case 2:
        return col2;
    default:
        assert(false);
        return col0;
    }
}

//------------------------------------------------------------------------------
const q3Vec3 q3Mat3::Column0() const { return col0; }

//------------------------------------------------------------------------------
const q3Vec3 q3Mat3::Column1() const { return col1; }

//------------------------------------------------------------------------------
const q3Vec3 q3Mat3::Column2() const { return col2; }

//------------------------------------------------------------------------------
const q3Vec3 q3Mat3::Row0() const { return q3Vec3(col0.x, col1.x, col2.x); }

//------------------------------------------------------------------------------
const q3Vec3 q3Mat3::Row1() const { return q3Vec3(col0.y, col1.y, col2.y); }

//------------------------------------------------------------------------------
const q3Vec3 q3Mat3::Row2() const { return q3Vec3(col0.z, col1.z, col2.z); }

//------------------------------------------------------------------------------
const q3Vec3 q3Mat3::operator*(const q3Vec3 &rhs) const
{
    // M * v (Standard Column-Major)
    return col0 * rhs.x + col1 * rhs.y + col2 * rhs.z;
}

//------------------------------------------------------------------------------
const q3Mat3 q3Mat3::operator*(const q3Mat3 &rhs) const
{
    // Matrix-Matrix multiplication: Columns of result are M * rhs.col
    return q3Mat3(*this * rhs.col0, *this * rhs.col1, *this * rhs.col2);
}

//------------------------------------------------------------------------------
const q3Mat3 q3Mat3::operator*(r32 f) const
{
    return q3Mat3(col0 * f, col1 * f, col2 * f);
}

//------------------------------------------------------------------------------
const q3Mat3 q3Mat3::operator+(const q3Mat3 &rhs) const
{
    return q3Mat3(col0 + rhs.col0, col1 + rhs.col1, col2 + rhs.col2);
}

//------------------------------------------------------------------------------
const q3Mat3 q3Mat3::operator-(const q3Mat3 &rhs) const
{
    return q3Mat3(col0 - rhs.col0, col1 - rhs.col1, col2 - rhs.col2);
}
