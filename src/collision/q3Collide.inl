#ifndef Q3COLLIDE_INL
#define Q3COLLIDE_INL

inline bool q3TrackFaceAxis(i32 *axis,
                            i32 n,
                            r32 s,
                            r32 *sMax,
                            const q3Vec3 &normal,
                            q3Vec3 *axisNormal)
{
    if (s > r32(0.0))
        return true;

    if (s > *sMax)
    {
        *sMax       = s;
        *axis       = n;
        *axisNormal = normal;
    }

    return false;
}

//------------------------------------------------------------------------------
inline bool q3TrackEdgeAxis(i32 *axis,
                            i32 n,
                            r32 s,
                            r32 *sMax,
                            const q3Vec3 &normal,
                            q3Vec3 *axisNormal)
{
    if (s > r32(0.0))
        return true;

    r32 l = r32(1.0) / q3Length(normal);
    s *= l;

    if (s > *sMax)
    {
        *sMax       = s;
        *axis       = n;
        *axisNormal = normal * l;
    }

    return false;
}

//------------------------------------------------------------------------------
inline void q3EdgesContact(q3Vec3 *CA,
                           q3Vec3 *CB,
                           const q3Vec3 &PA,
                           const q3Vec3 &QA,
                           const q3Vec3 &PB,
                           const q3Vec3 &QB)
{
    q3Vec3 DA = QA - PA;
    q3Vec3 DB = QB - PB;
    q3Vec3 r  = PA - PB;
    r32 a     = q3Dot(DA, DA);
    r32 e     = q3Dot(DB, DB);
    r32 f     = q3Dot(DB, r);
    r32 c     = q3Dot(DA, r);

    r32 b     = q3Dot(DA, DB);
    r32 denom = a * e - b * b;

    r32 TA = (b * f - c * e) / denom;
    r32 TB = (b * TA + f) / e;

    *CA = PA + DA * TA;
    *CB = PB + DB * TB;
}

//------------------------------------------------------------------------------
inline void q3SupportEdge(const q3Transform &tx,
                          const q3Vec3 &e,
                          q3Vec3 n,
                          q3Vec3 *aOut,
                          q3Vec3 *bOut)
{
    n           = q3MulT(tx.rotation, n);
    q3Vec3 absN = q3Abs(n);
    q3Vec3 a, b;

    // x > y
    if (absN.x > absN.y)
    {
        // x > y > z
        if (absN.y > absN.z)
        {
            a.Set(e.x, e.y, e.z);
            b.Set(e.x, e.y, -e.z);
        }

        // x > z > y || z > x > y
        else
        {
            a.Set(e.x, e.y, e.z);
            b.Set(e.x, -e.y, e.z);
        }
    }

    // y > x
    else
    {
        // y > x > z
        if (absN.x > absN.z)
        {
            a.Set(e.x, e.y, e.z);
            b.Set(e.x, e.y, -e.z);
        }

        // z > y > x || y > z > x
        else
        {
            a.Set(e.x, e.y, e.z);
            b.Set(-e.x, e.y, e.z);
        }
    }

    r32 signx = q3Sign(n.x);
    r32 signy = q3Sign(n.y);
    r32 signz = q3Sign(n.z);

    a.x *= signx;
    a.y *= signy;
    a.z *= signz;
    b.x *= signx;
    b.y *= signy;
    b.z *= signz;

    *aOut = q3Mul(tx, a);
    *bOut = q3Mul(tx, b);
}

//------------------------------------------------------------------------------
// Internal helper: Closest point on triangle to a point p
// Triangle defined by a, b, c
inline q3Vec3 q3ClosestPointToTriangle(const q3Vec3 &p,
                                       const q3Vec3 &a,
                                       const q3Vec3 &b,
                                       const q3Vec3 &c)
{
    q3Vec3 ab = b - a;
    q3Vec3 ac = c - a;
    q3Vec3 ap = p - a;
    r32 d1    = q3Dot(ab, ap);
    r32 d2    = q3Dot(ac, ap);

    if (d1 <= 0.0f && d2 <= 0.0f)
        return a; // vertex region a

    q3Vec3 bp = p - b;
    r32 d3    = q3Dot(ab, bp);
    r32 d4    = q3Dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3)
        return b; // vertex region b

    r32 vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
    {
        r32 v = d1 / (d1 - d3);
        return a + ab * v; // edge region ab
    }

    q3Vec3 cp = p - c;
    r32 d5    = q3Dot(ab, cp);
    r32 d6    = q3Dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6)
        return c; // vertex region c

    r32 vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
    {
        r32 w = d2 / (d2 - d6);
        return a + ac * w; // edge region ac
    }

    r32 va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
    {
        r32 w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w; // edge region bc
    }

    r32 denom = r32(1.0) / (va + vb + vc);
    r32 v     = vb * denom;
    r32 w     = vc * denom;
    return a + ab * v + ac * w; // face region
}

//------------------------------------------------------------------------------
// Helper: Closest points between two segments p1-q1 and p2-q2
// Returns squared distance, computes closest points c1 and c2
inline r32 q3ClosestPointSegmentSegment(const q3Vec3 &p1,
                                        const q3Vec3 &q1,
                                        const q3Vec3 &p2,
                                        const q3Vec3 &q2,
                                        q3Vec3 *c1,
                                        q3Vec3 *c2)
{
    q3Vec3 d1 = q1 - p1;
    q3Vec3 d2 = q2 - p2;
    q3Vec3 r  = p1 - p2;
    r32 a     = q3Dot(d1, d1);
    r32 e     = q3Dot(d2, d2);
    r32 f     = q3Dot(d2, r);

    const r32 epsilon = 1.0e-5f;

    r32 s, t;

    if (a <= epsilon && e <= epsilon)
    {
        s = t    = 0.0f;
        *c1      = p1;
        *c2      = p2;
        q3Vec3 d = *c1 - *c2;
        return q3Dot(d, d);
    }

    if (a <= epsilon)
    {
        s = 0.0f;
        t = f / e;
        t = q3Clamp(t, 0.0f, 1.0f);
    }
    else
    {
        r32 c = q3Dot(d1, r);
        if (e <= epsilon)
        {
            t = 0.0f;
            s = q3Clamp(-c / a, 0.0f, 1.0f);
        }
        else
        {
            r32 b     = q3Dot(d1, d2);
            r32 denom = a * e - b * b;

            if (denom != 0.0f)
            {
                s = q3Clamp((b * f - c * e) / denom, 0.0f, 1.0f);
            }
            else
            {
                s = 0.0f;
            }

            t = (b * s + f) / e;

            if (t < 0.0f)
            {
                t = 0.0f;
                s = q3Clamp(-c / a, 0.0f, 1.0f);
            }
            else if (t > 1.0f)
            {
                t = 1.0f;
                s = q3Clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    *c1      = p1 + d1 * s;
    *c2      = p2 + d2 * t;
    q3Vec3 d = *c1 - *c2;
    return q3Dot(d, d);
}

#endif // Q3COLLIDE_INL