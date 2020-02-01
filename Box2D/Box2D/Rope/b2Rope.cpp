/*
* Copyright (c) 2011 Erin Catto http://box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <Box2D/Rope/b2Rope.h>
#include <Box2D/Common/b2Draw.h>

struct b2RopeStretch
{
    int32_t i1, i2;
    double invMass1, invMass2;
    double L;
};

struct b2RopeBend
{
    int32_t i1, i2, i3;
    double invMass1, invMass2, invMass3;
    double invEffectiveMass;
    double lambda;
    double L1, L2;
};

b2Rope::b2Rope()
{
    m_position.SetZero();
    m_count = 0;
    m_stretchCount = 0;
    m_bendCount = 0;
    m_stretchConstraints = nullptr;
    m_bendConstraints = nullptr;
    m_bindPositions = nullptr;
    m_ps = nullptr;
    m_p0s = nullptr;
    m_vs = nullptr;
    m_invMasses = nullptr;
    m_gravity.SetZero();
}

b2Rope::~b2Rope()
{
    b2Free(m_stretchConstraints);
    b2Free(m_bendConstraints);
    b2Free(m_bindPositions);
    b2Free(m_ps);
    b2Free(m_p0s);
    b2Free(m_vs);
    b2Free(m_invMasses);
}

void b2Rope::Create(const b2RopeDef& def)
{
    b2Assert(def.count >= 3);
    m_position = def.position;
    m_count = def.count;
    m_bindPositions = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
    m_ps = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
    m_p0s = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
    m_vs = (b2Vec2*)b2Alloc(m_count * sizeof(b2Vec2));
    m_invMasses = (double*)b2Alloc(m_count * sizeof(double));

    for (int32_t i = 0; i < m_count; ++i)
    {
        m_bindPositions[i] = def.vertices[i];
        m_ps[i] = def.vertices[i] + m_position;
        m_p0s[i] = def.vertices[i] + m_position;
        m_vs[i].SetZero();

        double m = def.masses[i];
        if (m > 0.0)
        {
            m_invMasses[i] = 1.0 / m;
        }
        else
        {
            m_invMasses[i] = 0.0;
        }
    }

    m_stretchCount = m_count - 1;
    m_bendCount = m_count - 2;

    m_stretchConstraints = (b2RopeStretch*)b2Alloc(m_stretchCount * sizeof(b2RopeStretch));
    m_bendConstraints = (b2RopeBend*)b2Alloc(m_bendCount * sizeof(b2RopeBend));


    for (int32_t i = 0; i < m_stretchCount; ++i)
    {
        b2Vec2 p1 = m_ps[i];
        b2Vec2 p2 = m_ps[i+1];

        m_stretchConstraints[i].i1 = i;
        m_stretchConstraints[i].i2 = i + 1;
        m_stretchConstraints[i].L = b2Distance(p1, p2);
        m_stretchConstraints[i].invMass1 = m_invMasses[i];
        m_stretchConstraints[i].invMass2 = m_invMasses[i + 1];
    }

    for (int32_t i = 0; i < m_bendCount; ++i)
    {
        b2RopeBend& c = m_bendConstraints[i];

        b2Vec2 p1 = m_ps[i];
        b2Vec2 p2 = m_ps[i + 1];
        b2Vec2 p3 = m_ps[i + 2];

        c.i1 = i;
        c.i2 = i + 1;
        c.i3 = i + 2;
        c.invMass1 = m_invMasses[i];
        c.invMass2 = m_invMasses[i + 1];
        c.invMass3 = m_invMasses[i + 2];
        c.invEffectiveMass = 0.0;
        c.L1 = b2Distance(p1, p2);
        c.L2 = b2Distance(p2, p3);
        c.lambda = 0.0;

        // Pre-compute effective mass
        b2Vec2 d1 = p2 - p1;
        b2Vec2 d2 = p3 - p2;
        double L1sqr = d1.LengthSquared();
        double L2sqr = d2.LengthSquared();

        if (L1sqr * L2sqr == 0.0)
        {
            continue;
        }

        double a = b2Cross(d1, d2);
        double b = b2Dot(d1, d2);

        b2Vec2 Jd1 = (-1.0 / L1sqr) * d1.Skew();
        b2Vec2 Jd2 = (1.0 / L2sqr) * d2.Skew();

        b2Vec2 J1 = -Jd1;
        b2Vec2 J2 = Jd1 - Jd2;
        b2Vec2 J3 = Jd2;

        c.invEffectiveMass = c.invMass1 * b2Dot(J1, J1) + c.invMass2 * b2Dot(J2, J2) + c.invMass3 * b2Dot(J3, J3);
    }

    m_gravity = def.gravity;
    m_tuning = def.tuning;
}

void b2Rope::SetTuning(const b2RopeTuning& tuning)
{
    m_tuning = tuning;
}

void b2Rope::Step(double dt, int32_t iterations, const b2Vec2& position)
{
    if (dt == 0.0)
    {
        return;
    }

    const double inv_dt = 1.0 / dt;
    double d = expf(- dt * m_tuning.damping);

    // Apply gravity and damping
    for (int32_t i = 0; i < m_count; ++i)
    {
        if (m_invMasses[i] > 0.0)
        {
            m_vs[i] += dt * m_gravity;
            m_vs[i] *= d;
        }
        else
        {
            m_vs[i] = inv_dt * (m_bindPositions[i] + position - m_p0s[i]);
        }
    }

    // Apply bending spring
    if (m_tuning.bendingModel == b2_springAngleBendingModel)
    {
        ApplyBendForces(dt);
    }

    if (m_tuning.bendingModel == b2_softAngleBendingModel && m_tuning.warmStart)
    {
        for (int32_t i = 0; i < m_bendCount; ++i)
        {
            const b2RopeBend& c = m_bendConstraints[i];

            b2Vec2 p1 = m_ps[c.i1];
            b2Vec2 p2 = m_ps[c.i2];
            b2Vec2 p3 = m_ps[c.i3];

            b2Vec2 d1 = p2 - p1;
            b2Vec2 d2 = p3 - p2;

            double L1sqr, L2sqr;

            if (m_tuning.isometric)
            {
                L1sqr = c.L1 * c.L1;
                L2sqr = c.L2 * c.L2;
            }
            else
            {
                L1sqr = d1.LengthSquared();
                L2sqr = d2.LengthSquared();
            }

            if (L1sqr * L2sqr == 0.0)
            {
                continue;
            }

            b2Vec2 Jd1 = (-1.0 / L1sqr) * d1.Skew();
            b2Vec2 Jd2 = (1.0 / L2sqr) * d2.Skew();

            b2Vec2 J1 = -Jd1;
            b2Vec2 J2 = Jd1 - Jd2;
            b2Vec2 J3 = Jd2;

            m_vs[c.i1] += (c.invMass1 * c.lambda) * J1;
            m_vs[c.i2] += (c.invMass2 * c.lambda) * J2;
            m_vs[c.i3] += (c.invMass3 * c.lambda) * J3;
        }
    }
    else
    {
        for (int32_t i = 0; i < m_bendCount; ++i)
        {
            m_bendConstraints[i].lambda = 0.0;
        }
    }

    // Update position
    for (int32_t i = 0; i < m_count; ++i)
    {
        m_ps[i] += dt * m_vs[i];
    }

    // Solve constraints
    for (int32_t i = 0; i < iterations; ++i)
    {
        if (m_tuning.bendingModel == b2_pbdAngleBendingModel)
        {
            SolveBend_PBD_Angle();
        }
        else if (m_tuning.bendingModel == b2_xpbdAngleBendingModel)
        {
            SolveBend_XPBD_Angle(dt);
        }
        else if (m_tuning.bendingModel == b2_softAngleBendingModel)
        {
            SolveBend_Soft_Angle(dt);
        }
        else if (m_tuning.bendingModel == b2_pbdDistanceBendingModel)
        {
            SolveBend_PBD_Distance();
        }
        else if (m_tuning.bendingModel == b2_pbdHeightBendingModel)
        {
            SolveBend_PBD_Height();
        }

        SolveStretch();
    }

    // Constrain velocity
    for (int32_t i = 0; i < m_count; ++i)
    {
        m_vs[i] = inv_dt * (m_ps[i] - m_p0s[i]);
        m_p0s[i] = m_ps[i];
    }
}

void b2Rope::Reset(const b2Vec2& position)
{
    m_position = position;

    for (int32_t i = 0; i < m_count; ++i)
    {
        m_ps[i] = m_bindPositions[i] + m_position;
        m_p0s[i] = m_bindPositions[i] + m_position;
        m_vs[i].SetZero();
    }

    for (int32_t i = 0; i < m_bendCount; ++i)
    {
        m_bendConstraints[i].lambda = 0.0;
    }
}

void b2Rope::SolveStretch()
{
    const double stiffness = m_tuning.stretchStiffness;

    for (int32_t i = 0; i < m_stretchCount; ++i)
    {
        const b2RopeStretch& c = m_stretchConstraints[i];

        b2Vec2 p1 = m_ps[c.i1];
        b2Vec2 p2 = m_ps[c.i2];

        b2Vec2 d = p2 - p1;
        double L = d.Normalize();

        double sum = c.invMass1 + c.invMass2;
        if (sum == 0.0)
        {
            continue;
        }

        double s1 = c.invMass1 / sum;
        double s2 = c.invMass2 / sum;

        p1 -= stiffness * s1 * (c.L - L) * d;
        p2 += stiffness * s2 * (c.L - L) * d;

        m_ps[c.i1] = p1;
        m_ps[c.i2] = p2;
    }
}

void b2Rope::SolveBend_PBD_Angle()
{
    const double stiffness = m_tuning.bendStiffness;

    for (int32_t i = 0; i < m_bendCount; ++i)
    {
        const b2RopeBend& c = m_bendConstraints[i];

        b2Vec2 p1 = m_ps[c.i1];
        b2Vec2 p2 = m_ps[c.i2];
        b2Vec2 p3 = m_ps[c.i3];

        b2Vec2 d1 = p2 - p1;
        b2Vec2 d2 = p3 - p2;
        double a = b2Cross(d1, d2);
        double b = b2Dot(d1, d2);

        double angle = b2Atan2(a, b);

        double L1sqr, L2sqr;

        if (m_tuning.isometric)
        {
            L1sqr = c.L1 * c.L1;
            L2sqr = c.L2 * c.L2;
        }
        else
        {
            L1sqr = d1.LengthSquared();
            L2sqr = d2.LengthSquared();
        }

        if (L1sqr * L2sqr == 0.0)
        {
            continue;
        }

        b2Vec2 Jd1 = (-1.0 / L1sqr) * d1.Skew();
        b2Vec2 Jd2 = (1.0 / L2sqr) * d2.Skew();

        b2Vec2 J1 = -Jd1;
        b2Vec2 J2 = Jd1 - Jd2;
        b2Vec2 J3 = Jd2;

        double sum;
        if (m_tuning.fixedEffectiveMass)
        {
            sum = c.invEffectiveMass;
        }
        else
        {
            sum = c.invMass1 * b2Dot(J1, J1) + c.invMass2 * b2Dot(J2, J2) + c.invMass3 * b2Dot(J3, J3);
        }

        if (sum == 0.0)
        {
            sum = c.invEffectiveMass;
        }

        double mass = 1.0 / sum;
        double C = angle;
        double impulse = -stiffness * mass * C;

        p1 += (c.invMass1 * impulse) * J1;
        p2 += (c.invMass2 * impulse) * J2;
        p3 += (c.invMass3 * impulse) * J3;

        m_ps[c.i1] = p1;
        m_ps[c.i2] = p2;
        m_ps[c.i3] = p3;
    }
}

void b2Rope::SolveBend_XPBD_Angle(double dt)
{
    b2Assert(dt > 0.0);

    // omega = 2 * pi * hz
    const double omega = 2.0 * b2_pi * m_tuning.bendHertz;

    for (int32_t i = 0; i < m_bendCount; ++i)
    {
        b2RopeBend& c = m_bendConstraints[i];

        b2Vec2 p1 = m_ps[c.i1];
        b2Vec2 p2 = m_ps[c.i2];
        b2Vec2 p3 = m_ps[c.i3];

        b2Vec2 dp1 = p1 - m_p0s[c.i1];
        b2Vec2 dp2 = p2 - m_p0s[c.i2];
        b2Vec2 dp3 = p3 - m_p0s[c.i3];

        b2Vec2 d1 = p2 - p1;
        b2Vec2 d2 = p3 - p2;

        double L1sqr, L2sqr;

        if (m_tuning.isometric)
        {
            L1sqr = c.L1 * c.L1;
            L2sqr = c.L2 * c.L2;
        }
        else
        {
            L1sqr = d1.LengthSquared();
            L2sqr = d2.LengthSquared();
        }

        if (L1sqr * L2sqr == 0.0)
        {
            continue;
        }

        double a = b2Cross(d1, d2);
        double b = b2Dot(d1, d2);

        double angle = b2Atan2(a, b);

        b2Vec2 Jd1 = (-1.0 / L1sqr) * d1.Skew();
        b2Vec2 Jd2 = (1.0 / L2sqr) * d2.Skew();

        b2Vec2 J1 = -Jd1;
        b2Vec2 J2 = Jd1 - Jd2;
        b2Vec2 J3 = Jd2;

        double sum;
        if (m_tuning.fixedEffectiveMass)
        {
            sum = c.invEffectiveMass;
        }
        else
        {
            sum = c.invMass1 * b2Dot(J1, J1) + c.invMass2 * b2Dot(J2, J2) + c.invMass3 * b2Dot(J3, J3);
        }

        if (sum == 0.0)
        {
            continue;
        }

        double mass = 1.0 / sum;

        const double spring = mass * omega * omega;
        const double damper = 2.0 * mass * m_tuning.bendDamping * omega;

        const double alpha = 1.0 / (spring * dt * dt);
        const double beta = dt * dt * damper;
        const double gamma = alpha * beta / dt;
        double C = angle;

        // This is using the initial velocities
        double Cdot = b2Dot(J1, dp1) + b2Dot(J2, dp2) + b2Dot(J3, dp3);

        double B = C + alpha * c.lambda + gamma * Cdot;
        double sum2 = (1.0 + alpha * beta / dt) * sum + alpha;

        double impulse = -B / sum2;

        p1 += (c.invMass1 * impulse) * J1;
        p2 += (c.invMass2 * impulse) * J2;
        p3 += (c.invMass3 * impulse) * J3;

        m_ps[c.i1] = p1;
        m_ps[c.i2] = p2;
        m_ps[c.i3] = p3;
        c.lambda += impulse;
    }
}

void b2Rope::SolveBend_Soft_Angle(double dt)
{
    b2Assert(dt > 0.0);

    const double inv_dt = 1.0 / dt;

    // omega = 2 * pi * hz
    const double omega = 2.0 * b2_pi * m_tuning.bendHertz;

    for (int32_t i = 0; i < m_bendCount; ++i)
    {
        b2RopeBend& c = m_bendConstraints[i];

        b2Vec2 p1 = m_ps[c.i1];
        b2Vec2 p2 = m_ps[c.i2];
        b2Vec2 p3 = m_ps[c.i3];

        b2Vec2 v1 = inv_dt * (p1 - m_p0s[c.i1]);
        b2Vec2 v2 = inv_dt * (p2 - m_p0s[c.i2]);
        b2Vec2 v3 = inv_dt * (p3 - m_p0s[c.i3]);

        b2Vec2 d1 = p2 - p1;
        b2Vec2 d2 = p3 - p2;

        double L1sqr, L2sqr;

        if (m_tuning.isometric)
        {
            L1sqr = c.L1 * c.L1;
            L2sqr = c.L2 * c.L2;
        }
        else
        {
            L1sqr = d1.LengthSquared();
            L2sqr = d2.LengthSquared();
        }

        if (L1sqr * L2sqr == 0.0)
        {
            continue;
        }

        double a = b2Cross(d1, d2);
        double b = b2Dot(d1, d2);

        double C = b2Atan2(a, b);

        b2Vec2 Jd1 = (-1.0 / L1sqr) * d1.Skew();
        b2Vec2 Jd2 = (1.0 / L2sqr) * d2.Skew();

        b2Vec2 J1 = -Jd1;
        b2Vec2 J2 = Jd1 - Jd2;
        b2Vec2 J3 = Jd2;

        double sum = c.invMass1 * b2Dot(J1, J1) + c.invMass2 * b2Dot(J2, J2) + c.invMass3 * b2Dot(J3, J3);
        if (sum == 0.0)
        {
            continue;
        }

        double mass = 1.0 / sum;

        const double spring = mass * omega * omega;
        const double damper = 2.0 * mass * m_tuning.bendDamping * omega;

        double gamma = dt * (damper + dt * spring);
        if (gamma != 0.0)
        {
            gamma = 1.0 / gamma;
        }
        mass = 1.0 / (sum + gamma);

        double bias = C * dt * spring * gamma;

        // This is using the initial velocities
        double Cdot = b2Dot(J1, v1) + b2Dot(J2, v2) + b2Dot(J3, v3);

        double impulse = -mass * (Cdot + bias + gamma * c.lambda);
        v1 += (c.invMass1 * impulse) * J1;
        v2 += (c.invMass2 * impulse) * J2;
        v3 += (c.invMass3 * impulse) * J3;

        m_ps[c.i1] = m_p0s[c.i1] + dt * v1;
        m_ps[c.i2] = m_p0s[c.i2] + dt * v2;
        m_ps[c.i3] = m_p0s[c.i3] + dt * v3;
        c.lambda += impulse;
    }
}

void b2Rope::ApplyBendForces(double dt)
{
    // omega = 2 * pi * hz
    const double omega = 2.0 * b2_pi * m_tuning.bendHertz;

    for (int32_t i = 0; i < m_bendCount; ++i)
    {
        const b2RopeBend& c = m_bendConstraints[i];

        b2Vec2 p1 = m_ps[c.i1];
        b2Vec2 p2 = m_ps[c.i2];
        b2Vec2 p3 = m_ps[c.i3];

        b2Vec2 v1 = m_vs[c.i1];
        b2Vec2 v2 = m_vs[c.i2];
        b2Vec2 v3 = m_vs[c.i3];

        b2Vec2 d1 = p2 - p1;
        b2Vec2 d2 = p3 - p2;

        double L1sqr, L2sqr;

        if (m_tuning.isometric)
        {
            L1sqr = c.L1 * c.L1;
            L2sqr = c.L2 * c.L2;
        }
        else
        {
            L1sqr = d1.LengthSquared();
            L2sqr = d2.LengthSquared();
        }

        if (L1sqr * L2sqr == 0.0)
        {
            continue;
        }

        double a = b2Cross(d1, d2);
        double b = b2Dot(d1, d2);

        double angle = b2Atan2(a, b);

        b2Vec2 Jd1 = (-1.0 / L1sqr) * d1.Skew();
        b2Vec2 Jd2 = (1.0 / L2sqr) * d2.Skew();

        b2Vec2 J1 = -Jd1;
        b2Vec2 J2 = Jd1 - Jd2;
        b2Vec2 J3 = Jd2;

        double sum;
        if (m_tuning.fixedEffectiveMass)
        {
            sum = c.invEffectiveMass;
        }
        else
        {
            sum = c.invMass1 * b2Dot(J1, J1) + c.invMass2 * b2Dot(J2, J2) + c.invMass3 * b2Dot(J3, J3);
        }

        if (sum == 0.0)
        {
            continue;
        }

        double mass = 1.0 / sum;

        const double spring = mass * omega * omega;
        const double damper = 2.0 * mass * m_tuning.bendDamping * omega;

        double C = angle;
        double Cdot = b2Dot(J1, v1) + b2Dot(J2, v2) + b2Dot(J3, v3);

        double impulse = -dt * (spring * C + damper * Cdot);

        m_vs[c.i1] += (c.invMass1 * impulse) * J1;
        m_vs[c.i2] += (c.invMass2 * impulse) * J2;
        m_vs[c.i3] += (c.invMass3 * impulse) * J3;
    }
}

void b2Rope::SolveBend_PBD_Distance()
{
    const double stiffness = m_tuning.bendStiffness;

    for (int32_t i = 0; i < m_bendCount; ++i)
    {
        const b2RopeBend& c = m_bendConstraints[i];

        int32_t i1 = c.i1;
        int32_t i2 = c.i3;

        b2Vec2 p1 = m_ps[i1];
        b2Vec2 p2 = m_ps[i2];

        b2Vec2 d = p2 - p1;
        double L = d.Normalize();

        double sum = c.invMass1 + c.invMass3;
        if (sum == 0.0)
        {
            continue;
        }

        double s1 = c.invMass1 / sum;
        double s2 = c.invMass3 / sum;

        p1 -= stiffness * s1 * (c.L1 + c.L2 - L) * d;
        p2 += stiffness * s2 * (c.L1 + c.L2 - L) * d;

        m_ps[i1] = p1;
        m_ps[i2] = p2;
    }
}

void b2Rope::SolveBend_PBD_Height()
{
    const double stiffness = m_tuning.bendStiffness;

    for (int32_t i = 0; i < m_bendCount; ++i)
    {
        const b2RopeBend& c = m_bendConstraints[i];

        b2Vec2 p1 = m_ps[c.i1];
        b2Vec2 p2 = m_ps[c.i2];
        b2Vec2 p3 = m_ps[c.i3];

        b2Vec2 e1 = p2 - p1;
        b2Vec2 e2 = p3 - p2;
        b2Vec2 r = p3 - p1;

        double rr = r.LengthSquared();
        if (rr == 0.0)
        {
            continue;
        }

        double alpha = b2Dot(e2, r) / rr;
        double beta = b2Dot(e1, r) / rr;
        b2Vec2 d = alpha * p1 + beta * p3 - p2;

        double dLen = d.Length();

        if (dLen == 0.0)
        {
            continue;
        }

        b2Vec2 dHat = (1.0 / dLen) * d;

        b2Vec2 J1 = alpha * dHat;
        b2Vec2 J2 = -dHat;
        b2Vec2 J3 = beta * dHat;

        double sum = c.invMass1 * b2Dot(J1, J1) + c.invMass2 * b2Dot(J2, J2) + c.invMass3 * b2Dot(J3, J3);

        if (sum == 0.0)
        {
            continue;
        }

        double C = dLen;
        double mass = 1.0 / sum;
        double impulse = -stiffness * mass * C;

        p1 += (c.invMass1 * impulse) * J1;
        p2 += (c.invMass2 * impulse) * J2;
        p3 += (c.invMass3 * impulse) * J3;

        m_ps[c.i1] = p1;
        m_ps[c.i2] = p2;
        m_ps[c.i3] = p3;
    }
}
void b2Rope::Draw(b2Draw* draw) const
{
    b2Color c(0.4, 0.5, 0.7);
    b2Color pg(0.1, 0.8, 0.1);
    b2Color pd(0.7, 0.2, 0.4);

    for (int32_t i = 0; i < m_count - 1; ++i)
    {
        draw->DrawSegment(m_ps[i], m_ps[i+1], c);

        const b2Color& pc = m_invMasses[i] > 0.0 ? pd : pg;
        draw->DrawPoint(m_ps[i], 5.0, pc);
    }

    const b2Color& pc = m_invMasses[m_count - 1] > 0.0 ? pd : pg;
    draw->DrawPoint(m_ps[m_count - 1], 5.0, pc);
}
