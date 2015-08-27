/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/Joints/b2WheelJoint.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2TimeStep.h>

// Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(ay, d)
// Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
// J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

// Spring linear constraint
// C = dot(ax, d)
// Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
// J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

// Motor rotational constraint
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]

void b2WheelJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor, const b2Vec2& axis)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
	localAxisA = bodyA->GetLocalVector(axis);
}

b2WheelJoint::b2WheelJoint(const b2WheelJointDef* def)
: b2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_localXAxisA = def->localAxisA;
	m_localYAxisA = b2Cross(1.0, m_localXAxisA);

	m_mass = 0.0;
	m_impulse = 0.0;
	m_motorMass = 0.0;
	m_motorImpulse = 0.0;
	m_springMass = 0.0;
	m_springImpulse = 0.0;

	m_maxMotorTorque = def->maxMotorTorque;
	m_motorSpeed = def->motorSpeed;
	m_enableMotor = def->enableMotor;

	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;

	m_bias = 0.0;
	m_gamma = 0.0;

	m_ax.SetZero();
	m_ay.SetZero();
}

void b2WheelJoint::InitVelocityConstraints(const b2SolverData& data)
{
	m_indexA = m_bodyA->m_islandIndex;
	m_indexB = m_bodyB->m_islandIndex;
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassA = m_bodyA->m_invMass;
	m_invMassB = m_bodyB->m_invMass;
	m_invIA = m_bodyA->m_invI;
	m_invIB = m_bodyB->m_invI;

	double mA = m_invMassA, mB = m_invMassB;
	double iA = m_invIA, iB = m_invIB;

	b2Vec2 cA = data.positions[m_indexA].c;
	double aA = data.positions[m_indexA].a;
	b2Vec2 vA = data.velocities[m_indexA].v;
	double wA = data.velocities[m_indexA].w;

	b2Vec2 cB = data.positions[m_indexB].c;
	double aB = data.positions[m_indexB].a;
	b2Vec2 vB = data.velocities[m_indexB].v;
	double wB = data.velocities[m_indexB].w;

	b2Rot qA(aA), qB(aB);

	// Compute the effective masses.
	b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
	b2Vec2 d = cB + rB - cA - rA;

	// Point to line constraint
	{
		m_ay = b2Mul(qA, m_localYAxisA);
		m_sAy = b2Cross(d + rA, m_ay);
		m_sBy = b2Cross(rB, m_ay);

		m_mass = mA + mB + iA * m_sAy * m_sAy + iB * m_sBy * m_sBy;

		if (m_mass > 0.0)
		{
			m_mass = 1.0 / m_mass;
		}
	}

	// Spring constraint
	m_springMass = 0.0;
	m_bias = 0.0;
	m_gamma = 0.0;
	if (m_frequencyHz > 0.0)
	{
		m_ax = b2Mul(qA, m_localXAxisA);
		m_sAx = b2Cross(d + rA, m_ax);
		m_sBx = b2Cross(rB, m_ax);

		double invMass = mA + mB + iA * m_sAx * m_sAx + iB * m_sBx * m_sBx;

		if (invMass > 0.0)
		{
			m_springMass = 1.0 / invMass;

			double C = b2Dot(d, m_ax);

			// Frequency
			double omega = 2.0 * b2_pi * m_frequencyHz;

			// Damping coefficient
			double damping = 2.0 * m_springMass * m_dampingRatio * omega;

			// Spring stiffness
			double k = m_springMass * omega * omega;

			// magic formulas
			double h = data.step.dt;
			m_gamma = h * (damping + h * k);
			if (m_gamma > 0.0)
			{
				m_gamma = 1.0 / m_gamma;
			}

			m_bias = C * h * k * m_gamma;

			m_springMass = invMass + m_gamma;
			if (m_springMass > 0.0)
			{
				m_springMass = 1.0 / m_springMass;
			}
		}
	}
	else
	{
		m_springImpulse = 0.0;
	}

	// Rotational motor
	if (m_enableMotor)
	{
		m_motorMass = iA + iB;
		if (m_motorMass > 0.0)
		{
			m_motorMass = 1.0 / m_motorMass;
		}
	}
	else
	{
		m_motorMass = 0.0;
		m_motorImpulse = 0.0;
	}

	if (data.step.warmStarting)
	{
		// Account for variable time step.
		m_impulse *= data.step.dtRatio;
		m_springImpulse *= data.step.dtRatio;
		m_motorImpulse *= data.step.dtRatio;

		b2Vec2 P = m_impulse * m_ay + m_springImpulse * m_ax;
		double LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
		double LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;

		vA -= m_invMassA * P;
		wA -= m_invIA * LA;

		vB += m_invMassB * P;
		wB += m_invIB * LB;
	}
	else
	{
		m_impulse = 0.0;
		m_springImpulse = 0.0;
		m_motorImpulse = 0.0;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void b2WheelJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	double mA = m_invMassA, mB = m_invMassB;
	double iA = m_invIA, iB = m_invIB;

	b2Vec2 vA = data.velocities[m_indexA].v;
	double wA = data.velocities[m_indexA].w;
	b2Vec2 vB = data.velocities[m_indexB].v;
	double wB = data.velocities[m_indexB].w;

	// Solve spring constraint
	{
		double Cdot = b2Dot(m_ax, vB - vA) + m_sBx * wB - m_sAx * wA;
		double impulse = -m_springMass * (Cdot + m_bias + m_gamma * m_springImpulse);
		m_springImpulse += impulse;

		b2Vec2 P = impulse * m_ax;
		double LA = impulse * m_sAx;
		double LB = impulse * m_sBx;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	// Solve rotational motor constraint
	{
		double Cdot = wB - wA - m_motorSpeed;
		double impulse = -m_motorMass * Cdot;

		double oldImpulse = m_motorImpulse;
		double maxImpulse = data.step.dt * m_maxMotorTorque;
		m_motorImpulse = b2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		wA -= iA * impulse;
		wB += iB * impulse;
	}

	// Solve point to line constraint
	{
		double Cdot = b2Dot(m_ay, vB - vA) + m_sBy * wB - m_sAy * wA;
		double impulse = -m_mass * Cdot;
		m_impulse += impulse;

		b2Vec2 P = impulse * m_ay;
		double LA = impulse * m_sAy;
		double LB = impulse * m_sBy;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool b2WheelJoint::SolvePositionConstraints(const b2SolverData& data)
{
	b2Vec2 cA = data.positions[m_indexA].c;
	double aA = data.positions[m_indexA].a;
	b2Vec2 cB = data.positions[m_indexB].c;
	double aB = data.positions[m_indexB].a;

	b2Rot qA(aA), qB(aB);

	b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
	b2Vec2 d = (cB - cA) + rB - rA;

	b2Vec2 ay = b2Mul(qA, m_localYAxisA);

	double sAy = b2Cross(d + rA, ay);
	double sBy = b2Cross(rB, ay);

	double C = b2Dot(d, ay);

	double k = m_invMassA + m_invMassB + m_invIA * m_sAy * m_sAy + m_invIB * m_sBy * m_sBy;

	double impulse;
	if (k != 0.0)
	{
		impulse = - C / k;
	}
	else
	{
		impulse = 0.0;
	}

	b2Vec2 P = impulse * ay;
	double LA = impulse * sAy;
	double LB = impulse * sBy;

	cA -= m_invMassA * P;
	aA -= m_invIA * LA;
	cB += m_invMassB * P;
	aB += m_invIB * LB;

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;

	return b2Abs(C) <= b2_linearSlop;
}

b2Vec2 b2WheelJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec2 b2WheelJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2WheelJoint::GetReactionForce(double inv_dt) const
{
	return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
}

double b2WheelJoint::GetReactionTorque(double inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

double b2WheelJoint::GetPrismaticJointTranslation() const
{
	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;

	b2Vec2 pA = bA->GetWorldPoint(m_localAnchorA);
	b2Vec2 pB = bB->GetWorldPoint(m_localAnchorB);
	b2Vec2 d = pB - pA;
	b2Vec2 axis = bA->GetWorldVector(m_localXAxisA);

	double translation = b2Dot(d, axis);
	return translation;
}

double b2WheelJoint::GetPrismaticJointSpeed() const
{
	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;

	b2Vec2 rA = b2Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
	b2Vec2 rB = b2Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
	b2Vec2 p1 = bA->m_sweep.c + rA;
	b2Vec2 p2 = bB->m_sweep.c + rB;
	b2Vec2 d = p2 - p1;
	b2Vec2 axis = b2Mul(bA->m_xf.q, m_localXAxisA);

	b2Vec2 vA = bA->m_linearVelocity;
	b2Vec2 vB = bB->m_linearVelocity;
	double wA = bA->m_angularVelocity;
	double wB = bB->m_angularVelocity;

	double speed = b2Dot(d, b2Cross(wA, axis)) + b2Dot(axis, vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA));
	return speed;
}

double b2WheelJoint::GetRevoluteJointAngle() const
{
	b2Body* bA = m_bodyA;
	b2Body* bB = m_bodyB;
	return bB->m_sweep.a - bA->m_sweep.a;
}

double b2WheelJoint::GetRevoluteJointSpeed() const
{
	double wA = m_bodyA->m_angularVelocity;
	double wB = m_bodyB->m_angularVelocity;
	return wB - wA;
}

bool b2WheelJoint::IsMotorEnabled() const
{
	return m_enableMotor;
}

void b2WheelJoint::EnableMotor(bool flag)
{
	m_bodyA->SetAwakeResetSleepTime(true);
	m_bodyB->SetAwakeResetSleepTime(true);
	m_enableMotor = flag;
}

void b2WheelJoint::SetMotorSpeed(double speed)
{
	m_bodyA->SetAwakeResetSleepTime(true);
	m_bodyB->SetAwakeResetSleepTime(true);
	m_motorSpeed = speed;
}

void b2WheelJoint::SetMaxMotorTorque(double torque)
{
	m_bodyA->SetAwakeResetSleepTime(true);
	m_bodyB->SetAwakeResetSleepTime(true);
	m_maxMotorTorque = torque;
}

double b2WheelJoint::GetMotorTorque(double inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

void b2WheelJoint::Dump()
{
	int32_t indexA = m_bodyA->m_islandIndex;
	int32_t indexB = m_bodyB->m_islandIndex;

	b2Log("  b2WheelJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Log("  jd.localAxisA.Set(%.15lef, %.15lef);\n", m_localXAxisA.x, m_localXAxisA.y);
	b2Log("  jd.enableMotor = bool(%d);\n", m_enableMotor);
	b2Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
	b2Log("  jd.maxMotorTorque = %.15lef;\n", m_maxMotorTorque);
	b2Log("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
	b2Log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
	b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
