/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#ifndef B2_PULLEY_JOINT_H
#define B2_PULLEY_JOINT_H

#include <Box2D/Dynamics/Joints/b2Joint.h>

const double b2_minPulleyLength = 2.0;

/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, and a pulley ratio.
struct b2PulleyJointDef : public b2JointDef
{
	b2PulleyJointDef()
	{
		type = e_pulleyJoint;
		groundAnchorA.Set(-1.0, 1.0);
		groundAnchorB.Set(1.0, 1.0);
		localAnchorA.Set(-1.0, 0.0);
		localAnchorB.Set(1.0, 0.0);
		lengthA = 0.0;
		lengthB = 0.0;
		ratio = 1.0;
		collideConnected = true;
	}

	/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
	void Initialize(b2Body* bodyA, b2Body* bodyB,
					const b2Vec2& groundAnchorA, const b2Vec2& groundAnchorB,
					const b2Vec2& anchorA, const b2Vec2& anchorB,
					double ratio);

	/// The first ground anchor in world coordinates. This point never moves.
	b2Vec2 groundAnchorA;

	/// The second ground anchor in world coordinates. This point never moves.
	b2Vec2 groundAnchorB;

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB;

	/// The a reference length for the segment attached to bodyA.
	double lengthA;

	/// The a reference length for the segment attached to bodyB.
	double lengthB;

	/// The pulley ratio, used to simulate a block-and-tackle.
	double ratio;
};

/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// Warning: the pulley joint can get a bit squirrelly by itself. They often
/// work better when combined with prismatic joints. You should also cover the
/// the anchor points with static shapes to prevent one side from going to
/// zero length.
class b2PulleyJoint : public b2Joint
{
public:
    b2Vec2 GetAnchorA() const override;
    b2Vec2 GetAnchorB() const override;

    b2Vec2 GetReactionForce(double inv_dt) const override;
    double GetReactionTorque(double inv_dt) const override;

	/// Get the first ground anchor.
	b2Vec2 GetGroundAnchorA() const;

	/// Get the second ground anchor.
	b2Vec2 GetGroundAnchorB() const;

	/// Get the current length of the segment attached to bodyA.
	double GetLengthA() const;

	/// Get the current length of the segment attached to bodyB.
	double GetLengthB() const;

	/// Get the pulley ratio.
	double GetRatio() const;

	/// Get the current length of the segment attached to bodyA.
	double GetCurrentLengthA() const;

	/// Get the current length of the segment attached to bodyB.
	double GetCurrentLengthB() const;

	/// Dump joint to dmLog
    void Dump() override;

	/// Implement b2Joint::ShiftOrigin
    void ShiftOrigin(const b2Vec2& newOrigin) override;

protected:

	friend class b2Joint;
	b2PulleyJoint(const b2PulleyJointDef* data);

    void InitVelocityConstraints(const b2SolverData& data) override;
    void SolveVelocityConstraints(const b2SolverData& data) override;
    bool SolvePositionConstraints(const b2SolverData& data) override;

	b2Vec2 m_groundAnchorA;
	b2Vec2 m_groundAnchorB;
	double m_lengthA;
	double m_lengthB;
	
	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	double m_constant;
	double m_ratio;
	double m_impulse;

	// Solver temp
	int32_t m_indexA;
	int32_t m_indexB;
	b2Vec2 m_uA;
	b2Vec2 m_uB;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	double m_invMassA;
	double m_invMassB;
	double m_invIA;
	double m_invIB;
	double m_mass;
};

#endif
