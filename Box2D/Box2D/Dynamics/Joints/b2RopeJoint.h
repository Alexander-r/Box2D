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

#ifndef B2_ROPE_JOINT_H
#define B2_ROPE_JOINT_H

#include <Box2D/Dynamics/Joints/b2Joint.h>

/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collideConnected in b2JointDef.
struct b2RopeJointDef : public b2JointDef
{
	b2RopeJointDef()
	{
		type = e_ropeJoint;
		localAnchorA.Set(-1.0, 0.0);
		localAnchorB.Set(1.0, 0.0);
		maxLength = 0.0;
	}

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB;

	/// The maximum length of the rope.
	/// Warning: this must be larger than b2_linearSlop or
	/// the joint will have no effect.
	double maxLength;
};

/// A rope joint enforces a maximum distance between two points
/// on two bodies. It has no other effect.
/// Warning: if you attempt to change the maximum length during
/// the simulation you will get some non-physical behavior.
/// A model that would allow you to dynamically modify the length
/// would have some sponginess, so I chose not to implement it
/// that way. See b2DistanceJoint if you want to dynamically
/// control length.
class b2RopeJoint : public b2Joint
{
public:
    b2Vec2 GetAnchorA() const override;
    b2Vec2 GetAnchorB() const override;

    b2Vec2 GetReactionForce(double inv_dt) const override;
    double GetReactionTorque(double inv_dt) const override;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Set/Get the maximum length of the rope.
	void SetMaxLength(double length) { m_maxLength = length; }
	double GetMaxLength() const;

	b2LimitState GetLimitState() const;

	/// Dump joint to dmLog
    void Dump() override;

protected:

	friend class b2Joint;
	b2RopeJoint(const b2RopeJointDef* data);

    void InitVelocityConstraints(const b2SolverData& data) override;
    void SolveVelocityConstraints(const b2SolverData& data) override;
    bool SolvePositionConstraints(const b2SolverData& data) override;

	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	double m_maxLength;
	double m_length;
	double m_impulse;

	// Solver temp
	int32_t m_indexA;
	int32_t m_indexB;
	b2Vec2 m_u;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	double m_invMassA;
	double m_invMassB;
	double m_invIA;
	double m_invIB;
	double m_mass;
	b2LimitState m_state;
};

#endif
