/*
* Copyright (c) 2011 Erin Catto http://www.box2d.org
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

#ifndef B2_ROPE_H
#define B2_ROPE_H

#include <Box2D/Common/b2Math.h>

class b2Draw;

/// 
struct b2RopeDef
{
	b2RopeDef()
	{
        vertices = nullptr;
		count = 0;
        masses = nullptr;
		gravity.SetZero();
		damping = 0.1;
		k2 = 0.9;
		k3 = 0.1;
	}

	///
	b2Vec2* vertices;

	///
	int32_t count;

	///
	double* masses;

	///
	b2Vec2 gravity;

	///
	double damping;

	/// Stretching stiffness
	double k2;

	/// Bending stiffness. Values above 0.5 can make the simulation blow up.
	double k3;
};

/// 
class b2Rope
{
public:
	b2Rope();
	~b2Rope();

	///
	void Initialize(const b2RopeDef* def);

	///
	void Step(double timeStep, int32_t iterations);

	///
	int32_t GetVertexCount() const
	{
		return m_count;
	}

	///
	const b2Vec2* GetVertices() const
	{
		return m_ps;
	}

	///
	void Draw(b2Draw* draw) const;

	///
	void SetAngle(double angle);

private:

	void SolveC2();
	void SolveC3();

	int32_t m_count;
	b2Vec2* m_ps;
	b2Vec2* m_p0s;
	b2Vec2* m_vs;

	double* m_ims;

	double* m_Ls;
	double* m_as;

	b2Vec2 m_gravity;
	double m_damping;

	double m_k2;
	double m_k3;
};

#endif
