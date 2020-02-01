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
struct b2RopeStretch;
struct b2RopeBend;

enum b2BendingModel
{
    b2_springAngleBendingModel = 0,
    b2_pbdAngleBendingModel,
    b2_xpbdAngleBendingModel,
    b2_softAngleBendingModel,
    b2_pbdDistanceBendingModel,
    b2_pbdHeightBendingModel
};

///
struct b2RopeTuning
{
    b2RopeTuning()
    {
        bendingModel = b2_springAngleBendingModel;
        damping = 0.0;
        stretchStiffness = 1.0;
        bendStiffness = 0.5;
        bendHertz = 1.0;
        bendDamping = 0.0;
        isometric = false;
        fixedEffectiveMass = false;
        warmStart = false;
    }

    b2BendingModel bendingModel;
    double damping;
    double stretchStiffness;
    double bendStiffness;
    double bendHertz;
    double bendDamping;
    bool isometric;
    bool fixedEffectiveMass;
    bool warmStart;
};

///
struct b2RopeDef
{
    b2RopeDef()
    {
        position.SetZero();
        vertices = nullptr;
        count = 0;
        masses = nullptr;
        gravity.SetZero();
    }

    b2Vec2 position;
    b2Vec2* vertices;
    int32_t count;
    double* masses;
    b2Vec2 gravity;
    b2RopeTuning tuning;
};

///
class b2Rope
{
public:
    b2Rope();
    ~b2Rope();

    ///
    void Create(const b2RopeDef& def);

    ///
    void SetTuning(const b2RopeTuning& tuning);

    ///
    void Step(double timeStep, int32_t iterations, const b2Vec2& position);

    ///
    void Reset(const b2Vec2& position);

    ///
    void Draw(b2Draw* draw) const;

private:

    void SolveStretch();
    //void SolveBend_PBD_Distance();
    void SolveBend_PBD_Angle();
    void SolveBend_XPBD_Angle(double dt);
    void SolveBend_Soft_Angle(double dt);
    void SolveBend_PBD_Distance();
    void SolveBend_PBD_Height();
    void ApplyBendForces(double dt);

    b2Vec2 m_position;

    int32_t m_count;
    int32_t m_stretchCount;
    int32_t m_bendCount;

    b2RopeStretch* m_stretchConstraints;
    b2RopeBend* m_bendConstraints;

    b2Vec2* m_bindPositions;
    b2Vec2* m_ps;
    b2Vec2* m_p0s;
    b2Vec2* m_vs;

    double* m_invMasses;
    b2Vec2 m_gravity;

    b2RopeTuning m_tuning;
};

#endif
