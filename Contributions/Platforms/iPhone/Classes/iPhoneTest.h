/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* iPhone port by Simon Oliver - http://www.simonoliver.com - http://www.handcircus.com
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



#ifndef TEST_H
#define TEST_H

#import <UIKit/UIKit.h>
#include <Box2D/Box2D.h>
#include "GLES-Render.h"

#include <cstdlib>

class Test;
struct Settings;

typedef Test* TestCreateFcn();

#define	RAND_LIMIT	32767

/// Random number in range [-1,1]
inline double RandomFloat()
{
	double r = (double)(rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = 2.0 * r - 1.0;
	return r;
}

/// Random doubleing point number in range [lo, hi]
inline double RandomFloat(double lo, double hi)
{
	double r = (double)(rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = (hi - lo) * r + lo;
	return r;
}

/// Test settings. Some can be controlled in the GUI.
struct Settings
{
	Settings() :
		viewCenter(0.0, 20.0),
		hz(60.0),
		velocityIterations(8),
		positionIterations(3),
		drawShapes(1),
		drawJoints(1),
		drawAABBs(0),
		drawPairs(0),
		drawContactPoints(0),
		drawContactNormals(0),
		drawContactForces(0),
		drawFrictionForces(0),
		drawCOMs(0),
		drawStats(0),
		enableWarmStarting(1),
		enableContinuous(1),
		enableSubStepping(0),
		pause(0),
		singleStep(0)
		{}

	b2Vec2 viewCenter;
	double hz;
	int32_t velocityIterations;
	int32_t positionIterations;
	int32_t drawShapes;
	int32_t drawJoints;
	int32_t drawAABBs;
	int32_t drawPairs;
	int32_t drawContactPoints;
	int32_t drawContactNormals;
	int32_t drawContactForces;
	int32_t drawFrictionForces;
	int32_t drawCOMs;
	int32_t drawStats;
	int32_t enableWarmStarting;
	int32_t enableContinuous;
	int32_t enableSubStepping;
	int32_t pause;
	int32_t singleStep;
};

struct TestEntry
{
	const char *name;
	TestCreateFcn *createFcn;
};

extern TestEntry g_testEntries[];
// This is called when a joint in the world is implicitly destroyed
// because an attached body is destroyed. This gives us a chance to
// nullify the mouse joint.
class DestructionListener : public b2DestructionListener
	{
	public:
		void SayGoodbye(b2Fixture* fixture) { B2_NOT_USED(fixture); }
		void SayGoodbye(b2Joint* joint);
		
		Test* test;
	};

const int32_t k_maxContactPoints = 2048;

struct ContactPoint
{
	b2Fixture* fixtureA;
	b2Fixture* fixtureB;
	b2Vec2 normal;
	b2Vec2 position;
	b2PointState state;
};

class Test : public b2ContactListener
	{
	public:
		
		Test();
		virtual ~Test();
		
		void SetGravity(double x,double y);
		void SetTextLine(int32_t line) { m_textLine = line; }
		void DrawTitle(int x, int y, const char *string);
		virtual void Step(Settings* settings);
		virtual void Keyboard(unsigned char key) { B2_NOT_USED(key); }
		void ShiftMouseDown(const b2Vec2& p);
		virtual void MouseDown(const b2Vec2& p);
		virtual void MouseUp(const b2Vec2& p);
		void MouseMove(const b2Vec2& p);
		void LaunchBomb();
		void LaunchBomb(const b2Vec2& position, const b2Vec2& velocity);
		
		void SpawnBomb(const b2Vec2& worldPt);
		void CompleteBombSpawn(const b2Vec2& p);
		
		// Let derived tests know that a joint was destroyed.
		virtual void JointDestroyed(b2Joint* joint) { B2_NOT_USED(joint); }
		
		// Callbacks for derived classes.
		virtual void BeginContact(b2Contact* contact) { B2_NOT_USED(contact); }
		virtual void EndContact(b2Contact* contact) { B2_NOT_USED(contact); }
		virtual void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);
		virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
		{
			B2_NOT_USED(contact);
			B2_NOT_USED(impulse);
		}
		
	protected:
		friend class DestructionListener;
		friend class BoundaryListener;
		friend class ContactListener;
		
		b2Body* m_groundBody;
		b2AABB m_worldAABB;
		ContactPoint m_points[k_maxContactPoints];
		int32_t m_pointCount;
		DestructionListener m_destructionListener;
		GLESDebugDraw m_debugDraw;
		int32_t m_textLine;
		b2World* m_world;
		b2Body* m_bomb;
		b2MouseJoint* m_mouseJoint;
		b2Vec2 m_bombSpawnPoint;
		bool m_bombSpawning;
		b2Vec2 m_mouseWorld;
		int32_t m_stepCount;
	};

#endif
