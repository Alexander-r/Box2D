/*
* Copyright (c) 2008-2009 Erin Catto http://www.gphysics.com
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

#ifndef CAR_H
#define CAR_H

// Adapted from SpiritWalkers by darkzerox
class Car : public Test
{
public:
	Car()
	{
		{	// car body
			b2PolygonDef poly1, poly2;

			// bottom half
			poly1.vertexCount = 5;
			poly1.vertices[4].Set(-2.2,-0.74);
			poly1.vertices[3].Set(-2.2,0);
			poly1.vertices[2].Set(1.0,0);
			poly1.vertices[1].Set(2.2,-0.2);
			poly1.vertices[0].Set(2.2,-0.74);
			poly1.filter.groupIndex = -1;

			poly1.density		= 20.0;
			poly1.friction		= 0.68;
			poly1.filter.groupIndex	= -1;

			// top half
			poly2.vertexCount = 4;
			poly2.vertices[3].Set(-1.7,0);
			poly2.vertices[2].Set(-1.3,0.7);
			poly2.vertices[1].Set(0.5,0.74);
			poly2.vertices[0].Set(1.0,0);
			poly2.filter.groupIndex = -1;

			poly2.density		= 5.0;
			poly2.friction		= 0.68;
			poly2.filter.groupIndex	= -1;

			b2BodyDef bd;
			bd.position.Set(-35.0, 2.8);

			m_vehicle = m_world->CreateBody(&bd);
			m_vehicle->CreateFixture(&poly1);
			m_vehicle->CreateFixture(&poly2);
			m_vehicle->SetMassFromShapes();
		}

		{	// vehicle wheels
			b2CircleDef	circ;
			circ.density = 40.0;
			circ.radius = 0.38608;
			circ.friction = 0.8;
			circ.filter.groupIndex = -1;

			b2BodyDef bd;
			bd.allowSleep = false;
			bd.position.Set(-33.8, 2.0);

			m_rightWheel = m_world->CreateBody(&bd);
			m_rightWheel->CreateFixture(&circ);
			m_rightWheel->SetMassFromShapes();

			bd.position.Set(-36.2, 2.0);
			m_leftWheel = m_world->CreateBody(&bd);
			m_leftWheel->CreateFixture(&circ);
			m_leftWheel->SetMassFromShapes();
		}

		{	// join wheels to chassis
			b2Vec2 anchor;
			b2RevoluteJointDef jd;
			jd.Initialize(m_vehicle, m_leftWheel, m_leftWheel->GetWorldCenter());
			jd.collideConnected = false;
			jd.enableMotor = true;
			jd.maxMotorTorque = 10.0;
			jd.motorSpeed = 0.0;
			m_leftJoint = (b2RevoluteJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_vehicle, m_rightWheel, m_rightWheel->GetWorldCenter());
			jd.collideConnected = false;
			m_rightJoint = (b2RevoluteJoint*)m_world->CreateJoint(&jd);
		}

		{	// ground
			b2PolygonDef box;
			box.SetAsBox(19.5, 0.5);
			box.friction = 0.62;

			b2BodyDef bd;
			bd.position.Set(-25.0, 1.0);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateFixture(&box);
		}

		{	// more ground
			b2PolygonDef box;
			b2BodyDef bd;

			box.SetAsBox(9.5, 0.5, b2Vec2_zero, 0.1 * b2_pi);
			box.friction = 0.62;
			bd.position.Set(27.0 - 30.0, 3.1);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateFixture(&box);
		}

		{	// more ground
			b2PolygonDef box;
			b2BodyDef bd;

			box.SetAsBox(9.5, 0.5, b2Vec2_zero, -0.1 * b2_pi);
			box.friction = 0.62;
			bd.position.Set(55.0 - 30.0, 3.1);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateFixture(&box);
		}

		{	// more ground
			b2PolygonDef box;
			b2BodyDef bd;

			box.SetAsBox(9.5, 0.5, b2Vec2_zero, 0.03 * b2_pi);
			box.friction = 0.62;
			bd.position.Set(41.0, 2.0);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateFixture(&box);
		}

		{	// more ground
			b2PolygonDef box;
			b2BodyDef bd;

			box.SetAsBox(5.0, 0.5, b2Vec2_zero, 0.15 * b2_pi);
			box.friction = 0.62;
			bd.position.Set(50.0, 4.0);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateFixture(&box);
		}

		{	// more ground
			b2PolygonDef box;
			b2BodyDef bd;

			box.SetAsBox(20.0, 0.5);
			box.friction = 0.62;
			bd.position.Set(85.0, 2.0);

			b2Body* ground = m_world->CreateBody(&bd);
			ground->CreateFixture(&box);
		}
	}

	void Step(Settings* settings)
	{
		m_debugDraw.DrawString(5, m_textLine, "Keys: left = a, brake = s, right = d");
		m_textLine += 15;

		Test::Step(settings);
	}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'a':
			m_leftJoint->SetMaxMotorTorque(800.0);
			m_leftJoint->SetMotorSpeed(12.0);
			break;

		case 's':
			m_leftJoint->SetMaxMotorTorque(100.0);
			m_leftJoint->SetMotorSpeed(0.0);
			break;

		case 'd':
			m_leftJoint->SetMaxMotorTorque(1200.0);
			m_leftJoint->SetMotorSpeed(-36.0);
			break;
		}
	}

	static Test* Create()
	{
		return new Car;
	}

	b2Body* m_leftWheel;
	b2Body* m_rightWheel;
	b2Body* m_vehicle;
	b2RevoluteJoint* m_leftJoint;
	b2RevoluteJoint* m_rightJoint;
};

#endif
