/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
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

#ifndef PYRAMID_STATIC_EDGES_H
#define PYRAMID_STATIC_EDGES_H

class PyramidStaticEdges : public Test
{
public:
	PyramidStaticEdges()
	{
		{
			double coords[] = 
			{
				50.0,0.0,
				-50.0,0.0
			};
			
			b2Vec2 verts[2];
			
			for (int32_t i = 0; i < 2; i++)
			{
				verts[i].Set(coords[i*2], coords[i*2 + 1]);
			}
			
			b2BodyDef bd;
			bd.position.Set( 0.0, 0.0 );
			b2Body* body = m_world->CreateBody(&bd);
			b2EdgeDef edgeDef;
			edgeDef.vertex1 = verts[0];
			edgeDef.vertex2 = verts[1];
			body->CreateFixture(&edgeDef);
			
			//body->SetMassFromShapes();
		}

		{
			b2PolygonDef sd;
			double a = 0.5;
			sd.SetAsBox(a, a);
			sd.density = 5.0;

			b2Vec2 x(-10.0, 1.0);
			b2Vec2 y;
			b2Vec2 deltaX(0.5625, 2.0);
			b2Vec2 deltaY(1.125, 0.0);

			const int32_t N = 2;

			for (int32_t i = 0; i < N; ++i)
			{
				y = x;

				for (int32_t j = i; j < N; ++j)
				{
					b2BodyDef bd;
					bd.position = y;
					b2Body* body = m_world->CreateBody(&bd);
					body->CreateFixture(&sd);
					body->SetMassFromShapes();

					y += deltaY;
				}

				x += deltaX;
			}
		}
	}

	static Test* Create()
	{
		return new PyramidStaticEdges;
	}
};

#endif
