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

#ifndef STATIC_EDGES_H
#define STATIC_EDGES_H

class StaticEdges : public Test
{
public:
	StaticEdges()
	{
#if 0
		{
			b2CircleDef sd;
			sd.radius = 0.5;
			sd.localPosition.SetZero();
			sd.density = 2.0;

			b2BodyDef bd;
			bd.position.Set(0.0, 2.0);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&sd);
			body->SetMassFromShapes();
		}
#endif

		{
			b2CircleDef sd1;
			sd1.radius = 0.5;
			sd1.localPosition.Set(-0.5, 0.5);
			sd1.density = 2.0;

			b2CircleDef sd2;
			sd2.radius = 0.5;
			sd2.localPosition.Set(0.5, 0.5);
			sd2.density = 0.0; // massless

			for (int i = 0; i < 10; ++i)
			{
				double x = RandomFloat(-0.1, 0.1);
				b2BodyDef bd;
				bd.position.Set(x + 5.0, 1.05 + 2.5 * i);
				bd.angle = RandomFloat(-b2_pi, b2_pi);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&sd1);
				body->CreateFixture(&sd2);
				body->SetMassFromShapes();
			}
		}

		{
			b2PolygonDef sd1;
			sd1.SetAsBox(0.25, 0.5);
			sd1.density = 2.0;

			b2PolygonDef sd2;
			sd2.SetAsBox(0.25, 0.5, b2Vec2(0.0, -0.5), 0.5 * b2_pi);
			sd2.density = 2.0;

			for (int i = 0; i < 10; ++i)
			{
				double x = RandomFloat(-0.1, 0.1);
				b2BodyDef bd;
				bd.position.Set(x - 5.0, 1.05 + 2.5 * i);
				bd.angle = RandomFloat(-b2_pi, b2_pi);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&sd1);
				body->CreateFixture(&sd2);
				body->SetMassFromShapes();
			}
		}

		{
			b2XForm xf1;
			xf1.R.Set(0.3524 * b2_pi);
			xf1.position = b2Mul(xf1.R, b2Vec2(1.0, 0.0));

			b2PolygonDef sd1;
			sd1.vertexCount = 3;
			sd1.vertices[0] = b2Mul(xf1, b2Vec2(-1.0, 0.0));
			sd1.vertices[1] = b2Mul(xf1, b2Vec2(1.0, 0.0));
			sd1.vertices[2] = b2Mul(xf1, b2Vec2(0.0, 0.5));
			sd1.density = 2.0;

			b2XForm xf2;
			xf2.R.Set(-0.3524 * b2_pi);
			xf2.position = b2Mul(xf2.R, b2Vec2(-1.0, 0.0));

			b2PolygonDef sd2;
			sd2.vertexCount = 3;
			sd2.vertices[0] = b2Mul(xf2, b2Vec2(-1.0, 0.0));
			sd2.vertices[1] = b2Mul(xf2, b2Vec2(1.0, 0.0));
			sd2.vertices[2] = b2Mul(xf2, b2Vec2(0.0, 0.5));
			sd2.density = 2.0;

			for (int32_t i = 0; i < 10; ++i)
			{
				double x = RandomFloat(-0.1, 0.1);
				b2BodyDef bd;
				bd.position.Set(x, 2.05 + 2.5 * i);
				bd.angle = 0.0;
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&sd1);
				body->CreateFixture(&sd2);
				body->SetMassFromShapes();
			}
		}

		{
			double loop1[] = 
			{
                                0.063134534,8.3695248,
                                0.94701801,9.3165428,
				0.0,9.0640047,
				-0.12626907,10.326695,
                                1.4520943,11.77879,
				2.2728432,10.137292,
				2.3991123,11.147444,
				3.5986685,10.958041,
				3.9143411,7.3593722,
                                4.1668793,9.4428119,
                                5.4295699,9.3165428,
                                6.2503189,8.3063903,
				6.6922606,10.137292,
				4.9876282,9.8216191,
				4.7350901,10.958041,
				7.2604714,11.652521,
				10.732871,11.147444,
                                10.480333,10.642368,
				10.732871,9.8216191,
                                11.55362,9.4428119,
                                12.374369,9.3796773,
				13.005714,9.8216191,
                                13.195118,10.38983,
				13.005714,10.768637,
				12.626907,10.894906,
				12.753176,11.526252,
				13.573925,11.715655,
				14.836616,11.399982,
				16.351844,10.768637,
				17.867073,11.399982,
                                17.803939,10.263561,
				17.361997,8.3063903,
                                17.803939,8.1801212,
				18.056477,9.5059464,
                                18.182746,11.336848,
                                18.561553,11.210579,
				18.561553,9.6322155,
				18.561553,7.7381795,
                                18.687822,5.5284708,
                                19.382302,5.6547398,
                                19.066629,8.1801212,
				19.003495,10.263561,
                                19.066629,11.463117,
                                19.887378,11.841924,
				20.708127,11.273713,
                                21.0238,10.011023,
				20.708127,7.2962377,
				21.086934,6.2860852,
                                21.150069,3.7607038,
				20.392455,2.5611476,
                                18.624688,2.5611476,
                                20.771262,2.1192059,
                                20.771262,0.22516988,
                                18.624688,-0.2799064,
				13.826463,0.16203534,
				14.015867,1.7403987,
                                13.195118,2.1823404,
				12.626907,1.5509951,
				12.879445,0.85651522,
				12.626907,0.35143895,
				10.543467,1.298457,
				11.490485,3.9501074,
                                13.889598,3.6344347,
                                13.889598,2.9399549,
                                14.584077,3.8869729,
				11.932427,5.2127981,
                                9.7227183,4.0132419,
				10.796005,3.5081657,
                                9.7858528,3.2556275,
				10.796005,2.4980131,
				7.9549513,1.7403987,
				9.6595837,1.424726,
				9.217642,0.66711162,
				8.270624,-0.090502792,
				7.0079333,0.85651522,
                                6.1240498,-0.15363733,
                                6.1240498,3.192493,
				5.6821081,2.4348786,
                                4.9876282,2.1192059,
                                4.1037447,1.8666678,
                                3.0304576,1.8666678,
				2.0834396,2.245475,
                                1.6414979,2.6242822,
				1.3258252,3.5081657,
				1.2626907,0.47770802,
				0.63134534,0.035766276,
                                0.063134534,0.98278429
			};
			
			double loop2[] = 
			{
				8.270624,6.1598161,
				8.270624,5.3390672,
                                8.7757003,5.086529,
                                9.4701801,5.5284708,
				9.217642,6.033547,
				8.7757003,6.4123542
			};
			
			double loop3[] = 
			{
				-5.0, 10.0,
				5.0, 10.0,
				5.0, 0.0,
				-5.0, 0.0,
			};

			b2Vec2 pointLoop1[87];
			b2Vec2 pointLoop2[6];
			b2Vec2 pointLoop3[4];
			
			for (int32_t i = 0; i < 87; i++)
			{
				pointLoop1[i].Set(loop1[i*2] - 10.0, loop1[i*2 + 1]);
			}
			
			for (int32_t i = 0; i < 6; i++)
			{
				pointLoop2[i].Set(loop2[i*2] - 10.0, loop2[i*2 + 1]);
			}
			
			for (int32_t i = 0; i < 4; i++)
			{
				pointLoop3[i].Set(loop3[i*2], loop3[i*2 + 1]);
			}

			b2BodyDef bd;
			bd.position.Set( 0.0, 0.0 );
			b2Body* body = m_world->CreateBody(&bd);
			
			b2EdgeChainDef edgeDef;
			edgeDef.vertexCount = 87;
			edgeDef.vertices = pointLoop1;
			b2CreateEdgeChain(body, &edgeDef);
			
			edgeDef.vertexCount = 6;
			edgeDef.vertices = pointLoop2;
			b2CreateEdgeChain(body, &edgeDef);

			//edgeDef.vertexCount = 4;
			//edgeDef.vertices = pointLoop3;
			//b2CreateEdgeChain(body, &edgeDef);
		}
	}

	static Test* Create()
	{
		return new StaticEdges;
	}
};

#endif
