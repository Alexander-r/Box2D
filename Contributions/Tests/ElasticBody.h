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

#ifndef ELASTIC_BODY_H
#define ELASTIC_BODY_H

class ElasticBody : public Test
{
public:
	b2Body* bodies[64];
    b2Body*  m_ground;
	b2Body*  m_elev;
	b2PrismaticJoint* m_joint_elev;
	/// Main...
	ElasticBody()
	{
		/// Bottom static body
		{ 
			b2PolygonDef sd;
			sd.SetAsBox(50.0, 2.0);
			sd.friction = 0.1;
			sd.restitution = 0.1;
			b2BodyDef bd;
			bd.position.Set(-1.0, -7.5);
			m_ground = m_world->CreateBody(&bd);
			m_ground->CreateFixture(&sd);
		}
		/// Upper static body
		{
            b2PolygonDef sd;
			sd.SetAsBox(20.0, 0.50,b2Vec2(0.f,0.f),0.047*b2_pi);
			sd.friction    = 0.01;
			sd.restitution = 0.001;  
			b2BodyDef bd;
			bd.position.Set(-20.f, 93.0);
			b2Body* g = m_world->CreateBody(&bd);
			g->CreateFixture(&sd);
			sd.SetAsBox(15.f, 0.50,b2Vec2(-15.0,12.5),0.0);
            g->CreateFixture(&sd);

            sd.SetAsBox(20.f,0.5,b2Vec2(0.0,-25.0),-0.5);
			g->CreateFixture(&sd);
        }
		/// Left channel left wall
		{
            b2PolygonDef sd;
			sd.SetAsBox(0.7, 55.0);
			sd.friction    = 0.1;
			sd.restitution = 0.1;  
			b2BodyDef bd;
			bd.position.Set(-49.3, 50.0);
			b2Body* g = m_world->CreateBody(&bd);
			g->CreateFixture(&sd);
        }
		/// Right wall
		{
            b2PolygonDef sd;
			sd.SetAsBox(0.7, 55.0);
			sd.friction    = 0.1;
			sd.restitution = 0.1;  
			b2BodyDef bd;
			bd.position.Set(45.f, 50.0);
			b2Body* g = m_world->CreateBody(&bd);
			g->CreateFixture(&sd);
        }
		/// Left channel right upper wall  
		{
            b2PolygonDef sd;
			sd.SetAsBox(0.5, 20.0);
			sd.friction    = 0.05;
			sd.restitution = 0.01;  
			b2BodyDef bd;
			bd.position.Set(-42.0, 70.0);
		    bd.angle = -0.03*b2_pi;
			b2Body* g = m_world->CreateBody(&bd);
			g->CreateFixture(&sd);
		}
		/// Left channel right lower wall
		{
            b2PolygonDef sd;
			sd.SetAsBox(0.50, 23.0);
			sd.friction    = 0.05;
			sd.restitution = 0.01;  
			b2BodyDef bd;
			bd.position.Set(-44.0, 27.0);
			b2Body* g = m_world->CreateBody(&bd);
			g->CreateFixture(&sd);
        /// Bottom motors
		    b2CircleDef cd;
			cd.radius   = 3.0;
			cd.density  = 15.0;
			cd.friction = 1.f;
			cd.restitution = 0.2;
        /// 1. 
			bd.position.Set(-40.0,2.5);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&cd);
            body->SetMassFromShapes(); 
            b2RevoluteJointDef jr;
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
            jr.maxMotorTorque = 30000.f;
            jr.enableMotor    = true; 
            jr.motorSpeed     = 20.f;
			m_world->CreateJoint(&jr);
        /// 1. left down
			bd.position.Set(-46.0,-2.5);
            cd. radius = 1.5;  jr.motorSpeed  = -20.f;
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&cd);
            sd.SetAsBox(2.0, 0.50);
            body->CreateFixture(&sd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter());
			m_world->CreateJoint(&jr);
        /// 2.
            cd.radius   = 3.0; jr.motorSpeed  = 20.f;
			bd.position.Set(-32.0,2.5);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&cd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
			m_world->CreateJoint(&jr);
        /// 3.
            jr.motorSpeed     = 20.f;
			bd.position.Set(-24.0,1.5);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&cd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
			m_world->CreateJoint(&jr);
        /// 4.
			bd.position.Set(-16.0,0.8);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&cd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
			m_world->CreateJoint(&jr);
        /// 5.
			bd.position.Set(-8.0,0.5);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&cd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
			m_world->CreateJoint(&jr);
        /// 6.
			bd.position.Set(0.0,0.1);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&cd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
			m_world->CreateJoint(&jr);
        /// 7.
			bd.position.Set(8.0,-0.5);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&cd);
			sd.SetAsBox(3.7, 0.5);
			body->CreateFixture(&sd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter()+b2Vec2(0.f,1.f));
			m_world->CreateJoint(&jr);
        /// 8. right rotator
            sd.SetAsBox(5.f, 0.5);
            sd.density = 2.0;
			bd.position.Set(18.0,1.f);
			b2Body* rightmotor = m_world->CreateBody(&bd);
			rightmotor->CreateFixture(&sd);
			sd.SetAsBox(4.5, 0.5, b2Vec2(0.f,0.f),b2_pi/3.f);
			rightmotor->CreateFixture(&sd);
			sd.SetAsBox(4.5, 0.5, b2Vec2(0.f,0.f),b2_pi*2.f/3.f);
			rightmotor->CreateFixture(&sd);
			cd.radius = 4.2;
			rightmotor->CreateFixture(&cd);
            rightmotor->SetMassFromShapes(); 
			jr.Initialize (g,rightmotor,rightmotor->GetWorldCenter());
            jr.maxMotorTorque = 70000.f;
            jr.motorSpeed     = -4.f;
            m_world->CreateJoint(&jr);
        /// 9. left rotator
            sd.SetAsBox(8.5, 0.5);
            sd.density = 2.0;
			bd.position.Set(-34.0,17.f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&sd);
			sd.SetAsBox(8.5, 0.5, b2Vec2(0.f,0.f),b2_pi*.5);
			body->CreateFixture(&sd);
			cd.radius = 7.f;
			cd.friction = 0.9;
			body->CreateFixture(&cd);
            body->SetMassFromShapes(); 
			jr.Initialize (g,body,body->GetWorldCenter());
            jr.maxMotorTorque = 100000.f;
            jr.motorSpeed     = -5.f;            
            m_world->CreateJoint(&jr);
        /// big compressor
            sd.SetAsBox(3.0,4.f);
            sd.density = 10.0;
			bd.position.Set(-16.0,17.f);
			b2Body *hammerleft = m_world->CreateBody(&bd);
			hammerleft->CreateFixture(&sd);
			hammerleft->SetMassFromShapes();
			b2DistanceJointDef jd;
			jd.Initialize(body, hammerleft, body->GetWorldCenter()+b2Vec2(0.f,6.f), hammerleft->GetWorldCenter() );
			m_world->CreateJoint(&jd);

			bd.position.Set(4.0,17.f);
			b2Body *hammerright = m_world->CreateBody(&bd);
			hammerright->CreateFixture(&sd);
			hammerright->SetMassFromShapes();
			jd.Initialize(body, hammerright, body->GetWorldCenter()-b2Vec2(0.f,6.f), hammerright->GetWorldCenter() );
			m_world->CreateJoint(&jd);
            /// pusher
            sd.SetAsBox(6.f,0.75);
			bd.position.Set(-21.0,9.f);
			b2Body* pusher = m_world->CreateBody(&bd);
			pusher->CreateFixture(&sd);
			sd.SetAsBox(2.f,1.5,b2Vec2(-5.f,0.f),0.f);
			pusher->SetMassFromShapes();
			pusher->CreateFixture(&sd);
			jd.Initialize(rightmotor,pusher,rightmotor->GetWorldCenter()+b2Vec2(-8.0,0.f),
				          pusher->GetWorldCenter()+b2Vec2(5.0,0.f) );
			m_world->CreateJoint(&jd);
        }
        /// Static bodies above motors
       {
            b2PolygonDef sd;
			b2CircleDef  cd;
			sd.SetAsBox(9.0, 0.5);
			sd.friction    = 0.05;
			sd.restitution = 0.01;  
			b2BodyDef bd;
			bd.position.Set(-15.5, 12.f);
            bd.angle = 0.0;
			b2Body* g = m_world->CreateBody(&bd);
			g->CreateFixture(&sd);
		    
			sd.SetAsBox(8.f, 0.5, b2Vec2(23.f,0.f),0.f);
			g->CreateFixture(&sd);
            /// compressor statics  
			sd.SetAsBox(7.0, 0.5, b2Vec2(-2.f,9.f),0.f);
			g->CreateFixture(&sd);
			sd.SetAsBox(9.0, 0.5, b2Vec2(22.f,9.f),0.f);
			g->CreateFixture(&sd);

			sd.SetAsBox(19.0, 0.5, b2Vec2(-9.f,15.f),-0.05);
			g->CreateFixture(&sd);
			sd.SetAsBox(4.7, 0.5, b2Vec2(15.f,11.5),-0.5);
			g->CreateFixture(&sd);
            /// below compressor
			sd.SetAsBox(26.0, 0.3, b2Vec2(17.f,-4.4),-0.02);
			g->CreateFixture(&sd);
			cd.radius   = 1.0;	cd.friction = 1.0;
			cd.localPosition = b2Vec2(29.f,-6.f);
            g->CreateFixture(&cd); 
            cd.radius   = 0.7;
			cd.localPosition = b2Vec2(-2.f,-4.5);
            g->CreateFixture(&cd);
        }
        /// Elevator
        {
            b2BodyDef  bd;	
            b2CircleDef cd;
            b2PolygonDef sd;

			bd.position.Set(40.0,4.0);
			m_elev = m_world->CreateBody(&bd);

			sd.SetAsBox(0.5, 2.5,b2Vec2(3.0,-3.0), 0.f);
			sd.density     = 1.f;
			sd.friction    = 0.01;
			m_elev->CreateFixture(&sd);
			sd.SetAsBox(7.0, 0.5, b2Vec2(-3.5,-5.5), 0.f);
			m_elev->CreateFixture(&sd);
			sd.SetAsBox(0.5, 2.5, b2Vec2(-11.f,-3.5), 0.f);
			m_elev->CreateFixture(&sd);
            m_elev->SetMassFromShapes();           
		    
			b2PrismaticJointDef jp;
			jp.Initialize(m_ground,m_elev, bd.position, b2Vec2(0.0, 1.0));
			jp.lowerTranslation =  0.0;
			jp.upperTranslation = 100.0;
			jp.enableLimit = true;		
			jp.enableMotor = true;
			jp.maxMotorForce = 10000.f;
			jp.motorSpeed    = 0.f; 
			m_joint_elev = (b2PrismaticJoint*)m_world->CreateJoint(&jp);			  
            
			/// Korb
            sd.SetAsBox(2.3, 0.5,b2Vec2(1.f,0.0), 0.0);
            sd.density = 0.5;
            bd.position.Set(29.0,6.5);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&sd);
            sd.SetAsBox(2.5, 0.5,b2Vec2(3.0,-2.f), b2_pi/2.f);
            body->CreateFixture(&sd);
            sd.SetAsBox(4.6, 0.5,b2Vec2(7.8,-4.0), 0.f);
            body->CreateFixture(&sd);
            sd.SetAsBox(0.5, 4.5,b2Vec2(12.f,0.0), 0.f);
            body->CreateFixture(&sd);
            
			sd.SetAsBox(0.5, 0.5,b2Vec2(13.f,4.0), 0.f);
            body->CreateFixture(&sd);

            cd.radius   = 0.7; cd.density  = 1.f; cd.friction = 0.01;
            cd.localPosition = b2Vec2(0.f,0.f);
            body->CreateFixture(&cd);
            body->SetMassFromShapes();  

            b2RevoluteJointDef jr;
            jr.Initialize(m_elev,body, bd.position);
            jr.enableLimit = true;
            jr.lowerAngle  = -0.2;
            jr.upperAngle  = b2_pi*1.1;
            jr.collideConnected = true;
            m_world->CreateJoint(&jr);
            /// upper body exit
            sd.SetAsBox(14.0, 0.5,b2Vec2(-3.5,-10.0), 0.0);
            bd.position.Set(17.5,96.0);
            body = m_world->CreateBody(&bd);
            body->CreateFixture(&sd);
		}
		/// "Elastic body" 64 bodies - something like a lin. elastic compound
		/// connected via dynamic forces (springs) 
		{
			b2PolygonDef sd;
			sd.SetAsBox(0.55, 0.55);
			sd.density    = 1.5;
			sd.friction   = 0.01;
			sd.filter.groupIndex = -1;
			b2Vec2       startpoint(30.f,20.f);
			b2BodyDef    bd;
			bd.isBullet   = false;
  	 	    bd.allowSleep = false;	
			for (int i = 0; i < 8; ++i) 
            {
				for (int j = 0; j < 8; ++j) 
                {
					bd.position.Set(j*1.02, 2.51 + 1.02 * i);
					bd.position  += startpoint;
					b2Body* body  = m_world->CreateBody(&bd);
					bodies[8*i+j] = body;
					body->CreateFixture(&sd);
					body->SetMassFromShapes();
				}
			}
		}
	}
	///  Apply dynamic forces (springs) and check elevator state
	void Step(Settings* settings)
	{
		Test::Step(settings);
		for (int i=0; i<8; ++i){
			for (int j=0; j<8; ++j){
				b2Vec2 zero(0.0,0.0);
				b2Vec2 down(0.0, -0.5);
				b2Vec2 up(0.0, 0.5);
				b2Vec2 right(0.5, 0.0);
				b2Vec2 left(-0.5, 0.0);
				int ind = i*8+j;
				int indr = ind+1;
				int indd = ind+8;
				double spring = 500.0;
				double damp = 5.0;
				if (j<7) {
					AddSpringForce(*(bodies[ind]),zero,*(bodies[indr]),zero,spring, damp, 1.0);
					AddSpringForce(*(bodies[ind]),right,*(bodies[indr]),left,0.5*spring, damp, 0.0);
				}
				if (i<7) {
					AddSpringForce(*(bodies[ind]),zero,*(bodies[indd]),zero,spring, damp, 1.0);
					AddSpringForce(*(bodies[ind]),up,*(bodies[indd]),down,0.5*spring,damp,0.0);
				}
				int inddr = indd + 1;
				int inddl = indd - 1;
				double drdist = sqrtf(2.0);
				if (i < 7 && j < 7){
					AddSpringForce(*(bodies[ind]),zero,*(bodies[inddr]),zero,spring, damp, drdist);
				}
				if (i < 7 && j > 0){
					AddSpringForce(*(bodies[ind]),zero,*(bodies[inddl]),zero,spring, damp, drdist);
				}

				indr = ind+2;
				indd = ind+8*2;
				if (j<6) {
					AddSpringForce(*(bodies[ind]),zero,*(bodies[indr]),zero,spring, damp, 2.0);
				}
				if (i<6) {
					AddSpringForce(*(bodies[ind]),zero,*(bodies[indd]),zero,spring,damp,2.0);
				}

				inddr = indd + 2;
				inddl = indd - 2;
				drdist = sqrtf(2.0)*2.0;
				if (i < 6 && j < 6){
					AddSpringForce(*(bodies[ind]),zero,*(bodies[inddr]),zero,spring, damp, drdist);
				}
				if (i < 6 && j > 1){
					AddSpringForce(*(bodies[ind]),zero,*(bodies[inddl]),zero,spring, damp, drdist);
				}
			}
		}
		/// Check if bodies are near elevator
		///  Look if the body to lift is near the elevator
		b2Vec2 p1 = bodies[0]->GetWorldCenter(); 
        b2Vec2 p2 = bodies[63]->GetWorldCenter(); 
		///    m_elev:   elevator prism. joint
		b2Vec2 e = m_elev->GetWorldCenter() + b2Vec2(0.f,7.f);  
		// maybe not the best way to do it...
		// Bodies reached the elevator side 
		if ( p1.x>e.x || p2.x>e.x )	{
  		    // go up
			if ( ( p1.y<e.y || p2.y<e.y ) &&
				 ( m_joint_elev->GetJointTranslation()<=m_joint_elev->GetLowerLimit()+1.f ) ) 
			{
				m_joint_elev->SetMotorSpeed(20.f);
                //printf("lift goes up trans: %G\n",m_joint_elev->GetJointTranslation());
			}
		}
		// go down
		if ( (m_joint_elev->GetJointTranslation()>=m_joint_elev->GetUpperLimit()-2.f) ) 
		{
               m_joint_elev->SetMotorSpeed(-15.f);
               //printf("lift goes down: %G\n",m_joint_elev->GetJointTranslation());
		}
	}
   /// Add a spring force
   void AddSpringForce(b2Body& bA, b2Vec2& localA, b2Body& bB, b2Vec2& localB, double k, double friction, double desiredDist)
   {
        b2Vec2 pA = bA.GetWorldPoint(localA);
        b2Vec2 pB = bB.GetWorldPoint(localB);
        b2Vec2 diff = pB - pA;
        //Find velocities of attach points
        b2Vec2 vA = bA.GetLinearVelocity() - b2Cross(bA.GetWorldVector(localA), bA.GetAngularVelocity());
        b2Vec2 vB = bB.GetLinearVelocity() - b2Cross(bB.GetWorldVector(localB), bB.GetAngularVelocity());
        b2Vec2 vdiff = vB-vA;
        double dx = diff.Normalize(); //normalizes diff and puts length into dx
        double vrel = vdiff.x*diff.x + vdiff.y*diff.y;
        double forceMag = -k*(dx-desiredDist) - friction*vrel;
        diff *= forceMag; // diff *= forceMag
        bB.ApplyForce(diff, bA.GetWorldPoint(localA));
        diff *= -1.0;
        bA.ApplyForce(diff, bB.GetWorldPoint(localB));
    }
    /// Default constructor
	static Test* Create()
	{
		return new ElasticBody;
	}
};

#endif
