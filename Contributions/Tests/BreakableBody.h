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

/* Testbed example showing deformable and breakable bodies using the soft
 * b2DistanceJoint and a small,liteweight triangle mesher.
 * 2008-05-09 / nimodo 
 */
#ifndef BREAKABLE_BODY_H
#define BREAKABLE_BODY_H

#include "TriangleMesh.h"

/// utility macro
#define H(x) (x)/2.0 
#define N_MAXVERTEX 256

class BreakableBody : public Test
{

public:  
   BreakableBody()
   {   
        /// geometries
        double  gx  = 100.0, gy =  1.0, 
                 dx  =  34.0, br =   0.3; 
        double  sx=-dx-H(dx), sy = 30.f;  
        /// break joint, if the reactionforce exceeds: 
        maxAllowableForce = 100.0;
        m_drawMode = m_staticBodies = false;
        m_drawCount = 0;
        /// ground
        { 
         b2PolygonDef sd;
         b2BodyDef    bd;
         b2Body*      ground;       
		 bd.position.Set(0.0, 0.0);
         ground = m_world->CreateBody(&bd);
         /// bottom
         sd.SetAsBox( H(gx), H(gy) );
         ground->CreateFixture(&sd);
         sd.SetAsBox( H(dx), H(gy), b2Vec2(-dx,sy-1.0), 0.0 );
         ground->CreateFixture(&sd);
        }  
        /// dyn bodies 
        { 
         b2PolygonDef       pd;
         b2DistanceJointDef dj;
         
         dj.dampingRatio     = 0.0;
         dj.collideConnected = true;

         ExampleData('B');
         dj.frequencyHz      = 20.f;
         pd.density          = 1.0/70.0;
         pd.friction         = 0.4;
         pd.restitution      = 0.01;
         CreateSoftBody( b2Vec2(sx,sy), 0,  0, pd, dj, 
                         nodes,n_nodes,  segments,n_segments,  holes,n_holes) ;

         ExampleData('@');
         dj.frequencyHz      = 20.f;
         pd.density          = 1.0/36.0;
         pd.friction         = 0.1;
         pd.restitution      = 0.5;
         CreateSoftBody( b2Vec2(sx+6.f,sy), 0, 0, pd, dj, 
                         nodes,n_nodes,  segments,n_segments,  holes,n_holes) ;
         
         ExampleData('x');                        
         dj.frequencyHz      = 20.0;            
         pd.density          = 1.0/60.0;
         pd.friction         = 0.6;
         pd.restitution      = 0.0;
         CreateSoftBody( b2Vec2(sx+13.f,sy),  0, 0,   pd, dj, 
                         nodes,n_nodes,  segments,n_segments,  holes,n_holes) ;

         ExampleData('2');
         pd.density          = 0.01;
         pd.friction         = 0.3;
         pd.restitution      = 0.3;
         CreateSoftBody( b2Vec2(sx+20.f,sy),  0, 0,   pd, dj, 
                         nodes,n_nodes,  segments,n_segments,  holes,n_holes) ;

         ExampleData('D');
         CreateSoftBody( b2Vec2(sx+28.f,sy),  0, 0,   pd, dj, 
                         nodes,n_nodes,  segments,n_segments,  holes,n_holes) ;
          
         ExampleData('b');
         dj.frequencyHz      = 10.0;
         dj.dampingRatio     = 20.0;
         pd.friction         = 0.9;
         pd.restitution      = 0.01;
         pd.density          = 0.01;
         CreateSoftBody( b2Vec2(-5.f,5.f*gy),  0, 0,   pd, dj, 
                         nodes,n_nodes,  segments,n_segments,  holes,n_holes) ;
         
         b2CircleDef cd;
         b2BodyDef   bd;
         b2Body*     b;
         cd.radius = br;
         cd.density= 0.001; 
         bd.position.Set(0.0,10.0*gy);
         for (int32_t i=0; i<60; i++ )
         {
           b = m_world->CreateBody(&bd);
           b->CreateFixture (&cd);
           b->SetMassFromShapes();
         }
         
        }
    }

    /// Create compound (soft) body using a triangle mesh
    /// If meshDensity is 0, a minimal grid is generated.
    /// Actually pd and dj define the behaviour for all triangles
    void CreateSoftBody(b2Vec2 pos, int32_t meshDensity,int32_t options,
                       b2PolygonDef pd, b2DistanceJointDef dj,
                       tmVertex* nodes,int32_t n_nodes,
                       tmSegmentId *segments=NULL, int32_t n_segments=0,
                       tmVertex* holes=NULL, int32_t n_holes=0)
    {
        int32_t   i;
        /// TriangleMesh defs
        tmTriangle *triangles;        
        TriangleMesh     md;
        /// box2d defs
        b2BodyDef          bd;
        b2Body             *b; 
        /// in case of meshDensit>3 ...
        md.SetMaxVertexCount(meshDensity); 
        if (options>0) md.SetOptions(options);
        /// triangulator main 
        md.Mesh( nodes, n_nodes,  segments,n_segments,  holes, n_holes );
        md.PrintData();        
        /// bodies (triangles) 
        triangles = md.GetTriangles(); 
        if ( triangles==NULL ) return;
        pd.vertexCount = 3;
        for ( i=0; i<md.GetTriangleCount(); i++ )
        {
          if ( triangles[i].inside ) 
          {
            /// triangle -> b2PolygonDef
            pd.vertices[0].Set(triangles[i].v[0]->x, triangles[i].v[0]->y);
            pd.vertices[1].Set(triangles[i].v[1]->x, triangles[i].v[1]->y);
            pd.vertices[2].Set(triangles[i].v[2]->x, triangles[i].v[2]->y);
            bd.position.Set(pos.x,pos.y);
            b = m_world->CreateBody(&bd);
            b->CreateFixture(&pd);
            b->SetMassFromShapes();
            /// we need the body pointer in the triangles for the joints later
            triangles[i].userData = (void *)b;
          }
        }   
        /// joints
        /// for each triangle-pair in edges, connect with a distance joint
        tmEdge        *edges;
        tmTriangle    *t0,*t1;        
        b2Body        *b1,*b2; 
        edges = md.GetEdges();
        for ( i=0; i<md.GetEdgeCount(); i++ )
        {
           t0 = edges[i].t[0];
           t1 = edges[i].t[1];
           if ( (t0->inside==false) || (t1->inside==false) ) continue;      
           
           /// Get bodies
           b1 = (b2Body*)t0->userData;
           b2 = (b2Body*)t1->userData;
           if ( b1==NULL || b2==NULL ) continue;

           dj.Initialize( b1,b2, b1->GetWorldCenter(), b2->GetWorldCenter());     
           m_world->CreateJoint(&dj);
        }
        /// clean TriangleMesh 
        md.FreeMemory();
    }
  
    /// maybe here to check for maximal reaction forces to break a body
    void Step(Settings* settings)
    {
      b2Joint *jStressed=NULL; 
      double F=0.0, tmp;

      Test::Step(settings);

      for (b2Joint* j = m_world->GetJointList(); j; j = j->GetNext())
	    {
          tmp = j->GetReactionForce(settings->hz).Length();
          if ( tmp>F ) 
          {
              F = tmp;
              jStressed = j;
          }
	    }       
      if ( jStressed && (F>maxAllowableForce) ) 
      {
          m_world->DestroyJoint(jStressed);
      }

      m_debugDraw.DrawString(1, m_textLine,"max.reactionforce=%.0 allowable=%.0  change:-+", (double)F,(double)maxAllowableForce);       
      m_textLine += 12;

      m_debugDraw.DrawString(1, m_textLine,"drawmode(%s):d  mesh:m  static(%s):s", (m_drawMode)?"on":"off", (m_staticBodies)?"on":"off");       
      m_textLine += 12;

      for ( int32_t i=0; i<m_drawCount-1; i++ )
      {
          b2Vec2 p1,p2;
          p1.Set(m_drawVertices[i].x,m_drawVertices[i].y);
          p2.Set(m_drawVertices[i+1].x,m_drawVertices[i+1].y);
          m_debugDraw.DrawSegment(p1,p2,b2Color(0.6,0.2,0.2));
      }
    }

    /// default constructor for TestEntries.cpp
    static Test* Create()
    {
      return new BreakableBody;
    }

    void Keyboard(unsigned char key)
    {
      switch (key)
      {
       case '-':
           maxAllowableForce -= 5.0;
         break;

       case '+':
           maxAllowableForce += 5.0;
         break;

       case 'd':
           m_drawMode = !m_drawMode; 
         break;

       case 's':
           m_staticBodies = !m_staticBodies; 
         break;

       case 'm':
           if ( m_drawCount>0 )
           {              
             b2PolygonDef       pd;
             b2DistanceJointDef dj;
             dj.collideConnected = true;
             dj.frequencyHz      = 20.f;
             dj.dampingRatio     = 10.0;
             pd.density          = (m_staticBodies) ? 0.0 : 1.0/32.0;
             pd.friction         = 0.99;
             pd.restitution      = 0.01;
             CreateSoftBody( b2Vec2(0.0,0.0),  0, tmO_SEGMENTBOUNDARY|tmO_GRADING,   
                             pd, dj, m_drawVertices, m_drawCount) ;
             
             m_drawCount = 0;
             m_drawMode = false; 
           }
         break;
      }
    }

    void MouseDown(const b2Vec2& p)
    {   
        if ( m_drawMode && (m_drawCount<N_MAXVERTEX) )
        {
            m_drawVertices[m_drawCount].x = p.x; 
            m_drawVertices[m_drawCount].y = p.y;
            m_drawCount++;
        }
        else Test::MouseDown(p);
    }
/*
    void MouseMove(const b2Vec2& p)
    {
	    m_lastPoint = p;
		if (m_drawMode)
        {		         
        }
    }
*/
    void MouseUp(const b2Vec2& p)
    {
        Test::MouseUp(p);
    }

    /// examples
    void ExampleData(char which)
    {
     /// @ - ring
     static tmVertex ring_nodes[] = {
       { 6.00, 3.00},
       { 5.12, 5.12},
       { 3.00, 6.00},
       { 0.88, 5.12},
       { 0.00, 3.00},
       { 0.88, 0.88},
       { 3.00, 0.00},
       { 5.12, 0.88},
       { 4.50, 3.00},
       { 4.06, 4.06},
       { 3.00, 4.50},
       { 1.94, 4.06},
       { 1.50, 3.00},
       { 1.94, 1.94},
       { 3.00, 1.50},
       { 4.06, 1.94}
     };
     static tmSegmentId ring_segments[] = {
       { 9, 10 },
       { 10, 11 },
       { 11, 12 },
       { 12, 13 },
       { 13, 14 },
       { 14, 15 },
       { 15, 16 },
       { 16, 9 }
     };
     static tmVertex ring_holes[] = {
       { 3.00, 3.00}
     };    
	 /// 'B'
     static tmVertex B_nodes[] = {
       { 0.00, 0.00},
       { 4.00, 0.00},
       { 5.00, 2.00},
       { 5.00, 4.00},
       { 4.00, 5.00},
       { 5.00, 6.00},
       { 5.00, 8.00},
       { 4.00, 9.00},
       { 0.00, 9.00},
       { 0.00, 5.00},
       { 1.50, 1.50},
       { 3.50, 1.50},
       { 3.50, 4.00},
       { 1.50, 4.00},
       { 1.50, 6.00},
       { 3.50, 6.00},
       { 3.50, 8.50},
       { 1.50, 8.50}
     };
     static tmSegmentId B_segments[] = {
       { 1, 2 },
       { 2, 3 },
       { 3, 4 },
       { 4, 5 },
       { 5, 6 },
       { 6, 7 },
       { 7, 8 },
       { 8, 9 },
       { 9, 10 },
       { 10, 1 },
       { 11, 12 },
       { 12, 13 },
       { 13, 14 },
       { 14, 11 },
       { 15, 16 },
       { 16, 17 },
       { 17, 18 },
       { 18, 15 }
     };
     static tmVertex B_holes[] = {
       { 5.00, 5.00},
       { 2.50, 2.50},
       { 2.50, 7.00}
     };    
	 /// 'D'
     static tmVertex D_nodes[] = {
       { 0.00, 0.00},
       { 4.00, 0.00},
       { 5.00, 2.50},
       { 5.00, 7.00},
       { 4.00, 9.00},
       { 0.00, 9.00},
       { 0.00, 5.00},
       { 1.50, 2.50},
       { 3.50, 2.50},
       { 3.50, 7.00},
       { 1.50, 7.00},
     };
     static tmSegmentId D_segments[] = {
       { 1, 2 },
       { 2, 3 },
       { 3, 4 },
       { 4, 5 },
       { 5, 6 },
       { 6, 7 },
       { 7, 1 },
       { 8, 9 },
       { 9, 10 },
       { 10, 11 },
       { 11, 8 },
     };
     static tmVertex D_holes[] = {
      { 2.50, 5.00},
     };	
     /// 'x' 
     static tmVertex x_nodes[] = {
      { 0.00, 0.00},
       { 1.00, 0.00},
       { 5.00, 0.00},
       { 6.00, 0.00},
       { 6.00, 1.00},
       { 6.00, 5.00},
       { 6.00, 6.00},
       { 1.00, 6.00},
       { 5.00, 6.00},
       { 0.00, 6.00},
       { 0.00, 5.00},
       { 0.00, 1.00},
       { 3.00, 2.00},
       { 4.00, 3.00},
       { 3.00, 4.00},
       { 2.00, 3.00}
     };
     static tmSegmentId x_segments[] = {
      { 2, 13 },
       { 3, 13 },
       { 5, 14 },
       { 6, 14 },
       { 8, 15 },
       { 9, 15 },
       { 11, 16 },
       { 12, 16 }
     };
     static tmVertex x_holes[] = {
       { 3.00, 1.00},
       { 5.00, 3.00},
       { 3.00, 5.00},
       { 1.00, 3.00},
     };
	 /// '2'
     static tmVertex two_nodes[] = {
       { 0.00, 0.00},
       { 6.00, 0.00},
       { 6.00, 1.00},
       { 2.00, 1.00},
       { 2.00, 2.00},
       { 6.00, 6.00},
       { 6.00, 8.00},
       { 5.00, 9.00},
       { 2.00, 9.00},
       { 1.00, 7.50},
       { 0.00, 2.50},
       { 5.00, 6.50},
       { 5.00, 8.00},
       { 2.50, 8.00},
       { 2.00, 7.50},
     };
     static tmSegmentId two_segments[] = {
       { 1, 2 },
       { 2, 3 },
       { 3, 4 },
       { 4, 5 },
       { 5, 6 },
       { 6, 7 },
       { 7, 8 },
       { 8, 9 },
       { 9, 10 },
       { 10, 15 },
       { 11, 12 },
       { 12, 13 },
       { 13, 14 },
       { 14, 15 },
     };
     static tmVertex two_holes[] = {
       { 3.00, 5.00},
       { 4.00, 3.00},
     };
     /// '-' beam
     static tmVertex beam_nodes[] = {
       { 0.00, 0.00},
       { 32.00, 0.00},
       { 32.00, 3.00},
       { 0.00, 3.00},
     }; 
     static tmSegmentId *beam_segments  = NULL;
     static tmVertex    *beam_holes  = NULL;
     /// 'b' a box
     static tmVertex b_nodes[] = {
       { 0.00, 0.00},
       { 10.00, 0.00},
       { 10.00, 10.00},
       { 0.00, 10.00},
       { 2.00, 2.00},
       { 8.00, 2.00},
       { 8.00, 8.00},
       { 2.00, 8.00},
     };
     static tmSegmentId b_segments[] = {
       { 5, 6 },
       { 6, 7 },
       { 7, 8 },
       { 8, 5 },
     };
     static tmVertex b_holes[] = {
       { 5.0, 5.0},
     };     
     /// choose...
     switch( which )
     {      
            case 'B':
                nodes      = B_nodes;
                segments   = B_segments;
                holes      = B_holes;
                
                n_nodes    = sizeof(B_nodes)/sizeof(tmVertex);
                n_segments = (segments) ? sizeof(B_segments)/sizeof(tmSegmentId) : 0;
                n_holes    = (holes) ? sizeof(B_holes)/sizeof(tmVertex) : 0;            
            break;

            case 'D':
                nodes      = D_nodes;
                segments   = D_segments;
                holes      = D_holes;
                
                n_nodes    = sizeof(D_nodes)/sizeof(tmVertex);
                n_segments = (segments) ? sizeof(D_segments)/sizeof(tmSegmentId) : 0;
                n_holes    = (holes) ? sizeof(D_holes)/sizeof(tmVertex) : 0;            
            break;

            case 'x':
                nodes      = x_nodes;
                segments   = x_segments;
                holes      = x_holes;
                
                n_nodes    = sizeof(x_nodes)/sizeof(tmVertex);
                n_segments = (segments) ? sizeof(x_segments)/sizeof(tmSegmentId) : 0;
                n_holes    = (holes) ? sizeof(x_holes)/sizeof(tmVertex) : 0;            
            break;

            case '@':
                nodes      = ring_nodes;
                segments   = ring_segments;
                holes      = ring_holes ;
                
                n_nodes    = sizeof(ring_nodes)/sizeof(tmVertex);
                n_segments = (segments) ? sizeof(ring_segments)/sizeof(tmSegmentId) : 0;
                n_holes    = (holes) ? sizeof(ring_holes)/sizeof(tmVertex) : 0;
            break;              

            case '2':
                nodes      = two_nodes;
                segments   = two_segments;
                holes      = two_holes ;
                
                n_nodes    = sizeof(two_nodes)/sizeof(tmVertex);
                n_segments = (segments) ? sizeof(two_segments)/sizeof(tmSegmentId) : 0;
                n_holes    = (holes) ? sizeof(two_holes)/sizeof(tmVertex) : 0;
            break;                  
            case '-':
                nodes      = beam_nodes;
                segments   = beam_segments;
                holes      = beam_holes ;
                
                n_nodes    = sizeof(beam_nodes)/sizeof(tmVertex);
                n_segments = (segments) ? sizeof(beam_segments)/sizeof(tmSegmentId) : 0;
                n_holes    = (holes) ? sizeof(beam_holes)/sizeof(tmVertex) : 0;
            break;                  
            case 'b':
                nodes      = b_nodes;
                segments   = b_segments;
                holes      = b_holes;
                
                n_nodes    = sizeof(b_nodes)/sizeof(tmVertex);
                n_segments = (segments) ? sizeof(b_segments)/sizeof(tmSegmentId) : 0;
                n_holes    = (holes) ? sizeof(b_holes)/sizeof(tmVertex) : 0;            
            break;
     }
    }
    ///
    bool        m_drawMode, m_staticBodies;
    tmVertex    m_drawVertices[N_MAXVERTEX];
    int32_t       m_drawCount;
    /// 
    double     maxAllowableForce;
    /// temporary vars to hold the examples
    tmVertex    *nodes;
    int32_t        n_nodes;
    tmVertex    *holes;
    int32_t        n_holes;
    tmSegmentId *segments;
    int32_t        n_segments;
};

#undef H

#endif
