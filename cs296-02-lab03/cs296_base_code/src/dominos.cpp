/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */


#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs296
{
  /**  This is the constructor 
   * This is the documentation block for the constructor.
   */ 
/*! \par
*b2BodyDef bd defines the body. <br>
*m_world is the physics world creates in Box2D.<br>
*CreateFixture is used to fix a body to a parent body and hence many of its properties become local to the parent body.<br>
*CreateBody creates a new body in the m_world.<br>
*/
  dominos_t::dominos_t()
  {
		
    b2Body* b1; //! b1 is a pointer to the ground which is a Box2D body. 

	/*! \par Block 1: Ground
	 * A Box2D Edge shape used to represent ground. <br>
	 * Edge is set between two points on x-axis x1=-90 and x2=90. <br>
	 * bd is a Box2D body definition.<br>
	 * b1 is then pointed to a body created with body definition bd<br>
	 * A fixture is created on the body with Edge shape created earlier and 0 density
	 */
    {
      b2EdgeShape shape;     
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
          
    //~ {
	//~ /*! \par Block 2: Top Horizontal Shelf
	 //~ * Polygon is used to represent shelf. <br>
	 //~ * Height and width of Polygon are set as 0.5 units and 12 units respectively. <br>
	 //~ * A body bd is defined and position of center of body is set in x-y coordinates by bd.position.set . <br>
	 //~ * Polygon is bound to body to create the shelf using CreateFixture command. <br>         
	 //~ */
      //~ b2PolygonShape shape; 
      //~ shape.SetAsBox(6.0f, 0.25f); 
	//~ 
      //~ b2BodyDef bd;
      //~ bd.position.Set(-31.0f, 30.0f);  
      //~ b2Body* ground = m_world->CreateBody(&bd);
      //~ ground->CreateFixture(&shape, 0.0f); 
    //~ }
//~ 
    //~ 
    //~ {
	//~ /*! \par Block 3: Dominos 
	 //~ * Shape of dominos is set to be of that of polygon <br>
	 //~ * Height and width of Polygon are set as 0.1 units and 1.0 units respectively by SetAsBox. <br>
	 //~ * 10 dominos are defined with the help of a for loop and each dominos' x-y co-ordinates are set in for loop. <br>
	 //~ * The density of each dominos is set to 20 units by fd.density ad friction is set to 0.1 units by fd.friction. <br>       
	 //~ * A fixture definition is created by b2FixtureDef fd. 
	 //~ */
      //~ b2PolygonShape shape;
      //~ shape.SetAsBox(0.1f, 1.0f);
	//~ 
      //~ b2FixtureDef fd;
      //~ fd.shape = &shape;
      //~ fd.density = 20.0f;
      //~ fd.friction = 0.1f;
	//~ 
	//~ /*! \par
	//~ * 10 bodies are defined named bd. <br>
	//~ * All the bodies' type is set to dynamicBody. <br>
	//~ * Each dominos is bound to the body.
	//~ */
	//~ 
      //~ for (int i = 0; i < 10; ++i)
	//~ {
	  //~ b2BodyDef bd;
	  //~ bd.type = b2_dynamicBody;
	  //~ bd.position.Set(-35.5f + 1.0f * i, 31.25f);
	  //~ b2Body* body = m_world->CreateBody(&bd);
	  //~ body->CreateFixture(&fd);
	//~ }
    //~ }
      //~ 
    //~ 
    //~ {
	//~ /*! \par Block 4:Another Horizontal Shelf
	//~ * The shape of the shelp is set to a polygon by b2PolygonShape shape. <br>
	//~ * Its position is set to x=1.0 unit and y=6.0 unit.<br>
	//~ * It is fixed to ground using CreateFixture method.
	//~ */
      //~ b2PolygonShape shape;
      //~ shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);
	//~ 
      //~ b2BodyDef bd;
      //~ bd.position.Set(1.0f, 6.0f);
      //~ b2Body* ground = m_world->CreateBody(&bd);
      //~ ground->CreateFixture(&shape, 0.0f);
    //~ }
//~ 
//~ 
    //~ 
    //~ {
	//~ /*! \par Block 5:The pendulum that knocks the dominos off
	//~ */	
      //~ b2Body* b2;
      //~ {	
	//~ /*! \par
	 //~ * Variable:shape :: Type:Polygonshape :: Value:height=0.25 and width=1.5 units  <br>
	 //~ * Variable:bd :: Type:BodyDef :: Action:Pendulum bob is attached to it :: Value: position x=-36.5, y=28.0 <br>
	 //~ */
	//~ b2PolygonShape shape;
	//~ shape.SetAsBox(0.25f, 1.5f);  
	  //~ 
	//~ b2BodyDef bd;
	//~ bd.position.Set(-36.5f, 28.0f);
	//~ b2 = m_world->CreateBody(&bd);
	//~ b2->CreateFixture(&shape, 10.0f);
      //~ }
	//~ 
      //~ b2Body* b4;
      //~ {
		  //~ /*! \par
		   //~ * Variable:shape :: Type:PolygonShape :: Value:height=0.25 and width=0.25 units.  <br>
		   //~ * Variable:bd :: Type:BodyDef dynamic_body :: Value:position x=-40.0,y=33.0 :: Action:Acts as a bob. <br>
		   //~ * Variable:jd :: Type:RevoluteJointDef :: Value:position x=-37 and y40 units :: Action:Connecting fixture and bob.
		   //~ */   
	//~ b2PolygonShape shape; 
	//~ shape.SetAsBox(0.25f, 0.25f);
	  //~ 
	//~ b2BodyDef bd;
	//~ bd.type = b2_dynamicBody;
	//~ bd.position.Set(-40.0f, 33.0f);
	//~ b4 = m_world->CreateBody(&bd);
	//~ b4->CreateFixture(&shape, 2.0f);
      //~ }
	//~ 
      //~ b2RevoluteJointDef jd;
      //~ b2Vec2 anchor;
      //~ anchor.Set(-37.0f, 40.0f);
      //~ jd.Initialize(b2, b4, anchor);
      //~ m_world->CreateJoint(&jd);
    //~ }
      //~ 
    //~ 
    //~ {
		//~ /*! \par Block 6:The train of small spheres
		   //~ * Variable:spherebody :: Type:circle :: Value:radius=0.5  <br>
		   //~ * Variable:ballfd :: Type:FixtureDef :: Value:density=1,friction=0,restitution=0 <br>
		   //~ * Variable:ballbd :: number:10 :: Type:dynamicBody :: Value:position x=-22.2+ i and y=26.6 units :: Action:To move after hit by dominos. <br>
		   //~ * 
		   //~ */
      //~ b2Body* spherebody;
	//~ 
      //~ b2CircleShape circle;
      //~ circle.m_radius = 0.5;
	//~ 
      //~ b2FixtureDef ballfd;
      //~ ballfd.shape = &circle;
      //~ ballfd.density = 1.0f;
      //~ ballfd.friction = 0.0f;
      //~ ballfd.restitution = 0.0f;
	//~ 
      //~ for (int i = 0; i < 10; ++i)
	//~ {
	  //~ b2BodyDef ballbd;
	  //~ ballbd.type = b2_dynamicBody;
	  //~ ballbd.position.Set(-22.2f + i*1.0, 26.6f);
	  //~ spherebody = m_world->CreateBody(&ballbd);
	  //~ spherebody->CreateFixture(&ballfd);
	//~ }
    //~ }
//~ 
    //~ 
    //~ {
		 //~ /*! \par Block 7:The pulley system
		   //~ * Variable:bd :: Type:dynamicBody :: Value:position x=-10 and y=15 units.  <br>
		    //~ */   
      //~ b2BodyDef *bd = new b2BodyDef;
      //~ bd->type = b2_dynamicBody;
      //~ bd->position.Set(-10,15);
      //~ bd->fixedRotation = true;
      //~ 
      //~ 
	    //~ /*! \par
	     //~ * The open box 
		 //~ * Variable:fd1,fd2,fd3:: Type:FixtureDef :: Value:density=10,fricion=0.5,restitution=0 :: Action:To collect the spheres and pull down the pulley.  <br>
		 //~ * Variable:bs1 :: Type:PolygonShape :: Value: height=2,width=0.2; position x=0,y=-1.9.<br>
		 //~ * Variable:bs2 :: Type:PolygonShape :: Value: height=0.2,width=2; position x=2,y=-0.<br>
		 //~ * Variable:bs3 :: Type:PolygonShape :: Value: height=0.2,width=2; position x=-2,y=0.<br>
		 //~ * Variable:box1 :: Action: Creates a box of fd1,fd2 and fd3.<br>
		//~ */   
      //~ b2FixtureDef *fd1 = new b2FixtureDef;
      //~ fd1->density = 10.0;
      //~ fd1->friction = 0.5;
      //~ fd1->restitution = 0.f;
      //~ fd1->shape = new b2PolygonShape;
      //~ b2PolygonShape bs1;
      //~ bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      //~ fd1->shape = &bs1;
      //~ b2FixtureDef *fd2 = new b2FixtureDef;
      //~ fd2->density = 10.0;
      //~ fd2->friction = 0.5;
      //~ fd2->restitution = 0.f;
      //~ fd2->shape = new b2PolygonShape;
      //~ b2PolygonShape bs2;
      //~ bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      //~ fd2->shape = &bs2;
      //~ b2FixtureDef *fd3 = new b2FixtureDef;
      //~ fd3->density = 10.0;
      //~ fd3->friction = 0.5;
      //~ fd3->restitution = 0.f;
      //~ fd3->shape = new b2PolygonShape;
      //~ b2PolygonShape bs3;
      //~ bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      //~ fd3->shape = &bs3;
       //~ 
      //~ b2Body* box1 = m_world->CreateBody(bd);
      //~ box1->CreateFixture(fd1);
      //~ box1->CreateFixture(fd2);
      //~ box1->CreateFixture(fd3);
//~ 
      //~ 
      //~ /*! \par Block 8:The bar 
		 //~ * Variable:bd :: Value:position x=10,y=15; density=34.  <br>
		 //~ * Variable:fd1 :: Value: density=34. <br>
		 //~ * Variable:box2 :: Type:Body :: Action: Creates a fixture with fd1.<br>
		//~ */
      //~ bd->position.Set(10,15);	
      //~ fd1->density = 34.0;	  
      //~ b2Body* box2 = m_world->CreateBody(bd);
      //~ box2->CreateFixture(fd1);
//~ 
      //~ 
      //~ /*! \par Block 9:The pulley joint 
		 //~ * Variable:myjoint  <br>
		 //~ * Anchor point on body 1 in world axis of value (-10,15)<br>
		 //~ * Anchor point on body 2 in world axis of value (10,15)<br>
		 //~ * Anchor point on body 1 in world axis of value (-10,20)<br>
		 //~ * Anchor point on body 2 in world axis of value (10,20)<br>
		//~ */
      //~ b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      //~ b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
      //~ b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
      //~ b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for ground 1 in world axis
      //~ b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
      //~ float32 ratio = 1.0f; // Define ratio
      //~ myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      //~ m_world->CreateJoint(myjoint);
    //~ }
//~ 
    //~ 
    //~ {
		//~ /*! \par Block 10:The revolving horizontal platform
		 //~ * Variable:shape :: Type:PolygonShape :: Value: height=2.2,width=0.2 <br>
		 //~ * Variable:bd :: Value: position x=14,y=14 :: Type: dynamicBody :: Action: Makes the heavy sphere fall down <br>
		 //~ * Variable:shape2 :: Type:PolygonShape :: Value: height=0.2,width=2.0<br>
		 //~ * Variable:bd2 :: Value: position x=14,y=16. <br>
		 //~ * Variable:jointDef :: collideConnected = false :: Action: Sets localAnchorA and localAnchorB.  
		//~ */
      //~ b2PolygonShape shape;
      //~ shape.SetAsBox(2.2f, 0.2f);
	//~ 
      //~ b2BodyDef bd;
      //~ bd.position.Set(14.0f, 14.0f);
      //~ bd.type = b2_dynamicBody;
      //~ b2Body* body = m_world->CreateBody(&bd);
      //~ b2FixtureDef *fd = new b2FixtureDef;
      //~ fd->density = 1.f;
      //~ fd->shape = new b2PolygonShape;
      //~ fd->shape = &shape;
      //~ body->CreateFixture(fd);
//~ 
      //~ b2PolygonShape shape2;
      //~ shape2.SetAsBox(0.2f, 2.0f);
      //~ b2BodyDef bd2;
      //~ bd2.position.Set(14.0f, 16.0f);
      //~ b2Body* body2 = m_world->CreateBody(&bd2);
//~ 
      //~ b2RevoluteJointDef jointDef;
      //~ jointDef.bodyA = body;
      //~ jointDef.bodyB = body2;
      //~ jointDef.localAnchorA.Set(0,0);
      //~ jointDef.localAnchorB.Set(0,0);
      //~ jointDef.collideConnected = false;
      //~ m_world->CreateJoint(&jointDef);
    //~ }
//~ 
    //~ {
		//~ /*! \par Block 11:The heavy sphere on the platform
		 //~ * Variable:ballfd :: shape:Circle :: Value: radius=1, density=50,fricion=0,restitution=0 <br>
		 //~ * Variable:ballbd :: Type: dynamicBody :: position x=14,y=18 :: Action: Will fall on the see-saw system after horizontal platform starts rotating<br>  
		//~ */
      //~ b2Body* sbody;
      //~ b2CircleShape circle;
      //~ circle.m_radius = 1.0;
	//~ 
      //~ b2FixtureDef ballfd;
      //~ ballfd.shape = &circle;
      //~ ballfd.density = 50.0f;
      //~ ballfd.friction = 0.0f;
      //~ ballfd.restitution = 0.0f;
      //~ b2BodyDef ballbd;
      //~ ballbd.type = b2_dynamicBody;
      //~ ballbd.position.Set(14.0f, 18.0f);
      //~ sbody = m_world->CreateBody(&ballbd);
      //~ sbody->CreateFixture(&ballfd);
    //~ }
//~ 
//~ 
    //~ 
    //~ {
	//~ /*! \par Block 12:The see-saw system at the bottom
		 //~ */
	//~ /*! \par
	 //~ * The Wedge placed on the ground to support plank
	 //~ * Variable:poly :: Value: Triangle with coordinates (1,0),(-1,0),(0,1.5) :: Type: PolygonShape. <br>
         //~ * Variable:wedgefd :: Value: Density=10,friction=0,restitution=0,shape=poly :: Type: b2FixtureDef. <br>
         //~ * Variable:wedgebd :: Value: Position x=30 y=0 :: Type:b2BodyDef. <br>
         //~ */ 
      //~ b2Body* sbody;
      //~ b2PolygonShape poly;
      //~ b2Vec2 vertices[3];
      //~ vertices[0].Set(-1,0);
      //~ vertices[1].Set(1,0);
      //~ vertices[2].Set(0,1.5);
      //~ poly.Set(vertices, 3);
      //~ b2FixtureDef wedgefd;
      //~ wedgefd.shape = &poly;
      //~ wedgefd.density = 10.0f;
      //~ wedgefd.friction = 0.0f;
      //~ wedgefd.restitution = 0.0f;
      //~ b2BodyDef wedgebd;
      //~ wedgebd.position.Set(30.0f, 0.0f);
      //~ sbody = m_world->CreateBody(&wedgebd);
      //~ sbody->CreateFixture(&wedgefd);
      //~ 
      //~ /*! \par
       //~ * The plank on top of the wedge
       //~ * Variable:shape :: Value: height=15,width=0.2 :: Type:Polygonshape. <br>
       //~ * Variable:bd2 :: Value: Position x=30,y=1.5. <br>
       //~ * Variable:fd2 :: Value: density=1,shape=PolygonShape. <br>
       //~ * Variable:jd :: anchor: x=30,y=1.5.  
       //~ */
      //~ b2PolygonShape shape;
      //~ shape.SetAsBox(15.0f, 0.2f);  
      //~ b2BodyDef bd2;
      //~ bd2.position.Set(30.0f, 1.5f);
      //~ bd2.type = b2_dynamicBody;
      //~ b2Body* body = m_world->CreateBody(&bd2);
      //~ b2FixtureDef *fd2 = new b2FixtureDef;
      //~ fd2->density = 1.f;
      //~ fd2->shape = new b2PolygonShape;
      //~ fd2->shape = &shape;
      //~ body->CreateFixture(fd2);
//~ 
      //~ b2RevoluteJointDef jd;
      //~ b2Vec2 anchor;
      //~ anchor.Set(30.0f, 1.5f);
      //~ jd.Initialize(sbody, body, anchor);
      //~ m_world->CreateJoint(&jd);
//~ 
      //~ 
      //~ /*! \par
       //~ * The light box on the right side of the see-saw
       //~ * Variable:shape2 :: Value: height=2,width=2 :: Type:Polygonshape. <br>
       //~ * Variable:bd3 :: Value: Position x=40,y=2. <br>
       //~ * Variable:fd3 :: Value: density=0.01,shape=PolygonShape.  
       //~ */
      //~ b2PolygonShape shape2;
      //~ shape2.SetAsBox(2.0f, 2.0f);
      //~ b2BodyDef bd3;
      //~ bd3.position.Set(40.0f, 2.0f);
      //~ bd3.type = b2_dynamicBody;
      //~ b2Body* body3 = m_world->CreateBody(&bd3);
      //~ b2FixtureDef *fd3 = new b2FixtureDef;
      //~ fd3->density = 0.01f;
      //~ fd3->shape = new b2PolygonShape;
      //~ fd3->shape = &shape2;
      //~ body3->CreateFixture(fd3);
    //~ }
//~ 
     //~ //A fixed wedge giving direction to falling ball
     //~ {
	//~ /*! \par Block 13: The Wedge fixed to give direction to falling ball
	 //~ * Variable:polygon :: Value: Triangle with coordinates (0,0),(3,0),(0,2.5) :: Type: PolygonShape. <br>
         //~ * Variable:bf :: Value: Density=90, shape=polygon :: Type: b2FixtureDef. <br>
         //~ * Variable:bd :: Value: Position x=15.5 y=10 :: Type:b2BodyDef. <br>
         //~ */ 
	//~ b2Vec2 vertices[3];
	//~ vertices[0].Set(0.0f, 0.0f);
	//~ vertices[1].Set(3.0f, 0.0f);
	//~ vertices[2].Set(0.0f, 2.5f);
	//~ int32 count = 3;
	//~ b2PolygonShape polygon;
	//~ polygon.Set(vertices, count);
	//~ b2FixtureDef bf;
	//~ bf.shape = &polygon;
	//~ bf.density = 90.0f;
	//~ b2BodyDef bd;
	//~ bd.position.Set(15.5f, 10.0f);
	//~ b2Body* ground = m_world->CreateBody(&bd);
	//~ ground->CreateFixture(&bf);
    //~ }

     //One side rotating platform
     {
	/*! \par Block 14: One side Hinged platform
         */

	/*! \par 
	 * Plank that will be rotating
	 * Variable:shape :: Value: Width: 0.4, Length:25.0 :: Type: PolygonShape. <br>
         * Variable:fd :: Value: Density=0.0001, shape=shape :: Type: b2FixtureDef. <br>
         * Variable:bd :: Value: Position x=32.5 y=24 Type=b2_dynamicBody :: DataType:b2BodyDef. <br>
         */ 
      b2PolygonShape shape;
      shape.SetAsBox(12.5f, 0.5f);
      b2BodyDef bd;
      bd.position.Set(32.5f, 24.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 0.0001f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
      
      b2PolygonShape shape2;
      shape2.SetAsBox(0.5f, 12.5f);
      b2BodyDef bd2;
      bd2.position.Set(20.5f, 11.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body2 = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 0.0001f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape2;
      body2->CreateFixture(fd2);
      
      b2BodyDef bd3;
      bd3.position.Set(44.5f, 11.5f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      body3->CreateFixture(fd2);
      
      b2BodyDef bd4;
      bd4.position.Set(32.5f, 18.0f);
      bd4.type = b2_dynamicBody;
      b2Body* body4 = m_world->CreateBody(&bd4);
      body4->CreateFixture(fd);


	/*! \par 
	 * Anchor to join circle and plank.
	 * Variable:jointDef :: Value: collideConnected=false, Initialized with anchor at center of circle :: Type: RevoluteJointDef. <br>
         */      
      b2RevoluteJointDef jointDef;
      b2Vec2 v1(20.5f,24.0f);
      jointDef.Initialize(body,body2,v1);
      jointDef.localAnchorA.Set(-12.0f,0.0f) ;
      jointDef.localAnchorB.Set(0.0f,12.0f);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);		
      
      b2RevoluteJointDef jointDef2;
      b2Vec2 v2(44.5f,24.0f);
      jointDef2.Initialize(body,body3,v2);
      jointDef2.localAnchorA.Set(12.0f,0.0f) ;
      jointDef2.localAnchorB.Set(0.0f,12.0f);
      jointDef2.collideConnected = false;
      m_world->CreateJoint(&jointDef2);		
      
      b2RevoluteJointDef jointDef3;
      b2Vec2 v3(20.5f,18.0f);
      jointDef3.Initialize(body4,body2,v3);
      jointDef3.localAnchorA.Set(-12.0f,0.0f) ;
      jointDef3.localAnchorB.Set(0.0f,6.0f);
      jointDef3.collideConnected = false;
      m_world->CreateJoint(&jointDef3);		
      
      b2RevoluteJointDef jointDef4;
      b2Vec2 v4(44.5f,18.0f);
      jointDef4.Initialize(body4,body3,v4);
      jointDef4.localAnchorA.Set(12.0f,0.0f) ;
      jointDef4.localAnchorB.Set(0.0f,6.0f);
      jointDef4.collideConnected = false;
      m_world->CreateJoint(&jointDef4);		
     }
     //stopper
     //~ { 
	//~ /*! \par Block 15: Stopper to stop plank at one side.
	 //~ * Variable:shape :: Value: Width: 0.4, Length:2.0 :: Type: PolygonShape. <br>
         //~ * Variable:bf :: Value: Density=90, shape=shape :: Type: b2FixtureDef. <br>
         //~ * Variable:bd :: Value: Position x=45 y=20.5:: DataType:b2BodyDef. <br>
         //~ */ 
       //~ b2PolygonShape shape;
       //~ shape.SetAsBox(1.0f,0.2f);
       //~ b2BodyDef bd;
       //~ bd.position.Set(45.0f,20.5f);
       //~ b2Body* body = m_world->CreateBody(&bd); 	
       //~ b2FixtureDef bf;
       //~ bf.shape = &shape;
       //~ bf.density = 90.0f;
       //~ body->CreateFixture(&bf);
      //~ }
      //base for square block
      //~ {
	//~ /*! \par Block 16: Plank for Square to rest 
	 //~ * Variable:shape :: Value: Width: 0.4, Length:2.0 :: Type: PolygonShape. <br>
         //~ * Variable:bf :: Value: Density=90, Friction=1, shape=shape :: Type: b2FixtureDef. <br>
         //~ * Variable:bd :: Value: Position x=9 y=30.5 :: DataType:b2BodyDef. <br>
         //~ */ 
       //~ b2PolygonShape shape;
       //~ shape.SetAsBox(1.0f,0.2f);
       //~ b2BodyDef bd;
       //~ bd.position.Set(9.0f,30.5f);
       //~ b2Body* body = m_world->CreateBody(&bd); 	
       //~ b2FixtureDef bf;
       //~ bf.shape = &shape;
       //~ bf.density = 90.0f;
       //~ bf.friction=1.0f;
       //~ body->CreateFixture(&bf);
      //~ }
      //~ 
      //~ {
	//~ /*! \par Block 17: Falling Square 
	 //~ * Variable:shape2 :: Value: Width: 4, Length:4 :: Type: PolygonShape. <br>
         //~ * Variable:bf :: Value: Density=0.00001, shape=shape2 :: Type: b2FixtureDef. <br>
         //~ * Variable:bd :: Value: Position x=9 y=32.5 :: DataType:b2BodyDef. <br>
         //~ */ 
      //~ b2PolygonShape shape2;
      //~ shape2.SetAsBox(2.0f, 2.0f);
      //~ b2BodyDef bd3;
      //~ bd3.position.Set(9.0f, 32.5f);
      //~ bd3.type = b2_dynamicBody;
      //~ b2Body* body3 = m_world->CreateBody(&bd3);
      //~ b2FixtureDef *fd3 = new b2FixtureDef;
      //~ fd3->density = 0.00001f;
      //~ fd3->shape = new b2PolygonShape;
      //~ fd3->shape = &shape2;
      //~ body3->CreateFixture(fd3);
    //~ }
//~ 
	
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}


