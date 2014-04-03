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
    {
		/*! \par Block 6:The train of small spheres
		   * Variable:spherebody :: Type:circle :: Value:radius=0.5  <br>
		   * Variable:ballfd :: Type:FixtureDef :: Value:density=1,friction=0,restitution=0 <br>
		   * Variable:ballbd :: number:10 :: Type:dynamicBody :: Value:position x=-22.2+ i and y=26.6 units :: Action:To move after hit by dominos. <br>
		   * 
		   */
		
		//Horizontal top rod
      b2PolygonShape shape;
      shape.SetAsBox(8.5f, 0.5f);
      b2BodyDef bd;
      bd.position.Set(0.0f, 21.5f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 0.1f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
     // Left hand side vertical rod 
      b2PolygonShape shape2;
      shape2.SetAsBox(0.5f, 10.0f);
      b2BodyDef bd2;
      bd2.position.Set(-8.0f, 11.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body2 = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 0.1f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape2;
      body2->CreateFixture(fd2);
     //Right hand side vertical rod  
      b2BodyDef bd3;
      bd3.position.Set(8.0f, 11.5f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      body3->CreateFixture(fd2);
     // Lower horizontal rod 
      b2BodyDef bd4;
      bd4.position.Set(0.0f, 15.5f);
      bd4.type = b2_dynamicBody;
      b2Body* body4 = m_world->CreateBody(&bd4);
      body4->CreateFixture(fd);
     //Joint for top rod and left rod
      b2RevoluteJointDef jointDefs;
      b2Vec2 v1(-8.0f,21.5f);
      jointDefs.Initialize(body,body2,v1);
      jointDefs.localAnchorA.Set(-8.0f,0.0f) ;
      jointDefs.localAnchorB.Set(0.0f,9.5f);
      jointDefs.collideConnected = false;
      m_world->CreateJoint(&jointDefs);		
      //Joint for top rod and right rod
      b2RevoluteJointDef jointDefs2;
      b2Vec2 v2(8.0f,24.0f);
      jointDefs2.Initialize(body,body3,v2);
      jointDefs2.localAnchorA.Set(8.0f,0.0f) ;
      jointDefs2.localAnchorB.Set(0.0f,9.5f);
      jointDefs2.collideConnected = false;
      m_world->CreateJoint(&jointDefs2);		
      //joint for bottom rod and left rod
      b2RevoluteJointDef jointDefs3;
      b2Vec2 v3(-8.0f,18.0f);
      jointDefs3.Initialize(body4,body2,v3);
      jointDefs3.localAnchorA.Set(-8.0f,0.0f) ;
      jointDefs3.localAnchorB.Set(0.0f,4.0f);
      jointDefs3.collideConnected = false;
      m_world->CreateJoint(&jointDefs3);		
      //joint for bottom rod and right rod
      b2RevoluteJointDef jointDefs4;
      b2Vec2 v4(8.0f,18.0f);
      jointDefs4.Initialize(body4,body3,v4);
      jointDefs4.localAnchorA.Set(8.0f,0.0f) ;
      jointDefs4.localAnchorB.Set(0.0f,4.0f);
      jointDefs4.collideConnected = false;
      m_world->CreateJoint(&jointDefs4);	
      //left tyre
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 2.5;
      b2FixtureDef ballfds;
      ballfds.shape = &circle;
      ballfds.density = 1.0f;
      ballfds.friction = 0.5f;
      ballfds.restitution = 0.0f;
      b2BodyDef ballbds;
      ballbds.type = b2_dynamicBody;
      ballbds.position.Set(-8.0f, 2.5f);
      sbody = m_world->CreateBody(&ballbds);
      sbody->CreateFixture(&ballfds);	
      //right tyre
      b2Body* sbody2;
      b2BodyDef ballbd2;
      ballbd2.type = b2_dynamicBody;
      ballbd2.position.Set(8.0f, 2.5f);
      sbody2 = m_world->CreateBody(&ballbd2);
      sbody2->CreateFixture(&ballfds);	
      //Joint joining right rod and right tyre
      b2RevoluteJointDef jointDefs5;
      jointDefs5.Initialize(sbody2,body3,sbody2->GetWorldCenter());
      jointDefs5.localAnchorB.Set(0.0f,-9.5f);
      jointDefs5.collideConnected = false;
      m_world->CreateJoint(&jointDefs5);	
      jointDefs5.enableMotor = true;
      jointDefs5.maxMotorTorque = 200000.0f;
      jointDefs5.motorSpeed = 36.0f;
      //Joint joining left rod and left tyre
      b2RevoluteJointDef jointDefs6;
      jointDefs6.Initialize(sbody,body2,sbody->GetWorldCenter());
      jointDefs6.localAnchorB.Set(0.0f,-9.5f);
      jointDefs6.collideConnected = false;
      m_world->CreateJoint(&jointDefs6);	
      jointDefs6.enableMotor = true;
      jointDefs6.maxMotorTorque = 20000.0f;
      jointDefs6.motorSpeed = 36.0f;
      sbody2->SetAngularVelocity(10.0f); 
      sbody->SetAngularVelocity(-0.0f);
		
		//palash
		//front part
		
		//front tyre
		b2Body* spherebody;
		b2FixtureDef ballfd;
		ballfd.shape = &circle;
		ballfd.density = 1.0f;
		ballfd.friction = 0.5f;
		ballfd.restitution = 0.0f;
		b2BodyDef ballbd;
		ballbd.type = b2_dynamicBody;
		ballbd.position.Set(22.f, 2.5f);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);

		//L1
		int32 count = 4;
		b2Vec2 vertices[4];
		vertices[2].Set(0.5f, 0.5f);
		vertices[1].Set(0.5f, -5.5f);
		vertices[0].Set(-0.5f, -5.5f);
		vertices[3].Set(-0.5f, 0.5f);
		b2Body* L1;
		b2PolygonShape l1shape;
		l1shape.Set(vertices, count);
		b2FixtureDef l1fd;
		l1fd.shape = &l1shape;
		l1fd.density = 1.0f;
		l1fd.friction = 0.0f;
		l1fd.restitution = 0.0f;
		b2BodyDef l1bd;
		l1bd.type = b2_dynamicBody;
		l1bd.position.Set(22.f, 8.f);
		L1 = m_world->CreateBody(&l1bd);
		L1->CreateFixture(&l1fd);
		
		//L2
		b2Vec2 vertices1[4];
		vertices1[2].Set(0.5f, 0.5f);
		vertices1[1].Set(0.5f, -0.5f);
		vertices1[0].Set(-3.5f, -0.5f);
		vertices1[3].Set(-3.5f, 0.5f);
		b2Body* L2;
		b2PolygonShape l2shape;
		l2shape.Set(vertices1, count);
		b2FixtureDef l2fd;
		l2fd.shape = &l2shape;
		l2fd.density = 1.0f;
		l2fd.friction = 0.0f;
		l2fd.restitution = 0.0f;
		b2BodyDef l2bd;
		l2bd.type = b2_dynamicBody;
		l2bd.position.Set(22.f, 8.f);
		L2 = m_world->CreateBody(&l2bd);
		L2->CreateFixture(&l2fd);
		
		//R1
		b2Vec2 vertices2[4];
		vertices2[2].Set(0.5f, 20.f);
		vertices2[1].Set(0.5f, 0);
		vertices2[0].Set(-0.5f,0);
		vertices2[3].Set(-0.5f, 20.f);
		b2Body* R1;
		b2PolygonShape r1shape;
		r1shape.Set(vertices2, count);
		b2FixtureDef r1fd;
		r1fd.shape = &r1shape;
		r1fd.density = 1.0f;
		r1fd.friction = 0.0f;
		r1fd.restitution = 0.0f;
		b2BodyDef r1bd;
		r1bd.type = b2_dynamicBody;
		r1bd.position.Set(18.5f, 8.f);
		r1bd.angle = 0.2;
		R1 = m_world->CreateBody(&r1bd);
		R1->CreateFixture(&r1fd);
		
		//front tyre and L1
		b2RevoluteJointDef jointDef;
		jointDef.bodyA = spherebody;
		jointDef.bodyB = L1;
		jointDef.localAnchorA.Set(0,0);
		jointDef.localAnchorB.Set(0, -5.5f);
		jointDef.collideConnected = false;
		jointDef.enableMotor = true;
		m_world->CreateJoint(&jointDef);
		
		//L1 and L2
		b2WeldJointDef jointDef1;
		jointDef1.bodyA = L1;
		jointDef1.bodyB = L2;
		jointDef1.localAnchorA.Set(0,0);
		jointDef1.localAnchorB.Set(0,0);
		jointDef1.collideConnected = false;
		m_world->CreateJoint(&jointDef1);
		
		//L2 and R1
		b2RevoluteJointDef jointDef2;
		jointDef2.bodyA = L2;
		jointDef2.bodyB = R1;
		jointDef2.localAnchorA.Set(-3.5,0);
		jointDef2.localAnchorB.Set(0, 0);
		jointDef2.collideConnected = false;
		jointDef2.enableMotor = true;
		m_world->CreateJoint(&jointDef2);
		
		//back part
		
		//back tyre
		b2Body* spherebody1;
		b2BodyDef ballbd1;
		ballbd1.type = b2_dynamicBody;
		ballbd1.position.Set(-20.f, 2.5f);
		spherebody1 = m_world->CreateBody(&ballbd1);
		spherebody1->CreateFixture(&ballfd);
		
		//L3
		b2Vec2 vertices3[4];
		vertices3[2].Set(0.5f, 0.5f);
		vertices3[1].Set(0.5f, -12.5f);
		vertices3[0].Set(-0.5f, -12.5f);
		vertices3[3].Set(-0.5f, 0.5f);
		b2Body* L3;
		b2PolygonShape l3shape;
		l3shape.Set(vertices3, count);
		b2FixtureDef l3fd;
		l3fd.shape = &l3shape;
		l3fd.density = 1.0f;
		l3fd.friction = 0.0f;
		l3fd.restitution = 0.0f;
		b2BodyDef l3bd;
		l3bd.type = b2_dynamicBody;
		l3bd.position.Set(-20.f, 15.f);
		L3 = m_world->CreateBody(&l3bd);
		L3->CreateFixture(&l3fd);
		
		//L3 and back tyre
		b2RevoluteJointDef jointDef3;
		jointDef3.bodyA = spherebody1;
		jointDef3.bodyB = L3;
		jointDef3.localAnchorA.Set(0,0);
		jointDef3.localAnchorB.Set(0, -12.5f);
		jointDef3.collideConnected = false;
		jointDef3.enableMotor = true;
		m_world->CreateJoint(&jointDef3);
		
		//L4
		b2Vec2 vertices4[4];
		vertices4[2].Set(0.5f, 0.5f);
		vertices4[1].Set(0.5f, -0.5f);
		vertices4[0].Set(5.5f, -0.5f);
		vertices4[3].Set(5.5f, 0.5f);
		b2Body* L4;
		b2PolygonShape l4shape;
		l4shape.Set(vertices4, count);
		b2FixtureDef l4fd;
		l4fd.shape = &l4shape;
		l4fd.density = 1.0f;
		l4fd.friction = 0.0f;
		l4fd.restitution = 0.0f;
		b2BodyDef l4bd;
		l4bd.type = b2_dynamicBody;
		l4bd.position.Set(-20.f, 15.f);
		L4 = m_world->CreateBody(&l4bd);
		L4->CreateFixture(&l4fd);
		
		//L3 and L4
		b2WeldJointDef jointDef4;
		jointDef1.bodyA = L3;
		jointDef1.bodyB = L4;
		jointDef1.localAnchorA.Set(0,0);
		jointDef1.localAnchorB.Set(0,0);
		jointDef1.collideConnected = false;
		m_world->CreateJoint(&jointDef1);
		
		//R2
		b2Vec2 vertices5[4];
		vertices5[2].Set(0.5f, 6.5f);
		vertices5[1].Set(0.5f, 0);
		vertices5[0].Set(-0.5f,0);
		vertices5[3].Set(-0.5f, 6.5f);
		b2Body* R2;
		b2PolygonShape r2shape;
		r2shape.Set(vertices5, count);
		b2FixtureDef r2fd;
		r2fd.shape = &r2shape;
		r2fd.density = 1.0f;
		r2fd.friction = 0.0f;
		r2fd.restitution = 0.0f;
		b2BodyDef r2bd;
		r2bd.type = b2_dynamicBody;
		r2bd.position.Set(-14.5f, 15.f);
		r2bd.angle = -0.4;
		R2 = m_world->CreateBody(&r2bd);
		R2->CreateFixture(&r2fd);
		
		//L4 and R2
		b2RevoluteJointDef jointDef5;
		jointDef5.bodyA = L4;
		jointDef5.bodyB = R2;
		jointDef5.localAnchorA.Set(5.5,0);
		jointDef5.localAnchorB.Set(0, 0);
		jointDef5.collideConnected = false;
		jointDef5.enableMotor = true;
		m_world->CreateJoint(&jointDef5);
	}
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
//~ 
     //~ //One side rotating platform
     //~ {
	//~ /*! \par Block 14: One side Hinged platform
         //~ */
//~ 
	//~ /*! \par 
	 //~ * Plank that will be rotating
	 //~ * Variable:shape :: Value: Width: 0.4, Length:25.0 :: Type: PolygonShape. <br>
         //~ * Variable:fd :: Value: Density=0.0001, shape=shape :: Type: b2FixtureDef. <br>
         //~ * Variable:bd :: Value: Position x=32.5 y=24 Type=b2_dynamicBody :: DataType:b2BodyDef. <br>
         //~ */ 
      //~ b2PolygonShape shape;
      //~ shape.SetAsBox(12.5f, 0.2f);
      //~ b2BodyDef bd;
      //~ bd.position.Set(32.5f, 24.0f);
      //~ bd.type = b2_dynamicBody;
      //~ b2Body* body = m_world->CreateBody(&bd);
      //~ b2FixtureDef *fd = new b2FixtureDef;
      //~ fd->density = 0.0001f;
      //~ fd->shape = new b2PolygonShape;
      //~ fd->shape = &shape;
      //~ body->CreateFixture(fd);
//~ 
	//~ /*! \par 
	 //~ * Circle to which plank is anchored
	 //~ * Variable:circle :: Value: Radius = 1 :: Type: CircleShape. <br>
         //~ * Variable:fd2 :: Value: Density=1, shape=circle :: Type: b2FixtureDef. <br>
         //~ * Variable:ballbd :: Value: Position x=20 y=24 :: DataType:b2BodyDef. <br>
         //~ */      
      //~ b2Body* body2;
      //~ b2CircleShape circle;
      //~ circle.m_radius = 1.0f;
      //~ b2BodyDef ballbd;
      //~ ballbd.position.Set(20.0f, 24.0f);
      //~ body2 = m_world->CreateBody(&ballbd);
      //~ b2FixtureDef fd2;
      //~ fd2.density = 1.0f;
      //~ fd2.shape = &circle;
      //~ body2->CreateFixture(&fd2);
 //~ 
	//~ /*! \par 
	 //~ * Anchor to join circle and plank.
	 //~ * Variable:jointDef :: Value: collideConnected=false, Initialized with anchor at center of circle :: Type: RevoluteJointDef. <br>
         //~ */      
      //~ b2RevoluteJointDef jointDef;
      //~ jointDef.Initialize(body2,body,body2->GetWorldCenter());
      //~ jointDef.collideConnected = false;
      //~ m_world->CreateJoint(&jointDef);		
     //~ }
     //~ //stopper
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
      //~ //base for square block
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
	//~ 
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}


