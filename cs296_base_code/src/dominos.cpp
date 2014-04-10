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
	float i=0;
	b2RevoluteJoint* frontJoint; /*!< The revolute joint between frontmost tyre and rod connected to it */
	b2RevoluteJoint* backJoint;/*!< The revolute joint between backmost tyre and rod connected to it */
	b2RevoluteJoint* centerBackJoint;/*!< The revolute joint between centre back tyre and rod connected to it */
	b2RevoluteJoint* centerFrontJoint;/*!< The revolute joint between center front tyre and rod connected to it */
	///The inputs taken from the keyboard <br>
 	void dominos_t::keyboard(unsigned char key)
	{
		switch(key)
		{
			/**When 'd' key is pressed the speed of the all the revolute joints of tyres increases by 0.1f for each press untill it becomes 3.f <br>*/
			case('d'):
				if(i <3){
				i = i+0.1f;
				}
				frontJoint->SetMotorSpeed(i);
				backJoint->SetMotorSpeed(i);
				centerFrontJoint->SetMotorSpeed(i);
				centerBackJoint->SetMotorSpeed(i);
				break;
			/**Similarly when 'a' key is pressed the speed of the all the revolute joints of tyres decreases by 0.1f for each press untill it becomes -3.f <br>*/
			case('a'):
				if(i>-3){
				i = i-0.1f;
				}
				frontJoint->SetMotorSpeed(i);
				backJoint->SetMotorSpeed(i);
				centerFrontJoint->SetMotorSpeed(i);
				centerBackJoint->SetMotorSpeed(i);
				break;
		}
		
	}	

  dominos_t::dominos_t()
  {
		
    b2Body* b1; //! b1 is a pointer to the ground which is a Box2D body. 

	/*! \par Block 1: Ground
	 * A Box2D Edge shape used to represent ground. <br>
	 * Edge is set between two points on x-axis x1=-1800.0f and x2=1800.0f <br>
	 * bd is a Box2D body definition.<br>
	 * b1 is then pointed to a body created with body definition bd<br>
	 * A fixture is created on the body with Edge shape created earlier and 0 density since it is a static body <br>
	 */
    {
      b2EdgeShape shape;     
      shape.Set(b2Vec2(-1800.0f, 0.0f), b2Vec2(1800.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
	
    {
		//Center Horizontal top rod
		/*! \par centerTop
		 * Center Horizontal top rod <br>
		 * Shape: b2PolygonShape (Rectangle) <br>
		 * A box of length 14.0f and breadth 2.0f <br>
		 * Position x=0.0f, y=15.f <br>
		 * It is a dynamic body <br>
		 * categoryBit = 0x0002 <br>
		 * density=2700.f <br>
		 */
		b2PolygonShape centerTopShape;
		centerTopShape.SetAsBox(7.0f, 1.0f);
		b2BodyDef centerTopbd;
		centerTopbd.position.Set(0.0f, 15.0f);
		centerTopbd.type = b2_dynamicBody;
		b2Body* centerTop = m_world->CreateBody(&centerTopbd);
		b2FixtureDef centerTopfd;
		centerTopfd.filter.categoryBits = 0x0002;
		centerTopfd.density = 2700.0f;
		centerTopfd.shape = &centerTopShape;
		centerTop->CreateFixture(&centerTopfd);
		//Center Horizontal bottom rod
		/*! \par centerBottom
		 * Center Horizontal bottom rod <br>
		 * Position x=0.0f, y=11.0f <br> 
		 * It is a dynamic body <br>
		 * body and fixture definition same as centerTop <br>
		 */
		b2BodyDef centerBottombd;
		centerBottombd.position.Set(0.0f, 11.f);
		centerBottombd.type = b2_dynamicBody;
		b2Body* centerBottom = m_world->CreateBody(&centerBottombd);
		centerBottom->CreateFixture(&centerTopfd);
		//Center vertical left rod
		/*! \par centerLeft
		 * Center vertical left rod <br>
		 * This is a dynamic body <br>
		 * A polygon with four vertices making a rectangle with dimensions 2.0f x 13.0f <br>
		 * The position of its representative (center) point is x=-6.0f, y=10.0f <br>
		 * Category bit=0x0002 <br>
		 * Density of the rod is same as alumninium = 2700.0f <br>
		 */
		b2PolygonShape centerLeftShape;
		int32 count = 4;
		b2Vec2 ver[4];
		ver[2].Set(1.f, 6.0f);
		ver[1].Set(1.f, -7.0f);
		ver[0].Set(-1.f, -7.0f);
		ver[3].Set(-1.f, 6.0f);
		centerLeftShape.Set(ver, count);
		b2BodyDef centerLeftbd;
		centerLeftbd.position.Set(-6.0f, 10.0f);
		centerLeftbd.type = b2_dynamicBody;
		b2Body* centerLeft = m_world->CreateBody(&centerLeftbd);
		b2FixtureDef centerLeftfd;
		centerLeftfd.filter.categoryBits = 0x0002;
		centerLeftfd.density = 2700.0f;
		centerLeftfd.shape = &centerLeftShape;
		centerLeft->CreateFixture(&centerLeftfd);
		//Center Vertical Right rod
		/*! \par centerRight
		 * This is also a dynamic body <br>
		 * The fixture definition is same as that of center vertical left rod <br>
		 * The position of its representative (center) point is at x=6.0f, y=10.0f <br>
		 */
		b2BodyDef centerRightbd;
		centerRightbd.position.Set(6.0f, 10.0f);
		centerRightbd.type = b2_dynamicBody;
		b2Body* centerRight = m_world->CreateBody(&centerRightbd);
		centerRight->CreateFixture(&centerLeftfd);
		//Joint between center left and center top  rod
		/*! \par centerLeftTopJoint <br>
		 * The revolute joint between center left and center top rod <br>
		 * Joint position is x=-6.0f and y=15.0f <br>
		 * Body A is centerTop and its local anchor is set at (-6.0f, 0.0f) at the top end of centerLeft rod <br>
		 * Body B is centerLeft and its local anchor is set at (0.0f, 5.0f) at the left end point of centerTop rod <br>
		 * collideConnected Boolean is set to false so that the two rods do not collide <br>
		 */
		b2RevoluteJointDef centerLeftTopJoint;
        b2Vec2 centerLeftTopJointv1(-6.0f,15.0f);
        centerLeftTopJoint.Initialize(centerTop,centerLeft,centerLeftTopJointv1);
        centerLeftTopJoint.localAnchorA.Set(-6.0f,0.0f) ;
        centerLeftTopJoint.localAnchorB.Set(0.0f,5.0f);
        centerLeftTopJoint.collideConnected = false;
        m_world->CreateJoint(&centerLeftTopJoint);	
        //Joint between center right and center top  rod
        /*! \par centerRightTopJoint
         * The revolute joint between center right and center top rod <br>
         * Joint position is x=6.0f and y=15.0f <br>
         * Body A is centerTop and its local anchor is set at (6.0f,0.0f) at the top end of centerRight rod <br>
         * Body B is centerRight and its local Anchor is set at (0.0f,5.0f) at the right end point of centerTop rod <br>
         * collideConnected Boolean is set to false so that the two rods do not collide <br>
		 */
		b2RevoluteJointDef centerRightTopJoint;
        b2Vec2 centerRightTopJointv1(6.0f,15.0f);
        centerRightTopJoint.Initialize(centerTop,centerRight,centerRightTopJointv1);
        centerRightTopJoint.localAnchorA.Set(6.0f,0.0f) ;
        centerRightTopJoint.localAnchorB.Set(0.0f,5.0f);
        centerRightTopJoint.collideConnected = false;
        m_world->CreateJoint(&centerRightTopJoint);	
        //Joint between center left and center bottom  rod
        /*! \par centerLeftBottomJoint
         * The revolute joint between center left and center bottom rod <br>
         * Joint position is x=-6.0f and y=11.0f <br>
         * Body A is centerBottom and its localAnchor is set at (-6.0f, 0.0f) at around center of the centerLeft rod <br>
         * Body B is centerLeft and its localAnchor is set at (0.0f, 1.0f) at the left end point of the centerBottom rod <br>
         * collideConnected is set to false so that the two rods do not collide <br> 
         */
		b2RevoluteJointDef centerLeftBottomJoint;
        b2Vec2 centerLeftBottomJointv1(-6.0f,11.0f);
        centerLeftBottomJoint.Initialize(centerBottom,centerLeft,centerLeftBottomJointv1);
        centerLeftBottomJoint.localAnchorA.Set(-6.0f,0.0f) ;
        centerLeftBottomJoint.localAnchorB.Set(0.0f,1.0f);
        centerLeftBottomJoint.collideConnected = false;
        m_world->CreateJoint(&centerLeftBottomJoint);	
        //Joint between center right and center top  rod
        /*! \par centerRightBottomJoint
         * The revolute joint between between center right and center bottom rod <br>
         * Joint position is x=6.0f and y=11.0f <br>
         * Body A is centerBottom and its local Anchor is set at (6.0f, 0.0f) at around center of the centerRight rod <br>
         * Body B is centerRight and its local anchor is set at (0.0f, 1.0f) at the right endpoint of the centerBottom rod <br>
         * Again the collideConnected boolean is set to false so that the two rods do not collide <br> 
         */
		b2RevoluteJointDef centerRightBottomJoint;
        b2Vec2 centerRightBottomJointv1(6.0f,11.0f);
        centerRightBottomJoint.Initialize(centerBottom,centerRight,centerRightBottomJointv1);
        centerRightBottomJoint.localAnchorA.Set(6.0f,0.0f) ;
        centerRightBottomJoint.localAnchorB.Set(0.0f,1.0f);
        centerRightBottomJoint.collideConnected = false;
        m_world->CreateJoint(&centerRightBottomJoint);	
        //Center left tyre 
        /*! \par centerLeftTire and the centerRightTire <br>
         * Radius of the tyres is 3.5f <br>
         * Friction of the tyre is 1.0f <br>
         * The coefficient of restitution is also 1.0f for possibility of elastic collisions <br>
         * The position of centerLeftTire is (-6.0f, 3.5f) and centerRightTire is (6.0f, 3.5f) <br>
         */
        /*!
         * Common shape and fixture definitions for both tires <br>
         */ 
        b2CircleShape centerLeftcircle;
        centerLeftcircle.m_radius = 3.5f;
        b2FixtureDef centerLeftTirefd;
        centerLeftTirefd.shape = &centerLeftcircle;
        centerLeftTirefd.density = 3000.0f;
        centerLeftTirefd.friction = 1.0f;
        centerLeftTirefd.restitution = 1.0f;
        
        b2BodyDef centerLeftTirebd;
        centerLeftTirebd.type = b2_dynamicBody;
        centerLeftTirebd.position.Set(-6.0f, 3.5f);
        b2Body* centerLeftTire = m_world->CreateBody(&centerLeftTirebd);
        centerLeftTire->CreateFixture(&centerLeftTirefd);	
        //Center right tyre
        b2BodyDef centerRightTirebd;
        centerRightTirebd.type = b2_dynamicBody;
        centerRightTirebd.position.Set(6.0f, 3.5f);
        b2Body* centerRightTire = m_world->CreateBody(&centerRightTirebd);
        centerRightTire->CreateFixture(&centerLeftTirefd);	
        //Joint between center left tire and center left rod
        /*! \par centerLeftTireJoint
         * The revolute joint between centerLeftTire and centerLeft rod <br>
         * The local anchor of centerLeft rod is set at (0.0f,-6.5f) <br>
         * The motor of the joint is enabled and collideConnected is disabled <br>
         * The maximum motor torque that is allowed is 20000000.0f <br>
         * The motor speed is set to 3.0f <br>
         * This joint is created in m_world as centerBackJoint <br>
         */
        b2RevoluteJointDef centerLeftTirejoint;
        centerLeftTirejoint.Initialize(centerLeftTire,centerLeft,centerLeftTire->GetWorldCenter());
        centerLeftTirejoint.localAnchorB.Set(0.0f,-6.5f);
        centerLeftTirejoint.collideConnected = false;
        centerLeftTirejoint.enableMotor = true;
        centerLeftTirejoint.maxMotorTorque = 20000000.0f;
        //centerLeftTirejoint.motorSpeed = 3.0f;
        centerBackJoint=(b2RevoluteJoint*)m_world->CreateJoint(&centerLeftTirejoint);	
        //Joint between center right tire and center right rod
        /*! \par centerRightTireJoint
         * The revolute joint between centerRightTire and centerRight rod <br>
         * The local Anchor of centerRight rod is set at (0.0f, -6.5f) <br>
         * Similar to the centerLeftTireJoint the joint motor is enabled and collideConnected is disabled <br>
         * The maximum motor torque that is allowed is 20000000.0f <br>
         * It is created in m_world as centerFrontJoint <br>
         */
        b2RevoluteJointDef centerRightTirejoint;
        centerRightTirejoint.Initialize(centerRightTire,centerRight,centerRightTire->GetWorldCenter());
        centerRightTirejoint.localAnchorB.Set(0.0f,-6.5f);
        centerRightTirejoint.collideConnected = false;	
        centerRightTirejoint.enableMotor = true;
        centerRightTirejoint.maxMotorTorque = 20000000.0f;
        //centerRightTirejoint.motorSpeed = 3.0f;
        centerFrontJoint=(b2RevoluteJoint*)m_world->CreateJoint(&centerRightTirejoint);
        //main frame plate
        /*! \par mainFramePlate
         * It is the main frame plate of the bot <br>
         * The Shape of the plate is b2PolygonShape (Rectangle) and it is a box with length=21.0f and breadth=0.5f <br>
         * The position of the main plate is set at (-0.5f, 10.25f) <br>
         * The category bit of the frame is 0x0008 and the mask bit is 0x0010 <br>
         * The density is same as that of aluminium = 2700.0f <br>
         */
		b2PolygonShape mainFramePlate;
		mainFramePlate.SetAsBox(10.5f, 0.25f);
		b2BodyDef mainFramebd;
		mainFramebd.position.Set(-0.5f, 10.25f);
		mainFramebd.type = b2_dynamicBody;
		b2Body* mainFrame = m_world->CreateBody(&mainFramebd);
		b2FixtureDef mainFramefd;
		mainFramefd.filter.categoryBits = 0x0008;
		mainFramefd.filter.maskBits = 0x0010;
		mainFramefd.density = 2700.f;
		mainFramefd.shape = &mainFramePlate;
		mainFrame->CreateFixture(&mainFramefd);
		//main frame structure for attaching central bogeys
		/*! \par attachCenterFrameShape
		 * Main frame structure for attaching central bogeys <br>
		 * Category bit is 0x0008 and mask bit 0x0010 <br>
		 * The density is again 2700.0f <br>
		 * The length and breadth are 5.5f(vertical) and 3.0f(horizontal) respectively <br>
		 * The position is set to (0.0f, 13.25f) <br>
		 */
		b2PolygonShape attachCenterFrameshape;
		attachCenterFrameshape.SetAsBox(1.5f, 2.75f);
		b2BodyDef attachCenterFramebd;
		attachCenterFramebd.position.Set(0.0f, 13.25f);
		attachCenterFramebd.type = b2_dynamicBody;
		b2Body* attachCenterFrame = m_world->CreateBody(&attachCenterFramebd);
		b2FixtureDef attachCenterFramefd;
		attachCenterFramefd.filter.categoryBits = 0x0008;
		attachCenterFramefd.filter.maskBits = 0x0010;
		attachCenterFramefd.density = 2700.0f;
		attachCenterFramefd.shape = &attachCenterFrameshape;
		attachCenterFrame->CreateFixture(&attachCenterFramefd);
		//Joint for main frame and center top rod
		/*! \par mainTopJoint
		 * The revolute joint between attached centerFrame and centerTop rod <br>
		 * The local anchor of attach center frame is (0.0,1.75f) <br>
		 * The allowed rotation of the joint is (-0.6f,0.6f) <br>
		 * collideConnected is set to false so that the connected rods do not collide <br>
		 */
        b2RevoluteJointDef mainTopjoint;
		mainTopjoint.Initialize(attachCenterFrame,centerTop,centerTop->GetWorldCenter());
		mainTopjoint.localAnchorA.Set(0.0f,1.75f) ;
		mainTopjoint.localAnchorB.Set(0.0f,0.0f);
		mainTopjoint.enableLimit = true;
		mainTopjoint.upperAngle = 0.6f;
		mainTopjoint.lowerAngle = -0.6f;
		mainTopjoint.collideConnected = false;
		m_world->CreateJoint(&mainTopjoint);		
		//Joint for main frame and bottom rod 
		/*! \par mainBottomjoint
		 * The revolute joint between attachCenterFrame and centerBottom rod <br>
		 * The local anchor of attachCenterFrame is set at (0.0f,-1.75f) <br>
		 * collideConnected is set to false so that the connected rods do not collide <br>
		 */
		b2RevoluteJointDef mainBottomjoint;
		mainBottomjoint.Initialize(attachCenterFrame,centerBottom,centerBottom->GetWorldCenter());
		mainBottomjoint.localAnchorA.Set(0.0f,-1.75f) ;
		mainBottomjoint.localAnchorB.Set(0.0f,0.0f);
		mainBottomjoint.collideConnected = false;
		m_world->CreateJoint(&mainBottomjoint);		
		//Joint for main plate and main frame to attach center frame a weld joint
		/*! \par mainFrameCenterJoint
		 * It is weld joint between  attachCenterFrame and mainFrame <br>
		 * The local anchor of attachCenterFrame is (0.0f,-2,75f) <br>
		 * The local anchor of mainFrame is (0.5f, 0.25f) <br>
		 * collideConnected is set to false so that the connected rods do not collide <br>
		 */
		b2WeldJointDef mainFrameCenterJoint;
		mainFrameCenterJoint.bodyA=attachCenterFrame;
		mainFrameCenterJoint.bodyB=mainFrame;
		mainFrameCenterJoint.localAnchorA.Set(0.0f,-2.75f) ;
		mainFrameCenterJoint.localAnchorB.Set(0.5f,0.25f);
		mainFrameCenterJoint.collideConnected = false;
		m_world->CreateJoint(&mainFrameCenterJoint);
		//Attaching back fork
		/*! \par backFrameShape
		 * This is used to attch the back fork <br>
		 * It is set as a box of length 2.0f and breadth 4.5f <br>
		 * Its position is set at (-10.0f, 13.25f) <br>
		 * Its category bit is 0x0008 <br>
		 * The density is 10000.0f <br>
		 */
		b2PolygonShape backFrameShape;
		backFrameShape.SetAsBox(1.0f, 2.75f);
		b2BodyDef backFramebd;
		backFramebd.position.Set(-10.0f, 13.25f);
		backFramebd.type = b2_dynamicBody;
		b2Body* backFrame = m_world->CreateBody(&backFramebd);
		b2FixtureDef backFramefd;
		backFramefd.filter.categoryBits = 0x0008;
		backFramefd.density = 2700.0f;
		backFramefd.friction = 0.0f;
		backFramefd.shape = &backFrameShape;
		backFrame->CreateFixture(&backFramefd);
		//Joint for main plate and back frame to attach back frame a weld joint
		/*! \par mainFrameBackJoint
		 * It is weld joint between main plate and back frame to attach back frame <br>
		 * The local anchor of backFrame is (0.0f, 2.75f) <br>
		 * The local anchor of mainFrame is (-9.5f, 0.25f) <br>
		 * collideConnected is set to false so that the connected rods do not collide <br>
		 */
		b2WeldJointDef mainFrameBackJoint;
		mainFrameBackJoint.bodyA=backFrame;
		mainFrameBackJoint.bodyB=mainFrame;
		mainFrameBackJoint.localAnchorA.Set(0.0f, -2.75f) ;
		mainFrameBackJoint.localAnchorB.Set(-9.5f, 0.25f);
		mainFrameBackJoint.collideConnected = false;
		m_world->CreateJoint(&mainFrameBackJoint);
		//Back frame part 3
		/*! \par backPart3
		 * It is the slanted rod connecting the backFrame rod with the back horizontal rod <br>
		 * The rod is a b2PolygonShape and has 4 vertices and is a parallelogram <br>
		 * The position of the rod is (-10.0f, 15.0f) <br>
		 * The density of the rod is 2700.0f, it is frictionless and is perfectly elastic <br>
		 */
		b2Vec2 backPart3vertices[4];
		backPart3vertices[2].Set(1.0f, 1.0f);
		backPart3vertices[1].Set(1.0f, -1.0f);
		backPart3vertices[0].Set(-5.0f, -5.0f);
		backPart3vertices[3].Set(-5.0f, -3.0f);
		b2Body* backPart3;
		b2PolygonShape backPart3shape;
		backPart3shape.Set(backPart3vertices, count);
		b2FixtureDef backPart3fd;
		backPart3fd.shape = &backPart3shape;
		backPart3fd.density = 2700.0f;
		backPart3fd.friction = 0.0f;
		backPart3fd.restitution = 1.0f;
		b2BodyDef backPart3bd;
		backPart3bd.type = b2_dynamicBody;
		backPart3bd.position.Set(-10.0f, 15.0f);
		backPart3 = m_world->CreateBody(&backPart3bd);
		backPart3->CreateFixture(&backPart3fd);
		//Joint for backPart3 and backFrame
		/*! \par mainBackJoint
		 * The revolute joint between the backPart3 rod and the backFrame rod <br>
		 * Body A is backFrame and the local anchor is set at (0.0f, 1.75f) at the right end of the backPart3 rod <br>
		 * Body B is backPart3 and the local anchor is set at (0.0f, 0.0f) at the top end of the backFrame rod <br>
		 * The angle limit of the joint is (-0.1f, 0.1f) <br>
		 * collideConnected is set to false so that the two connected rods do not collide <br>
		 */
        b2RevoluteJointDef mainBackjoint;
		mainBackjoint.Initialize(backFrame,backPart3,backPart3->GetWorldCenter());
		mainBackjoint.enableLimit = true;
		mainBackjoint.upperAngle = 0.1f;
		mainBackjoint.lowerAngle = -0.1f;
		mainBackjoint.localAnchorA.Set(0.0f,1.75f) ;
		mainBackjoint.localAnchorB.Set(0.0f,0.0f);
		mainBackjoint.collideConnected = false;
		m_world->CreateJoint(&mainBackjoint);
		//Back Frame part 2	
		/*! \par backPart2
		 * The back horizontal rod
		 * The shape of the rod is a rectangle with dimensions 8.0f x 2.0f <br>
		 * The position of the rod is set at (-14.0f, 11.0f) <br>
		 * The density of the rod is 2700.0f, it is frictionless and is perfectly elastic <br>
		 */
		b2Vec2 backPart2vertices[4];
		backPart2vertices[2].Set(1.0f, 1.0f);
		backPart2vertices[1].Set(1.0f, -1.0f);
		backPart2vertices[0].Set(-7.0f, -1.0f);
		backPart2vertices[3].Set(-7.0f, 1.0f);
		b2Body* backPart2;
		b2PolygonShape backPart2shape;
		backPart2shape.Set(backPart2vertices, count);
		b2FixtureDef backPart2fd;
		backPart2fd.shape = &backPart2shape;
		backPart2fd.density = 2700.0f;
		backPart2fd.friction = 0.0f;
		backPart2fd.restitution = 1.0f;
		b2BodyDef backPart2bd;
		backPart2bd.type = b2_dynamicBody;
		backPart2bd.position.Set(-14.0f, 11.0f);
		backPart2 = m_world->CreateBody(&backPart2bd);
		backPart2->CreateFixture(&backPart2fd);
		//Joint for backPart2 and backPart3 a weld joint
		/*! \par Back23Joint
		 * The weld joint between backPart3 rod and backPart2 rod <br>
		 * Body A is backPart3 and the local anchor is set at (-4.0f, -4.0f) at the right end of the backPart2 rod <br>
		 * Body B is backPart2 and the local anchor is set at (0.0f, 0.0f) at the left end of the backPart3 rod <br>
		 * collideConnected is set to false so the the connected rods do not collide <br>
		 */
		b2WeldJointDef Back23Joint;
		Back23Joint.bodyA=backPart3;
		Back23Joint.bodyB=backPart2;
		Back23Joint.localAnchorA.Set(-4.0f,-4.0f) ;
		Back23Joint.localAnchorB.Set(0.0f,0.0f);
		Back23Joint.collideConnected = false;
		m_world->CreateJoint(&Back23Joint);
		//back frame Part 1
		/*! \par BackPart1
		 * The vertical rod connecting the backmost tire and the backPart2 <br>
		 * The shape of the rod is a rectangle with dimensions 2.0f x 9.0f <br>
		 * The position of the rod is set at (-20.0f, 11.0f) <br>
		 * The density of the rod is 2700.0f, it is frictionless and is perfectly elastic <br>
		 */
		b2Vec2 backPart1vertices[4];
		backPart1vertices[2].Set(1.0f, 1.0f);
		backPart1vertices[1].Set(1.0f, -8.0f);
		backPart1vertices[0].Set(-1.0f, -8.0f);
		backPart1vertices[3].Set(-1.0f, 1.0f);
		b2Body* backPart1;
		b2PolygonShape backPart1shape;
		backPart1shape.Set(backPart1vertices, count);
		b2FixtureDef backPart1fd;
		backPart1fd.shape = &backPart1shape;
		backPart1fd.density = 2700.0f;
		backPart1fd.friction = 0.0f;
		backPart1fd.restitution = 1.0f;
		b2BodyDef backPart1bd;
		backPart1bd.type = b2_dynamicBody;
		backPart1bd.position.Set(-20.0f, 11.0f);
		backPart1 = m_world->CreateBody(&backPart1bd);
		backPart1->CreateFixture(&backPart1fd);
		//Joint for backPart2 and backPart1 a weld joint
		/*! \par Back21Joint
		 * The revolute joint between the vertical rod connected to the backmost rod and the backPart2 rod
		 * Body A is backPart1 and the local anchor is set to (0.0f, 0.0f) at the right end point of the backPart2 rod <br>
		 * Body B is backPart2 and the local anchor is set to (-6.0f, 0.0f) at the top end of the backPart1 rod <br>
		 * collideConnected is set to false so that the two connected rods do not collide <br>
		 */
		b2WeldJointDef Back21Joint;
		Back21Joint.bodyA=backPart1;
		Back21Joint.bodyB=backPart2;
		Back21Joint.localAnchorA.Set(0.0f,0.0f) ;
		Back21Joint.localAnchorB.Set(-6.0f,0.0f);
		Back21Joint.collideConnected = false;
		m_world->CreateJoint(&Back21Joint);
		//Back tyre
		/*! \par backTire
		 * The backmost tire of the bot <br>
		 * It is a dynamic body <br>
		 * The position of the tire is set at (-20.0f, 3.5f) <br>
		 * The fixture definitions is same as that of centerLeftTire <br>
		 */
		b2BodyDef backTirebd;
        backTirebd.type = b2_dynamicBody;
        backTirebd.position.Set(-20.0f, 3.5f);
        b2Body* backTire = m_world->CreateBody(&backTirebd);
        backTire->CreateFixture(&centerLeftTirefd);	
		//Joint between back tire and back frame part 1
		/*! \par backTireJoint
		 * The revolute joint between the backmost tire and the backPart1 rod <br>
		 * The bodies are connected at the center of the backTire <br>
		 * The local anchor point of backPart1 rod is (0.0f, -7.5f) <br>
		 * Motor has been used in this joint with maximum torque of 20000000.0f and speed of 3.0f <br>
		 * collideConnected is set to false so the the tire and the connected rode do n ot collide <br>
		 */
        b2RevoluteJointDef backTirejoint;
        backTirejoint.Initialize(backTire,backPart1,backTire->GetWorldCenter());
        backTirejoint.localAnchorB.Set(0.0f,-7.5f);
        backTirejoint.collideConnected = false;	
        backTirejoint.enableMotor = true;
        backTirejoint.maxMotorTorque = 20000000.0f;
        //backTirejoint.motorSpeed = 3.0f;
        backJoint=(b2RevoluteJoint*)m_world->CreateJoint(&backTirejoint);
        //Box to join front fork and main plate
        /*! \par frontFrameshape
         * This is the box used to connect the main plate to the front fork <br>
         * The box is shaped like a rectangular box with dimensions 5.0f x 5.5f <br>
         * The box is set at position (7.5f, 13.25f) <br>
         * The category bits are 0x0008 and the mask bits are 0x0010 <br>
         * The density of the box is same as that of aluminium = 2700.0f <br>
         */
        b2PolygonShape frontFrameshape;
		frontFrameshape.SetAsBox(2.5f, 2.75f);
		b2BodyDef frontFramebd;
		frontFramebd.position.Set(7.5f, 13.25f);
		frontFramebd.type = b2_dynamicBody;
		b2Body* frontFrame = m_world->CreateBody(&frontFramebd);
		b2FixtureDef frontFramefd;
		frontFramefd.filter.categoryBits = 0x0008;
		frontFramefd.filter.maskBits = 0x0010;
		frontFramefd.density = 2700.0f;
		frontFramefd.shape = &frontFrameshape;
		frontFrame->CreateFixture(&frontFramefd);
		//Joint for main plate and front frame to attach front fork a weld joint
		/*! \par mainFrameFrontJoint
		 * The weld joint between thw main plate and the front frame used to attach front forks <br>
		 * Body A is frontFrame and the local anchor is set at (0.0f, -2.75f) at the right end of the mainFrame <br>
		 * Body B is mainFrame and the local anchor is set at (8.0f, 0.25f) at the bottom end of the frontFrame rod <br>
		 * collideConnected is set to false so that the connected rods do not collide <br>
		 */
		b2WeldJointDef mainFrameFrontJoint;
		mainFrameFrontJoint.bodyA=frontFrame;
		mainFrameFrontJoint.bodyB=mainFrame;
		mainFrameFrontJoint.localAnchorA.Set(0.0f,-2.75f) ;
		mainFrameFrontJoint.localAnchorB.Set(8.0f,0.25f);
		mainFrameFrontJoint.collideConnected = false;
		m_world->CreateJoint(&mainFrameFrontJoint);
		//Front frame upper fork
		/*! \par frontPartUpperFork
		 * It is the upper fork connected to the frontFrame rod <br>
		 * It is a b2PolygonShape with 4 vertices (a parallelogram) <br>
		 * The position of the fork is at (6.5f, 14.0f) <br>
		 * The category bits are 0x0008 and the mask bits are 0x0010 <br>
		 * The density of rod is 2700.0f, it is frictionless and is perfectly elastic <br>
		 */
		b2Vec2 frontPartUpperForkvertices[4];
		frontPartUpperForkvertices[2].Set(-1.f, 1.f);
		frontPartUpperForkvertices[1].Set(-1.f, -1.0f);
		frontPartUpperForkvertices[0].Set(7.f, 6.5f);
		frontPartUpperForkvertices[3].Set(7.f, 8.5f);
		b2Body* frontPartUpperFork;
		b2PolygonShape frontPartUpperForkshape;
		frontPartUpperForkshape.Set(frontPartUpperForkvertices, count);
		b2FixtureDef frontPartUpperForkfd;
		frontPartUpperForkfd.shape = &frontPartUpperForkshape;
		frontPartUpperForkfd.density = 2700.0f;
		frontPartUpperForkfd.filter.categoryBits = 0x0008;
		frontPartUpperForkfd.filter.maskBits = 0x0010;
		frontPartUpperForkfd.friction = 0.0f;
		frontPartUpperForkfd.restitution = 1.0f;
		b2BodyDef frontPartUpperForkbd;
		frontPartUpperForkbd.type = b2_dynamicBody;
		frontPartUpperForkbd.position.Set(6.5f, 14.0f);
		frontPartUpperFork = m_world->CreateBody(&frontPartUpperForkbd);
		frontPartUpperFork->CreateFixture(&frontPartUpperForkfd);
		//Joint between front upper fork and front plate
		/*! \par frontUpperForkJoint
		 * The revolute joint between the upper fork and the front plate <br>
		 * Body A is frontPartUpperFork and the local anchor is at (0.0f, 0.0f) at the left top point of frontFrame rod <br>
		 * Body B is frontFrame and the local anchor is at (-1.0f, 0.75f) at the left end point of the frontPartUpperFork rod <br>
		 * The rotation angle has been limited to (-1.0f, 1.0f) <br>
		 * collideConnected has been set to false so that the two rods do not collide <br>
		 */
        b2RevoluteJointDef frontUpperForkjoint;
        frontUpperForkjoint.bodyA=frontPartUpperFork,
        frontUpperForkjoint.bodyB=frontFrame;
        frontUpperForkjoint.localAnchorA.Set(0.0f,0.0f);
        frontUpperForkjoint.localAnchorB.Set(-1.0f,0.75f);
        frontUpperForkjoint.collideConnected = false;	
        frontUpperForkjoint.enableLimit = true;
		frontUpperForkjoint.upperAngle = 1.0f;
		frontUpperForkjoint.lowerAngle = -1.0f;
        m_world->CreateJoint(&frontUpperForkjoint);
        //Front frame part 3
        /*! \par frontPart3
		 * It is the slanted rod connecting the frontPartUpperFork rod with the front horizontal rod <br>
		 * The rod is a b2PolygonShape and has 4 vertices and is a parallelogram <br>
		 * The position of the rod is (12.5f, 21.5f) <br>
		 * The density of the rod is 2700.0f, it is frictionless and is perfectly elastic <br>
		 * The category bits are 0x0008 <br>
		 */
		b2Vec2 frontPart3vertices[4];
		frontPart3vertices[2].Set(-1.f, 1.f);
		frontPart3vertices[1].Set(1.f, 1.0f);
		frontPart3vertices[0].Set(8.f, -13.f);
		frontPart3vertices[3].Set(6.f, -13.f);
		b2Body* frontPart3;
		b2PolygonShape frontPart3shape;
		frontPart3shape.Set(frontPart3vertices, count);
		b2FixtureDef frontPart3fd;
		frontPart3fd.shape = &frontPart3shape;
		frontPart3fd.density = 2700.0f;
		frontPart3fd.filter.categoryBits = 0x0008;
		frontPart3fd.friction = 0.0f;
		frontPart3fd.restitution = 1.0f;
		b2BodyDef frontPart3bd;
		frontPart3bd.type = b2_dynamicBody;
		frontPart3bd.position.Set(12.5f, 21.5f);
		frontPart3 = m_world->CreateBody(&frontPart3bd);
		frontPart3->CreateFixture(&frontPart3fd);
		//Joint between front upper fork and front frame part 3
		/*! \par frontUpperForkPart3joint
		 * The revolute joint between the front Upper fork and the frontPart3 rod <br>
		 * Body A is frontPartUpperFork and the local anchor is set at (6.0f, 7.0f) at the left top end of the frontPart3 rod <br>
		 * Body B is frontPart3 and the local anchor is set at (0.0f, 0.0f) at the right end point of the frontPartUpperFork rod <br>
		 * collideConnected is set to false so that the two roda do not collide <br>
		 */
        b2RevoluteJointDef frontUpperForkPart3joint;
        frontUpperForkPart3joint.bodyA=frontPartUpperFork,
        frontUpperForkPart3joint.bodyB=frontPart3;
        frontUpperForkPart3joint.localAnchorA.Set(6.0f,7.0f);
        frontUpperForkPart3joint.localAnchorB.Set(0.0f,0.0f);
        frontUpperForkPart3joint.collideConnected = false;	
        m_world->CreateJoint(&frontUpperForkPart3joint);
        //Front frame lower fork
        /*! \par frontPartLowerFork
         * It is the lower fork connected to the frontFrame rod <br>
		 * It is a b2PolygonShape with 4 vertices (a parallelogram) <br>
		 * The position of the fork is at (9.0f, 12.0f) <br>
		 * The category bits are 0x0010 and the mask bits are 0x0008 <br>
		 * The density of rod is 2700.0f, it is frictionless and is perfectly elastic <br>
		 */
		b2Vec2 frontPartLowerForkvertices[4];
		frontPartLowerForkvertices[2].Set(-1.0f, 1.0f);
		frontPartLowerForkvertices[1].Set(-1.0f, 0.0f);
		frontPartLowerForkvertices[0].Set(6.5f, 4.5f);
		frontPartLowerForkvertices[3].Set(6.5f, 6.5f);
		b2Body* frontPartLowerFork;
		b2PolygonShape frontPartLowerForkshape;
		frontPartLowerForkshape.Set(frontPartLowerForkvertices, count);
		b2FixtureDef frontPartLowerForkfd;
		frontPartLowerForkfd.shape = &frontPartLowerForkshape;
		frontPartLowerForkfd.density = 2700.0f;
		frontPartLowerForkfd.filter.categoryBits = 0x0010;
		frontPartLowerForkfd.filter.maskBits = 0x0008;
		frontPartLowerForkfd.friction = 0.0f;
		frontPartLowerForkfd.restitution = 1.0f;
		b2BodyDef frontPartLowerForkbd;
		frontPartLowerForkbd.type = b2_dynamicBody;
		frontPartLowerForkbd.position.Set(9.0f, 12.0f);
		frontPartLowerFork = m_world->CreateBody(&frontPartLowerForkbd);
		frontPartLowerFork->CreateFixture(&frontPartLowerForkfd);
		//Joint between front lower fork and front plate
		/*! \par frontLowerForkJoint
		 * The revolute joint between frontPartLowerFork and the frontFrame rods <br>
		 * Body A is frontPartLowerFork and the local anchor point is (0.0f, 0.0f) at around the center of the frontFrame rod <br>
		 * Body B is frontFrame and the local anchor point is (1.5f, -0.75f) at the left end of the frontPartLowerFork <br>
		 * collideConnected is set to false so that the two connected rods do not collide <br>
		 */
        b2RevoluteJointDef frontLowerForkjoint;
        frontLowerForkjoint.bodyA=frontPartLowerFork,
        frontLowerForkjoint.bodyB=frontFrame;
        frontLowerForkjoint.localAnchorA.Set(0.0f,0.0f);
        frontLowerForkjoint.localAnchorB.Set(1.5f,-0.75f);
        frontLowerForkjoint.collideConnected = false;	
        m_world->CreateJoint(&frontLowerForkjoint);
        //Joint between front lower fork and front frame part 3
        /*! \par frontLowerForkPart3joint
         * The revolute joint between the lower fork and the frontPart3 rod <br>
         * Body A is frontPartLowerFork and the local anchor point is (5.5f, 5.0f) at around the center of the frontPart3 rod <br>
         * Body B is frontPart3 and the local anchor point is (2.0f, -4.5f) at the right end point of the frontPartLowerFork rod <br>
         * collideConnected is set to false so that the two connected rods do not collide <br>
         */
        b2RevoluteJointDef frontLowerForkPart3joint;
        frontLowerForkPart3joint.bodyA=frontPartLowerFork,
        frontLowerForkPart3joint.bodyB=frontPart3;
        frontLowerForkPart3joint.localAnchorA.Set(5.5f,5.0f);
        frontLowerForkPart3joint.localAnchorB.Set(2.0f,-4.5f);
        frontLowerForkPart3joint.collideConnected = false;	
        m_world->CreateJoint(&frontLowerForkPart3joint);
        //Spring
        /*! \par SpringJoint
         * This is a distance joint created between the frontPartLowerFork and the frontPartUpperFork rod <br>
         * Body A is frontPartLowerFork and the local anchor is at (2.75f, 2.75f) <br>
         * Body B is frontPartUpperFork and the local anchor is at (3.0f, 3.75f) <br>
         * The freaquency of the joint is 2.0f and the damping ratio is 0.1f <br>
         * collideCOnnected is set to false so that the two rods do not collide <br>
         */
        b2DistanceJointDef Springjoint;
        Springjoint.bodyA=frontPartLowerFork,
        Springjoint.bodyB=frontPartUpperFork;
        Springjoint.localAnchorA.Set(2.75f,2.75f);
        Springjoint.localAnchorB.Set(3.0f,3.75f);
        Springjoint.collideConnected = false;
        Springjoint.frequencyHz = 2.0f;
        Springjoint.dampingRatio = 0.1f;	
        m_world->CreateJoint(&Springjoint);
        //front frame part 2 
        /*! \par frontPart2
         * The horizontal rod connected to the vertical rod connected to the frontmost tyre <br>
         * It has a b2PolygonShape with 4 vertices and is a rectangle <br>
         * The position of this rod is at (19.5f, 9.5f) <br>
         * The density of the rod is 2700.0f, it is frictionless and is perfectly elastic <br>
         */
        b2Vec2 frontPart2vertices[4];
		frontPart2vertices[2].Set(-1.f, 1.f);
		frontPart2vertices[1].Set(-1.f, -1.f);
		frontPart2vertices[0].Set(3.5f, -1.f);
		frontPart2vertices[3].Set(3.5f, 1.f);
		b2Body* frontPart2;
		b2PolygonShape frontPart2shape;
		frontPart2shape.Set(frontPart2vertices, count);
		b2FixtureDef frontPart2fd;
		frontPart2fd.shape = &frontPart2shape;
		frontPart2fd.density = 2700.0f;
		frontPart2fd.friction = 0.0f;
		frontPart2fd.restitution = 1.0f;
		b2BodyDef frontPart2bd;
		frontPart2bd.type = b2_dynamicBody;
		frontPart2bd.position.Set(19.5f, 9.5f);
		frontPart2 = m_world->CreateBody(&frontPart2bd);
		frontPart2->CreateFixture(&frontPart2fd);
		//Joining front part 2 and front part 3
		/*! \par front23Joint
		 * The weld joint between frontPart3 rod and frontPart2 rod <br>
		 * Body A is frontPart3 and the local anchor is set at (7.0f, -12.0f) at the left end of the backPart2 rod <br>
		 * Body B is frontPart2 and the local anchor is set at (0.0f, 0.0f) at the right end of the backPart3 rod <br>
		 * collideConnected is set to false so the the connected rods do not collide <br>
		 */
		b2WeldJointDef front23Joint;
		front23Joint.bodyA=frontPart3;
		front23Joint.bodyB=frontPart2;
		front23Joint.localAnchorA.Set(7.0f,-12.0f) ;
		front23Joint.localAnchorB.Set(0.0f,0.0f);
		front23Joint.collideConnected = false;
		m_world->CreateJoint(&front23Joint);
		//front frame part 1
		/*! \par frontPart1
		 * This is the vertical rod connected to the frontmost tyre <br>
		 * It has a b2PolygonShape with 4 vertices and is a rectangle <br>
		 * The position of the rod is set at (22.0f, 9.5f) <br>
		 * The density of the rod is 2700.0f, it is frictionless and is perfectly elastic <br>
		 */
		b2Vec2 frontPart1vertices[4];
		frontPart1vertices[2].Set(1.f, 1.f);
		frontPart1vertices[1].Set(1.f, -6.5f);
		frontPart1vertices[0].Set(-1.f, -6.5f);
		frontPart1vertices[3].Set(-1.f, 1.f);
		b2Body* frontPart1;
		b2PolygonShape frontPart1shape;
		frontPart1shape.Set(frontPart1vertices, count);
		b2FixtureDef frontPart1fd;
		frontPart1fd.shape = &frontPart1shape;
		frontPart1fd.density = 2700.0f;
		frontPart1fd.friction = 0.0f;
		frontPart1fd.restitution = 1.0f;
		b2BodyDef frontPart1bd;
		frontPart1bd.type = b2_dynamicBody;
		frontPart1bd.position.Set(22.0f, 9.5f);
		frontPart1 = m_world->CreateBody(&frontPart1bd);
		frontPart1->CreateFixture(&frontPart1fd);
		//Joining front part 2 and front part 1
		/*! \par front21Joint
		 * The weld joint between the vertical rod connected to the frontmost tyre and the frontPart2 rod
		 * Body A is frontPart1 and the local anchor is set to (0.0f, 0.0f) at the right end point of the frontPart2 rod <br>
		 * Body B is frontPart2 and the local anchor is set to (2.5f, 0.0f) at the top end of the frontPart1 rod <br>
		 * collideConnected is set to false so that the two connected rods do not collide <br>
		 */
		b2WeldJointDef front21Joint;
		front21Joint.bodyA=frontPart1;
		front21Joint.bodyB=frontPart2;
		front21Joint.localAnchorA.Set(0.0f,0.0f) ;
		front21Joint.localAnchorB.Set(2.5f,0.0f);
		front21Joint.collideConnected = false;
		m_world->CreateJoint(&front21Joint);
		//front tyre
		/*! \par frontTire
		 * The frontmost tire of the bot <br>
		 * It is a dynamic body <br>
		 * The position of the tire is set at (22.0f, 3.5f) <br>
		 * The fixture definitions is same as that of centerLeftTire <br>
		 */
		b2BodyDef frontTirebd;
        frontTirebd.type = b2_dynamicBody;
        frontTirebd.position.Set(22.0f, 3.5f);
        b2Body* frontTire = m_world->CreateBody(&frontTirebd);
        frontTire->CreateFixture(&centerLeftTirefd);	
		//Joint between front tire and front frame part 1
		/*! \par frontTireJoint
		 * The revolute joint between the frontmost tire and the frontPart1 rod <br>
		 * The bodies are connected at the center of the frontTire <br>
		 * The local anchor point of frontPart1 rod is (0.0f, -7.5f) <br>
		 * Motor has been used in this joint with maximum torque of 20000000.0f and speed of 3.0f <br>
		 * collideConnected is set to false so the the tire and the connected rode do not collide <br>
		 */
        b2RevoluteJointDef frontTirejoint;
        frontTirejoint.Initialize(frontTire,frontPart1,frontTire->GetWorldCenter());
        frontTirejoint.localAnchorB.Set(0.0f,-6.0f);
        frontTirejoint.collideConnected = false;	
        frontTirejoint.enableMotor = true;
        frontTirejoint.maxMotorTorque = 20000000.0f;
        //frontTirejoint.motorSpeed = 3.0f;
        frontJoint = (b2RevoluteJoint*)m_world->CreateJoint(&frontTirejoint);
	}
	//Terrain
	/*! \par Terrains
	 * Terrains for testing the bot
	 * Terrains have slopes, straight roads etc. and they are all made from b2Polygonshape with different number of vertices <br>
	 * They all have friction=1.0f and are perfectly elastic bodies <br>
	 */
    {   	
		b2Vec2 Terrain1vertices[5];
		Terrain1vertices[0].Set(0.f, 0.f);
		Terrain1vertices[1].Set(50.f, 30.0f);
		Terrain1vertices[2].Set(120.f, 30.f);
		Terrain1vertices[3].Set(120.f, 0.f);
		Terrain1vertices[4].Set(0.f, 0.f);
		b2Body* Terrain1;
		b2PolygonShape Terrain1shape;
		Terrain1shape.Set(Terrain1vertices, 5);
		b2FixtureDef Terrain1fd;
		Terrain1fd.shape = &Terrain1shape;
		Terrain1fd.friction = 1.0f;
		Terrain1fd.restitution = 1.0f;
		b2BodyDef Terrain1bd;
		Terrain1bd.position.Set(70.f, 0.f);
		Terrain1 = m_world->CreateBody(&Terrain1bd);
		Terrain1->CreateFixture(&Terrain1fd);
		
		b2Vec2 Terrain2vertices[5];
		Terrain2vertices[0].Set(0.f, 0.f);
		Terrain2vertices[1].Set(0.f, 35.0f);
		Terrain2vertices[2].Set(60.f, 60.f);
		Terrain2vertices[3].Set(130.f, 60.f);
		Terrain2vertices[4].Set(230.f, 0.f);
		b2Body* Terrain2;
		b2PolygonShape Terrain2shape;
		Terrain2shape.Set(Terrain2vertices, 5);
		b2FixtureDef Terrain2fd;
		Terrain2fd.shape = &Terrain2shape;
		Terrain2fd.friction = 1.0f;
		Terrain2fd.restitution = 1.0f;
		b2BodyDef Terrain2bd;
		Terrain2bd.position.Set(190.f, 0.f);
		Terrain2 = m_world->CreateBody(&Terrain2bd);
		Terrain2->CreateFixture(&Terrain2fd);
		
		b2Vec2 Terrain3vertices[5];
		Terrain3vertices[0].Set(0.f, 0.f);
		Terrain3vertices[1].Set(100.f, 40.0f);
		Terrain3vertices[2].Set(200.f, 60.f);
		Terrain3vertices[3].Set(330.f, 30.f);
		Terrain3vertices[4].Set(430.f, 0.f);
		b2Body* Terrain3;
		b2PolygonShape Terrain3shape;
		Terrain3shape.Set(Terrain3vertices, 5);
		b2FixtureDef Terrain3fd;
		Terrain3fd.shape = &Terrain3shape;
		Terrain3fd.friction = 1.0f;
		Terrain3fd.restitution = 1.0f;
		b2BodyDef Terrain3bd;
		Terrain3bd.position.Set(420.f, 0.f);
		Terrain3 = m_world->CreateBody(&Terrain3bd);
		Terrain3->CreateFixture(&Terrain3fd);

    }
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}


