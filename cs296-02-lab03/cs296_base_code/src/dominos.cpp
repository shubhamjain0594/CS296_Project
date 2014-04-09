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
	b2RevoluteJoint* backJoint;/*!< The revolute joint between centre front tyre and rod connected to it */
	b2RevoluteJoint* centerBackJoint;/*!< The revolute joint between centre back tyre and rod connected to it */
	b2RevoluteJoint* centerFrontJoint;/*!< The revolute joint between backmost tyre and rod connected to it */
	///The inputs taken from the keyboard
 	void dominos_t::keyboard(unsigned char key)
	{
		switch(key)
		{
			/**When 'd' key is pressed the speed of the all the revolute joints of tyres increases by 0.1f for each press untill it becomes 3.f*/
			case('d'):
				if(i <3){
				i = i+0.1f;
				}
				frontJoint->SetMotorSpeed(i);
				backJoint->SetMotorSpeed(i);
				centerFrontJoint->SetMotorSpeed(i);
				centerBackJoint->SetMotorSpeed(i);
				break;
			/**Similarly when 'a' key is pressed the speed of the all the revolute joints of tyres decreases by 0.1f for each press untill it becomes -3.f*/
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
	 * Edge is set between two points on x-axis x1=-1800 and x2=1800. <br>
	 * bd is a Box2D body definition.<br>
	 * b1 is then pointed to a body created with body definition bd<br>
	 * A fixture is created on the body with Edge shape created earlier and 0 density
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
		 * Center Horizontal top rod
		 * Shape: b2PolygonShape
		 * A box of length 14.f and breadth 2.f
		 * Position x=0,y=15.f
		 * It is a dynamic body
		 * categoryBit = 0x0002
		 * density=2700.f
		 */
		b2PolygonShape centerTopShape;
		centerTopShape.SetAsBox(7.0f, 1.f);
		b2BodyDef centerTopbd;
		centerTopbd.position.Set(0.0f, 15.f);
		centerTopbd.type = b2_dynamicBody;
		b2Body* centerTop = m_world->CreateBody(&centerTopbd);
		b2FixtureDef centerTopfd;
		centerTopfd.filter.categoryBits = 0x0002;
		centerTopfd.density = 2700.0f;
		centerTopfd.shape = &centerTopShape;
		centerTop->CreateFixture(&centerTopfd);
		//Center Horizontal bottom rod
		/*! \par centerBottom
		 * Position x=0, y=11.f 
		 * It is a dynamic body
		 * body definition same as centerTop
		 */
		b2BodyDef centerBottombd;
		centerBottombd.position.Set(0.0f, 11.f);
		centerBottombd.type = b2_dynamicBody;
		b2Body* centerBottom = m_world->CreateBody(&centerBottombd);
		centerBottom->CreateFixture(&centerTopfd);
		//Center vertical left rod
		/*! \par centerLeft
		 * Center vertical left rod
		 * A polygon with four vertices around a point near the horizontal bottom rod
		 * The position of its representative point is x=-6, y=10
		 * Category bit=0x0002
		 * Density of the rod is same as alumninium = 2700.f
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
		 * This is also a dynamic body
		 * The fixture definition is same to center vertical left rod
		 * The position is set at x=6, y=10
		 */
		b2BodyDef centerRightbd;
		centerRightbd.position.Set(6.0f, 10.0f);
		centerRightbd.type = b2_dynamicBody;
		b2Body* centerRight = m_world->CreateBody(&centerRightbd);
		centerRight->CreateFixture(&centerLeftfd);
		//Joint between center left and center top  rod
		/*! \par centerLeftTopJoint
		 * A Revolute joint between center left and center top rod
		 * At a position x=-6 and y=15
		 * Body A is centerTop and its local anchor with respect to its representative point is (-6,0)
		 * Body B is centerLeft and its local anchor is set at (0,5)
		 * collideConneced Boolean is set to false so that the two rods do not collide
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
         * This is also a revolute joint between center right and center top joint
         * Body a is centerTop and its local anchor is set at (6,0)
         * Body B is centerRight and its local Anchor is set at (0,5) at the end point of centerTop rod
         * collideConneced Boolean is set to false so that the two rods do not collide
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
         * This is a revolute joint between center left and center bottom rod
         * It is made at an initial position of (-6,11)
         * Body A=centerBottom, localAnchor=(-6,0)
         * Body B=centerLeft, localAnchor=(0,1)
         * collideConnected is false and hence the two bodies do not collide 
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
         * Position is set at (6,11)
         * Body A is centerBottom and its local Anchor is set at (6,0)
         * Body B is centerRight and its local anchor is set at (0,1)
         * Again the collideConnected boolean is false so that the two rods do not collide 
         */
		b2RevoluteJointDef centerRightBottomJoint;
        b2Vec2 centerRightBottomJointv1(6.0f,11.0f);
        centerRightBottomJoint.Initialize(centerBottom,centerRight,centerRightBottomJointv1);
        centerRightBottomJoint.localAnchorA.Set(6.0f,0.0f) ;
        centerRightBottomJoint.localAnchorB.Set(0.0f,1.0f);
        centerRightBottomJoint.collideConnected = false;
        m_world->CreateJoint(&centerRightBottomJoint);	
        //Center left tyre 
        /*! \par centerLeftTire and the centerRightTire
         *  Radius of the tyres is 3.5f
         * Friction of the tyre is 1.f
         * The coefficient of restitution is also 1.f for possibility of elastic collisions
         * The position of centerLeftTire is (-6,3.5) and centerRightTire is (6,3.5)
         */
        b2CircleShape centerLeftcircle;
        centerLeftcircle.m_radius = 3.5f;
        b2FixtureDef centerLeftTirefd;
        centerLeftTirefd.shape = &centerLeftcircle;
        centerLeftTirefd.density = 3000.0f;
        centerLeftTirefd.friction = 1.f;
        centerLeftTirefd.restitution = 1.f;
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
         * A revolute joint between centerLeftTire and centerLeft rod
         * The local anchor of centerLeft rod is at (0,-6.5f)
         * The motor of the joint is enabled and collideConnected is disabled
         * The maximum motor torque that is allowed is 20000000.0f
         * This joint is created in m_world as centerBackJoint
         */
        b2RevoluteJointDef centerLeftTirejoint;
        centerLeftTirejoint.Initialize(centerLeftTire,centerLeft,centerLeftTire->GetWorldCenter());
        centerLeftTirejoint.localAnchorB.Set(0.0f,-6.5f);
        centerLeftTirejoint.collideConnected = false;
        centerLeftTirejoint.enableMotor = true;
        centerLeftTirejoint.maxMotorTorque = 20000000.0f;
        centerLeftTirejoint.motorSpeed = 3.0f;
        centerBackJoint=(b2RevoluteJoint*)m_world->CreateJoint(&centerLeftTirejoint);	
        //Joint between center right tire and center right rod
        /*! \par centerRightTireJoint
         * A revolute joint between centerRightTire and centerRight
         * The local Anchor of centerRight rod is set at (0,-6.5f)
         * Similar to the centerLeftTireJoint the joint motor is enabled and collideConnected is disabled
         * The maximum motor torque that is allowed is 20000000.0f
         * It is created as centerFrontJoint in m_world
         */
        b2RevoluteJointDef centerRightTirejoint;
        centerRightTirejoint.Initialize(centerRightTire,centerRight,centerRightTire->GetWorldCenter());
        centerRightTirejoint.localAnchorB.Set(0.0f,-6.5f);
        centerRightTirejoint.collideConnected = false;	
        centerRightTirejoint.enableMotor = true;
        centerRightTirejoint.maxMotorTorque = 20000000.0f;
        centerRightTirejoint.motorSpeed = 3.0f;
        centerFrontJoint=(b2RevoluteJoint*)m_world->CreateJoint(&centerRightTirejoint);
        //main frame plate
        /*! \par mainFramePlate
         * It is the main frame plate of the bot
         * The Shape of the plate is b2PolygonShape and it is a box with length=21 and breadth=1
         * The category bit of the frame is 0x0008 and the mask bit is 0x0010 
         * The density is same as that of aluminium = 2700  
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
		 * Main frame structure for attaching central bogeys
		 * Category bit is 0x0008 and mask bit 0x0010
		 * The density is again 2700
		 * The length and breadth are 4.5(vertical) and 3(horizontal) respectively
		 * The position is set to (0,13.25)
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
		attachCenterFramefd.density = 2700.f;
		attachCenterFramefd.shape = &attachCenterFrameshape;
		attachCenterFrame->CreateFixture(&attachCenterFramefd);
		//Joint for main frame and center top rod
		/*! \par mainTopJoint
		 * This is a revolute joint between attach centerFrame and centerTop
		 * The local anchor of attach center frame is (0,1.75)
		 * The allowed rotation of he joint is (-0.6f,0.6f)
		 * collideConnected is disabled
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
		 * This is a revolute joint between attachCenterFrame and centerBottom
		 * The local anchor of attachCenterFrame is set at (0,-1.75f)
		 * collideConnected is disabled
		 */
		b2RevoluteJointDef mainBottomjoint;
		mainBottomjoint.Initialize(attachCenterFrame,centerBottom,centerBottom->GetWorldCenter());
		mainBottomjoint.localAnchorA.Set(0.0f,-1.75f) ;
		mainBottomjoint.localAnchorB.Set(0.0f,0.0f);
		mainBottomjoint.collideConnected = false;
		m_world->CreateJoint(&mainBottomjoint);		
		//Joint for main plate and main frame to attach center frame a weld joint
		/*! \par mainFameCenterJoint
		 * It is weld joint between  attachCenterFrame and mainFrame
		 * The local anchor of attachCenterFrame is (0,-2,75)
		 * The local anchor of mainFrame is (0.5,0.25)
		 * collideConnected is disabled
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
		 * This is used to attch the back fork
		 * It is set as a box of horizontal length 2 and verical length 4.5f
		 * Its position is set at (-10,13.25)
		 * Its category bit is 0x0008
		 * The density is 10000 
		 */
		b2PolygonShape backFrameShape;
		backFrameShape.SetAsBox(1.f, 2.75f);
		b2BodyDef backFramebd;
		backFramebd.position.Set(-10.f, 13.25f);
		backFramebd.type = b2_dynamicBody;
		b2Body* backFrame = m_world->CreateBody(&backFramebd);
		b2FixtureDef backFramefd;
		backFramefd.filter.categoryBits = 0x0008;
		backFramefd.density = 2700.f;
		backFramefd.friction = 0.0f;
		backFramefd.shape = &backFrameShape;
		backFrame->CreateFixture(&backFramefd);
		//Joint for main plate and back frame to attach back frame a weld joint
		/*! \par mainFrameBackJoint
		 * It is weld joint between main plate and back frame to attch back frame
		 * The local anchor of backFrame is (0,2.75)
		 * The local anchor of mainFrame is (-9.5,0.25)
		 * collideConnected is disabled
		 */
		b2WeldJointDef mainFrameBackJoint;
		mainFrameBackJoint.bodyA=backFrame;
		mainFrameBackJoint.bodyB=mainFrame;
		mainFrameBackJoint.localAnchorA.Set(0.0f,-2.75f) ;
		mainFrameBackJoint.localAnchorB.Set(-9.5f,0.25f);
		mainFrameBackJoint.collideConnected = false;
		m_world->CreateJoint(&mainFrameBackJoint);
		//Back frame part 3
		/*! \par backPart3
		 * 
		 */
		b2Vec2 backPart3vertices[4];
		backPart3vertices[2].Set(1.f, 1.f);
		backPart3vertices[1].Set(1.f, -1.0f);
		backPart3vertices[0].Set(-5.f, -5.0f);
		backPart3vertices[3].Set(-5.f, -3.f);
		b2Body* backPart3;
		b2PolygonShape backPart3shape;
		backPart3shape.Set(backPart3vertices, count);
		b2FixtureDef backPart3fd;
		backPart3fd.shape = &backPart3shape;
		backPart3fd.density = 2700.0f;
		backPart3fd.friction = 0.0f;
		backPart3fd.restitution = 1.f;
		b2BodyDef backPart3bd;
		backPart3bd.type = b2_dynamicBody;
		backPart3bd.position.Set(-10.0f, 15.0f);
		backPart3 = m_world->CreateBody(&backPart3bd);
		backPart3->CreateFixture(&backPart3fd);
		//Joint for backPart3 and backFrame
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
		b2Vec2 backPart2vertices[4];
		backPart2vertices[2].Set(1.f, 1.f);
		backPart2vertices[1].Set(1.f, -1.f);
		backPart2vertices[0].Set(-7.f, -1.f);
		backPart2vertices[3].Set(-7.f, 1.f);
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
		backPart2bd.position.Set(-14.0f, 11.f);
		backPart2 = m_world->CreateBody(&backPart2bd);
		backPart2->CreateFixture(&backPart2fd);
		//Joint for backPart2 and backPart3 a weld joint
		b2WeldJointDef Back23Joint;
		Back23Joint.bodyA=backPart3;
		Back23Joint.bodyB=backPart2;
		Back23Joint.localAnchorA.Set(-4.0f,-4.0f) ;
		Back23Joint.localAnchorB.Set(0.0f,0.0f);
		Back23Joint.collideConnected = false;
		m_world->CreateJoint(&Back23Joint);
		//back frame Part 1
		b2Vec2 backPart1vertices[4];
		backPart1vertices[2].Set(1.f, 1.f);
		backPart1vertices[1].Set(1.f, -8.f);
		backPart1vertices[0].Set(-1.f, -8.f);
		backPart1vertices[3].Set(-1.f, 1.f);
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
		backPart1bd.position.Set(-20.0f, 11.f);
		backPart1 = m_world->CreateBody(&backPart1bd);
		backPart1->CreateFixture(&backPart1fd);
		//Joint for backPart2 and backPart1 a weld joint
		b2WeldJointDef Back21Joint;
		Back21Joint.bodyA=backPart1;
		Back21Joint.bodyB=backPart2;
		Back21Joint.localAnchorA.Set(0.0f,0.0f) ;
		Back21Joint.localAnchorB.Set(-6.0f,0.0f);
		Back21Joint.collideConnected = false;
		m_world->CreateJoint(&Back21Joint);
		//Back tyre
		b2BodyDef backTirebd;
        backTirebd.type = b2_dynamicBody;
        backTirebd.position.Set(-20.0f, 3.5f);
        b2Body* backTire = m_world->CreateBody(&backTirebd);
        backTire->CreateFixture(&centerLeftTirefd);	
		//Joint between back tire and back frame part 1
        b2RevoluteJointDef backTirejoint;
        backTirejoint.Initialize(backTire,backPart1,backTire->GetWorldCenter());
        backTirejoint.localAnchorB.Set(0.0f,-7.5f);
        backTirejoint.collideConnected = false;	
        backTirejoint.enableMotor = true;
        backTirejoint.maxMotorTorque = 20000000.0f;
        backTirejoint.motorSpeed = 3.0f;
        backJoint=(b2RevoluteJoint*)m_world->CreateJoint(&backTirejoint);
        //Box to join front fork and main plate
        b2PolygonShape frontFrameshape;
		frontFrameshape.SetAsBox(2.5f, 2.75f);
		b2BodyDef frontFramebd;
		frontFramebd.position.Set(7.5f, 13.25f);
		frontFramebd.type = b2_dynamicBody;
		b2Body* frontFrame = m_world->CreateBody(&frontFramebd);
		b2FixtureDef frontFramefd;
		frontFramefd.filter.categoryBits = 0x0008;
		frontFramefd.filter.maskBits = 0x0010;
		frontFramefd.density = 2700.f;
		frontFramefd.shape = &frontFrameshape;
		frontFrame->CreateFixture(&frontFramefd);
		//Joint for main plate and front frame to attach front fork a weld joint
		b2WeldJointDef mainFrameFrontJoint;
		mainFrameFrontJoint.bodyA=frontFrame;
		mainFrameFrontJoint.bodyB=mainFrame;
		mainFrameFrontJoint.localAnchorA.Set(0.0f,-2.75f) ;
		mainFrameFrontJoint.localAnchorB.Set(8.f,0.25f);
		mainFrameFrontJoint.collideConnected = false;
		m_world->CreateJoint(&mainFrameFrontJoint);
		//Front frame upper fork
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
        b2RevoluteJointDef frontUpperForkjoint;
        frontUpperForkjoint.bodyA=frontPartUpperFork,
        frontUpperForkjoint.bodyB=frontFrame;
        frontUpperForkjoint.localAnchorA.Set(0,0);
        frontUpperForkjoint.localAnchorB.Set(-1.0f,0.75f);
        frontUpperForkjoint.collideConnected = false;	
        frontUpperForkjoint.enableLimit = true;
		frontUpperForkjoint.upperAngle = 1.f;
		frontUpperForkjoint.lowerAngle = -1.f;
        m_world->CreateJoint(&frontUpperForkjoint);
        //Front frame part 3
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
        b2RevoluteJointDef frontUpperForkPart3joint;
        frontUpperForkPart3joint.bodyA=frontPartUpperFork,
        frontUpperForkPart3joint.bodyB=frontPart3;
        frontUpperForkPart3joint.localAnchorA.Set(6.f,7.f);
        frontUpperForkPart3joint.localAnchorB.Set(0.0f,0.f);
        frontUpperForkPart3joint.collideConnected = false;	
        m_world->CreateJoint(&frontUpperForkPart3joint);
        //Front frame lower fork
		b2Vec2 frontPartLowerForkvertices[4];
		frontPartLowerForkvertices[2].Set(-1.f, 1.f);
		frontPartLowerForkvertices[1].Set(-1.f, 0.0f);
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
        b2RevoluteJointDef frontLowerForkjoint;
        frontLowerForkjoint.bodyA=frontPartLowerFork,
        frontLowerForkjoint.bodyB=frontFrame;
        frontLowerForkjoint.localAnchorA.Set(0,0);
        frontLowerForkjoint.localAnchorB.Set(1.5f,-0.75f);
        frontLowerForkjoint.collideConnected = false;	
        m_world->CreateJoint(&frontLowerForkjoint);
        //Joint between front upper fork and front frame part 3
        b2RevoluteJointDef frontLowerForkPart3joint;
        frontLowerForkPart3joint.bodyA=frontPartLowerFork,
        frontLowerForkPart3joint.bodyB=frontPart3;
        frontLowerForkPart3joint.localAnchorA.Set(5.5f,5.f);
        frontLowerForkPart3joint.localAnchorB.Set(2.0f,-4.5f);
        frontLowerForkPart3joint.collideConnected = false;	
        m_world->CreateJoint(&frontLowerForkPart3joint);
        //Spring
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
		b2WeldJointDef front23Joint;
		front23Joint.bodyA=frontPart3;
		front23Joint.bodyB=frontPart2;
		front23Joint.localAnchorA.Set(7.0f,-12.0f) ;
		front23Joint.localAnchorB.Set(0.0f,0.0f);
		front23Joint.collideConnected = false;
		m_world->CreateJoint(&front23Joint);
		//front frame part 1
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
		b2WeldJointDef front21Joint;
		front21Joint.bodyA=frontPart1;
		front21Joint.bodyB=frontPart2;
		front21Joint.localAnchorA.Set(0.0f,0.0f) ;
		front21Joint.localAnchorB.Set(2.5f,0.0f);
		front21Joint.collideConnected = false;
		m_world->CreateJoint(&front21Joint);
		//front tyre
		b2BodyDef frontTirebd;
        frontTirebd.type = b2_dynamicBody;
        frontTirebd.position.Set(22.0f, 3.5f);
        b2Body* frontTire = m_world->CreateBody(&frontTirebd);
        frontTire->CreateFixture(&centerLeftTirefd);	
		//Joint between front tire and front frame part 1
        b2RevoluteJointDef frontTirejoint;
        frontTirejoint.Initialize(frontTire,frontPart1,frontTire->GetWorldCenter());
        frontTirejoint.localAnchorB.Set(0.0f,-6.0f);
        frontTirejoint.collideConnected = false;	
        frontTirejoint.enableMotor = true;
        frontTirejoint.maxMotorTorque = 20000000.0f;
        frontTirejoint.motorSpeed = 3.0f;
        frontJoint = (b2RevoluteJoint*)m_world->CreateJoint(&frontTirejoint);

        
      


		
		
	}
	//Terrain
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
		Terrain2vertices[4].Set(210.f, 0.f);
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

    }
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}


