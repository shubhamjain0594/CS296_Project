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
	b2RevoluteJoint* join;
 	void dominos_t::keyboard(unsigned char key)
	{
		switch(key)
		{
			case('a'):
				join->SetMotorSpeed(10.0f);
		}
		
	}	

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
    {
		//Center Horizontal top rod
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
		b2BodyDef centerBottombd;
		centerBottombd.position.Set(0.0f, 11.f);
		centerBottombd.type = b2_dynamicBody;
		b2Body* centerBottom = m_world->CreateBody(&centerBottombd);
		centerBottom->CreateFixture(&centerTopfd);
		//Center vertical left rod
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
		b2BodyDef centerRightbd;
		centerRightbd.position.Set(6.0f, 10.0f);
		centerRightbd.type = b2_dynamicBody;
		b2Body* centerRight = m_world->CreateBody(&centerRightbd);
		centerRight->CreateFixture(&centerLeftfd);
		//Joint between center left and center top  rod
		b2RevoluteJointDef centerLeftTopJoint;
        b2Vec2 centerLeftTopJointv1(-6.0f,15.0f);
        centerLeftTopJoint.Initialize(centerTop,centerLeft,centerLeftTopJointv1);
        centerLeftTopJoint.localAnchorA.Set(-6.0f,0.0f) ;
        centerLeftTopJoint.localAnchorB.Set(0.0f,5.0f);
        centerLeftTopJoint.collideConnected = false;
        m_world->CreateJoint(&centerLeftTopJoint);	
        //Joint between center right and center top  rod
		b2RevoluteJointDef centerRightTopJoint;
        b2Vec2 centerRightTopJointv1(6.0f,15.0f);
        centerRightTopJoint.Initialize(centerTop,centerRight,centerRightTopJointv1);
        centerRightTopJoint.localAnchorA.Set(6.0f,0.0f) ;
        centerRightTopJoint.localAnchorB.Set(0.0f,5.0f);
        centerRightTopJoint.collideConnected = false;
        m_world->CreateJoint(&centerRightTopJoint);	
        //Joint between center left and center bottom  rod
		b2RevoluteJointDef centerLeftBottomJoint;
        b2Vec2 centerLeftBottomJointv1(-6.0f,11.0f);
        centerLeftBottomJoint.Initialize(centerBottom,centerLeft,centerLeftBottomJointv1);
        centerLeftBottomJoint.localAnchorA.Set(-6.0f,0.0f) ;
        centerLeftBottomJoint.localAnchorB.Set(0.0f,1.0f);
        centerLeftBottomJoint.collideConnected = false;
        m_world->CreateJoint(&centerLeftBottomJoint);	
        //Joint between center right and center top  rod
		b2RevoluteJointDef centerRightBottomJoint;
        b2Vec2 centerRightBottomJointv1(6.0f,11.0f);
        centerRightBottomJoint.Initialize(centerBottom,centerRight,centerRightBottomJointv1);
        centerRightBottomJoint.localAnchorA.Set(6.0f,0.0f) ;
        centerRightBottomJoint.localAnchorB.Set(0.0f,1.0f);
        centerRightBottomJoint.collideConnected = false;
        m_world->CreateJoint(&centerRightBottomJoint);	
        //Center left tyre 
        b2CircleShape centerLeftcircle;
        centerLeftcircle.m_radius = 3.5f;
        b2FixtureDef centerLeftTirefd;
        centerLeftTirefd.shape = &centerLeftcircle;
        centerLeftTirefd.density = 1600.0f;
        centerLeftTirefd.friction = 0.5f;
        centerLeftTirefd.restitution = 0.5f;
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
        b2RevoluteJointDef centerLeftTirejoint;
        centerLeftTirejoint.Initialize(centerLeftTire,centerLeft,centerLeftTire->GetWorldCenter());
        centerLeftTirejoint.localAnchorB.Set(0.0f,-6.5f);
        centerLeftTirejoint.collideConnected = false;
        centerLeftTirejoint.enableMotor = true;
        centerLeftTirejoint.maxMotorTorque = 20000000.0f;
        //centerLeftTirejoint.motorSpeed = 10.0f;
        m_world->CreateJoint(&centerLeftTirejoint);	
        //Joint between center right tire and center right rod
        b2RevoluteJointDef centerRightTirejoint;
        centerRightTirejoint.Initialize(centerRightTire,centerRight,centerRightTire->GetWorldCenter());
        centerRightTirejoint.localAnchorB.Set(0.0f,-6.5f);
        centerRightTirejoint.collideConnected = false;	
        centerRightTirejoint.enableMotor = true;
        centerRightTirejoint.maxMotorTorque = 20000000.0f;
        //centerRightTirejoint.motorSpeed = 10.0f;
        m_world->CreateJoint(&centerRightTirejoint);
        //main frame plate
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
        b2RevoluteJointDef mainTopjoint;
		mainTopjoint.Initialize(attachCenterFrame,centerTop,centerTop->GetWorldCenter());
		mainTopjoint.localAnchorA.Set(0.0f,1.75f) ;
		mainTopjoint.localAnchorB.Set(0.0f,0.0f);
		mainTopjoint.enableLimit = true;
		mainTopjoint.upperAngle = 0.5f;
		mainTopjoint.lowerAngle = -0.5f;
		mainTopjoint.collideConnected = false;
		m_world->CreateJoint(&mainTopjoint);		
		//Joint for main frame and bottom rod 
		b2RevoluteJointDef mainBottomjoint;
		mainBottomjoint.Initialize(attachCenterFrame,centerBottom,centerBottom->GetWorldCenter());
		mainBottomjoint.localAnchorA.Set(0.0f,-1.75f) ;
		mainBottomjoint.localAnchorB.Set(0.0f,0.0f);
		mainBottomjoint.collideConnected = false;
		m_world->CreateJoint(&mainBottomjoint);		
		//Joint for main plate and main frame to attach center frame a weld joint
		b2WeldJointDef mainFrameCenterJoint;
		mainFrameCenterJoint.bodyA=attachCenterFrame;
		mainFrameCenterJoint.bodyB=mainFrame;
		mainFrameCenterJoint.localAnchorA.Set(0.0f,-2.75f) ;
		mainFrameCenterJoint.localAnchorB.Set(0.5f,0.25f);
		mainFrameCenterJoint.collideConnected = false;
		m_world->CreateJoint(&mainFrameCenterJoint);
		//Attaching back fork
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
		b2WeldJointDef mainFrameBackJoint;
		mainFrameBackJoint.bodyA=backFrame;
		mainFrameBackJoint.bodyB=mainFrame;
		mainFrameBackJoint.localAnchorA.Set(0.0f,-2.75f) ;
		mainFrameBackJoint.localAnchorB.Set(-9.5f,0.25f);
		mainFrameBackJoint.collideConnected = false;
		m_world->CreateJoint(&mainFrameBackJoint);
		//Back frame part 3
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
		backPart3fd.restitution = 0.0f;
		b2BodyDef backPart3bd;
		backPart3bd.type = b2_dynamicBody;
		backPart3bd.position.Set(-10.0f, 15.0f);
		backPart3 = m_world->CreateBody(&backPart3bd);
		backPart3->CreateFixture(&backPart3fd);
		//Joint for backPart3 and backFrame
        b2RevoluteJointDef mainBackjoint;
		mainBackjoint.Initialize(backFrame,backPart3,backPart3->GetWorldCenter());
		mainBackjoint.enableLimit = true;
		mainBackjoint.upperAngle = 0.f;
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
		backPart2fd.restitution = 0.0f;
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
		backPart1fd.restitution = 0.0f;
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
        //backTirejoint.motorSpeed = 10.0f;
        m_world->CreateJoint(&backTirejoint);
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
		frontPartUpperForkfd.restitution = 0.0f;
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
		frontUpperForkjoint.upperAngle = 0.8f;
		frontUpperForkjoint.lowerAngle = -0.8f;
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
		frontPart3fd.restitution = 0.0f;
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
		frontPartLowerForkfd.restitution = 0.0f;
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
		frontPart2fd.restitution = 0.0f;
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
		frontPart1fd.friction = 0.5f;
		frontPart1fd.restitution = 0.0f;
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
        //frontTirejoint.motorSpeed = 10.0f;
        join = (b2RevoluteJoint*)m_world->CreateJoint(&frontTirejoint);

        
      


		
		
	}
    {   
      b2BodyDef *bd = new b2BodyDef;
      bd->position.Set(90,4);
      bd->fixedRotation = true;
      
      //~ 
	    /*! \par
	     * The open box 
		 * Variable:fd1,fd2,fd3:: Type:FixtureDef :: Value:density=10,fricion=0.5,restitution=0 :: Action:To collect the spheres and pull down the pulley.  <br>
		 * Variable:bs1 :: Type:PolygonShape :: Value: height=2,width=0.2; position x=0,y=-1.9.<br>
		 * Variable:bs2 :: Type:PolygonShape :: Value: height=0.2,width=2; position x=2,y=-0.<br>
		 * Variable:bs3 :: Type:PolygonShape :: Value: height=0.2,width=2; position x=-2,y=0.<br>
		 * Variable:box1 :: Action: Creates a box of fd1,fd2 and fd3.<br>
		*/   
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 100;
      fd1->restitution = 0.5f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(60,4);
      fd1->shape = &bs1;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
    }
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}


