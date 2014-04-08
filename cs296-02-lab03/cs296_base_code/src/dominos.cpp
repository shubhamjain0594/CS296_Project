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
 	void dominos_t::keyboard(unsigned char key)
	{
	}

  dominos_t::dominos_t()
  {
		
    b2Body* ground; //! ground is a pointer to the ground which is a Box2D body. 

	/*! \par Block 1: Ground
	 * A Box2D Edge shape used to represent ground. <br>
	 * Edge is set between two points on x-axis x1=-90 and x2=90. <br>
	 * bd is a Box2D body definition.<br>
	 * b1 is then pointed to a body created with body definition bd<br>
	 * A fixture is created on the body with Edge shape created earlier and 0 density
	 */
	 {
		 b2EdgeShape groundShape;
		 groundShape.Set(b2Vec2(-2000.0f, 0.0f), b2Vec2(2000.0f, 0.0f));
		 b2BodyDef groundDef;
		 ground = m_world->CreateBody(&groundDef);
		 ground->CreateFixture(&groundShape, 0.0f);
	 }
	 
	 {
		 //A Rod
		 b2PolygonShape centerRodShape;
		 centerRodShape.SetAsBox(5.0f, 0.5f);
		 
		 b2FixtureDef centerRodFixDef;
		 centerRodFixDef.shape = &centerRodShape;
		 centerRodFixDef.density = 1000.0f;
		 centerRodFixDef.friction = 0.25f;
		 centerRodFixDef.restitution = 0.25f;
		 
		 b2BodyDef centerRodDef;
		 centerRodDef.type = b2_dynamicBody;
		 centerRodDef.position.Set(0.0f, 2.5f);
		 
		 b2Body* centerRod = m_world->CreateBody(&centerRodDef);
		 centerRod->CreateFixture(&centerRodFixDef);
		 
		 //Common circle shape and fixture definitions for all wheels/tires
		 b2CircleShape wheel;
		 wheel.m_radius = 2.5f;
		 
		 b2FixtureDef wheelFixDef;
		 wheelFixDef.shape = &wheel;
		 wheelFixDef.density = 1000.0f;
		 wheelFixDef.friction = 0.25f;
		 wheelFixDef.restitution = 0.25f; //
		 
		 //Definitions for the center left wheel
		 b2BodyDef centerLeftWheelDef;
		 centerLeftWheelDef.type = b2_dynamicBody;
		 centerLeftWheelDef.position.Set(-5.0f, 2.5f);
		 
		 b2Body* centerLeftWheel = m_world->CreateBody(&centerLeftWheelDef);
		 centerLeftWheel->CreateFixture(&wheelFixDef); //
		 
		 //Definitions for the center right wheel
		 b2BodyDef centerRightWheelDef;
		 centerRightWheelDef.type = b2_dynamicBody;
		 centerRightWheelDef.position.Set(5.0f, 2.5f);
		 
		 b2Body*centerRightWheel = m_world->CreateBody(&centerRightWheelDef);
		 centerRightWheel->CreateFixture(&wheelFixDef); //
		 
		 //Temporary Joining the rod and the wheels
		 b2RevoluteJointDef leftWheelJointDef;
		 leftWheelJointDef.Initialize(centerRod, centerLeftWheel, centerLeftWheel->GetWorldCenter());
		 leftWheelJointDef.enableMotor = true;
		 leftWheelJointDef.maxMotorTorque = 10000000000.0f;
		 leftWheelJointDef.motorSpeed = -1.0f;
		 m_world->CreateJoint(&leftWheelJointDef);
		 
		 b2RevoluteJointDef rightWheelJointDef;
		 rightWheelJointDef.Initialize(centerRod, centerRightWheel, centerRightWheel->GetWorldCenter());
		 rightWheelJointDef.enableMotor = true;
		 rightWheelJointDef.maxMotorTorque = 10000000000.0f;
		 rightWheelJointDef.motorSpeed = -5.0f;
		 m_world->CreateJoint(&rightWheelJointDef);
		 
		 //A Temporary Obstacle
		 b2PolygonShape boxShape;
		 boxShape.SetAsBox(10.0f, 1.0f);
		 
		 b2BodyDef boxDef;
		 boxDef.position.Set(20.0f, 1.0f);
		 
		 b2Body* box = m_world->CreateBody(&boxDef);
		 box->CreateFixture(&boxShape, 0.0f);
	 }
		 
		 
		 
 }
 
 sim_t *sim = new sim_t("CS 296 Project by Group 02", dominos_t::create);
 
}
