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

#ifndef WHEEL_CONSTRAINT_H
#define WHEEL_CONSTRAINT_H

#include <stdio.h>

class MyWheel
{
public:
	float32 wheelDims[2];

	float32 defaultDensity;
	float32 defaultFriction;
	float32 accelerateForce;

	b2Body* wheels[1];
	b2World* world;
	b2Body* ground;

	MyWheel(b2World *_world, b2Body* _ground, float32 width, float32 height) :
		defaultDensity(20.0f),
		defaultFriction(0.68f),
		accelerateForce(2500.0f),
		ground(_ground),
		world(_world)
	{
		wheelDims[0] = 1.0;
		wheelDims[1] = 0.3;
		b2FixtureDef wheelDef;
		b2PolygonShape wheelShape;
		wheelShape.SetAsBox(wheelDims[0], wheelDims[1]);
		wheelDef.shape = &wheelShape;
		wheelDef.density = defaultDensity/4.0;
		wheelDef.friction = defaultFriction;

		wheels[0] = makeWheel(width, height, wheelDef);

		// for (b2Body* b = world->GetBodyList(); b; b = b->GetNext()) {
		// 	b->Dump();
		// }
	}

	virtual ~MyWheel()
	{
	}

	void Steer(float32 dir = 1.0)
	{
		float32 torque_incr = 500.0;
		b2Body *wheel = this->wheels[0];
		wheel->ApplyTorque(dir * torque_incr);
	}

	void Accelerate(float32 ax = 0.0, float32 ay = 0.0)
	{
		b2Body *wheel = this->wheels[0];
		b2Vec2 dir = b2Vec2(accelerateForce * ax, accelerateForce * ay);
		b2Vec2 force_vector =  wheel->GetWorldVector(dir);
		wheel->ApplyForce(force_vector, wheel->GetWorldCenter());
	}

private:

	b2Body* makeWheel(float32 xmul, float32 ymul, b2FixtureDef &wheelDef)
	{
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		b2Body *wheel;
		bd.position = b2Vec2(xmul, ymul);

		wheel = world->CreateBody(&bd);
		wheel->CreateFixture(&wheelDef);
		wheel->SetLinearDamping(0.0); // 0.01f
		wheel->SetAngularDamping(0.0); // 5.0
		wheel->ResetMassData(); // necessary?

		// Add friction joint:
		{
			b2FrictionJointDef jd;
			jd.localAnchorA.SetZero();
			jd.localAnchorB.SetZero();
			jd.bodyA = ground;
			jd.bodyB = wheel;
			jd.collideConnected = true;
			jd.maxForce = 100.0;  // * mass
			jd.maxTorque = 50.0; // * mass
			jd.muX = 0.001;
			jd.muY = 0.9;
			jd.muT = 1.0;
			world->CreateJoint(&jd);
		}

		return wheel;
	}
};

class TopDownWheel : public Test
{

	MyWheel *wheel;
public:

	TopDownWheel()
{
		m_world->SetGravity(b2Vec2(0.0f, 0.0f));

		b2Body* ground;
		b2BodyDef bd;
		bd.position.Set(0.0f, 0.0f);
		ground = m_world->CreateBody(&bd);

		wheel = new MyWheel(m_world, ground, 2.0, 4.0);
}

	void Keyboard(unsigned char key)
	{
		switch (key)
		{
		case 'w':
			wheel->Accelerate(1.0, 0.0);
			break;

		case 's':
			wheel->Accelerate(-1.0, 0.0);
			break;

		case 'd':
			wheel->Accelerate(0.0, -1.0);
			break;

		case 'a':
			wheel->Accelerate(0.0, 1.0);
			break;

		case 'q':
			wheel->Steer(1.0f);
			break;

		case 'e':
			wheel->Steer(-1.0f);
			break;
		}
	}

	void KeyboardUp(unsigned char key)
	{
		wheel->Steer(0.0f);
	}
	virtual ~TopDownWheel()
	{
		delete wheel;
	}

	static Test* Create()
	{
		return new TopDownWheel;
	}

};

#endif
