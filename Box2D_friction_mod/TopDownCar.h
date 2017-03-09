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

#ifndef TOP_DOWN_CAR_H
#define TOP_DOWN_CAR_H

#include <stdio.h>

class MyCar
{
 public:
  float32 wheelDims[2];

  float32 halfWidth;
  float32 halfHeight;
  float32 defaultDensity;
  float32 defaultFriction;
  float32 accelerateForce;

  b2Body* wheels[4];
  b2Body* chassis;
  b2World* world;
  b2Body* ground;
  b2RevoluteJoint *frontLeftJoint;

  bool frontWheelDrive;

 MyCar(b2World *_world, b2Body* _ground, float32 width, float32 height) :
  halfWidth(width/2.0f),
    halfHeight(height/2.0f),
    defaultDensity(20.0f),
    defaultFriction(0.68f),
    accelerateForce(2500.0f),
    ground(_ground),
    world(_world),
    frontWheelDrive(true)
      {
	wheelDims[0] = .125;
	wheelDims[1] = .5;

	{
	  b2BodyDef chassisBd;
	  chassisBd.type = b2_dynamicBody;
	  chassisBd.position = b2Vec2(0.0f, 0.0f);
	  // give the chassis some damping
	  //chassisBd.linearDamping = 1.0;
	  //chassisBd.angularDamping = 1.0;

	  chassis = world->CreateBody(&chassisBd);

	  b2FixtureDef chassisFixture;
	  b2PolygonShape chassisShape;
	  chassisShape.SetAsBox(halfWidth, halfHeight);
	  chassisFixture.shape = &chassisShape;
	  chassisFixture.density = 0.05 * defaultDensity;
	  chassisFixture.friction = defaultFriction;

	  chassis->CreateFixture(&chassisFixture);
	  chassis->ResetMassData(); // necessary?
	}
	
	{
	  b2FixtureDef wheelDef;
	  b2PolygonShape wheelShape;
	  wheelShape.SetAsBox(wheelDims[0], wheelDims[1]);
	  wheelDef.shape = &wheelShape;
	  wheelDef.density = defaultDensity/4.0;
	  wheelDef.friction = defaultFriction;

	  wheels[0] = makeWheel(-1, 1, wheelDef);
	  wheels[1] = makeWheel(1, 1, wheelDef);
	  wheels[2] = makeWheel(-1, -1, wheelDef);
	  wheels[3] = makeWheel(1, -1, wheelDef);
	}

	{
	  //Make the thing that ties the wheels together.
	  b2DistanceJointDef rackDef;

	  b2Body *w0 = wheels[0];
	  b2Body *w1 = wheels[1];

	  float32 offset = wheelDims[1] / 2.0f * 2.0f;

	  b2Vec2 pos0 = w0->GetWorldCenter();
	  b2Vec2 pos1 = w1->GetWorldCenter();

	  pos0.y += offset;
	  pos1.y += offset;

	  rackDef.Initialize(w0, w1, pos0, pos1);

	  world->CreateJoint(&rackDef);
	}

	// for (b2Body* b = world->GetBodyList(); b; b = b->GetNext()) {
	// 	b->Dump();
	// }
      }

  void Steer(float32 dir = 1.0)
  {
    frontLeftJoint->SetMotorSpeed(dir * 2.0);
  }

  void Accelerate(float32 dir = 1.0)
  {
    // Which two wheels do we power?
    int offset = frontWheelDrive ? 0 : 2;

    for (int i = 0; i < 2; i++) {
      b2Body *wheel = wheels[i + offset];
      b2Vec2 force_vector = wheel->GetTransform().q.GetYAxis();
      force_vector *= accelerateForce * dir;
      wheel->ApplyForce(force_vector, wheel->GetWorldCenter());
    }
  }


  virtual ~MyCar()
    {
    }

 private:

  b2Body* makeWheel(float32 xmul, float32 ymul, b2FixtureDef &wheelDef)
  {
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    b2Body *wheel;
    bd.position = b2Vec2(xmul * (halfWidth - wheelDims[0]), ymul * halfHeight);

    wheel = world->CreateBody(&bd);
    wheel->CreateFixture(&wheelDef);
    wheel->SetLinearDamping(0.01f);
    wheel->SetAngularDamping(5.0);
    wheel->ResetMassData(); // necessary?

    b2RevoluteJointDef jointDef;
    jointDef.Initialize(chassis, wheel, wheel->GetWorldCenter());

    // If we're doing the front wheels
    if (ymul > 0) {
      jointDef.lowerAngle = -0.25 * b2_pi;
      jointDef.upperAngle = 0.25 * b2_pi;
    }

    jointDef.enableLimit = true;

    b2RevoluteJoint *joint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);
    if (xmul < 0 && ymul > 0)
      {
	frontLeftJoint = joint;
	frontLeftJoint->EnableMotor(true);
	frontLeftJoint->SetMaxMotorTorque(5000.0f);
      }

    {
      float mass = wheel->GetMass();

      b2FrictionJointDef jd;
      jd.localAnchorA.SetZero();
      jd.localAnchorB.SetZero();
      jd.bodyA = ground;
      jd.bodyB = wheel;
      jd.collideConnected = true;
      jd.maxForce = 100.0;  //* mass
      jd.maxTorque = 50.0; //* mass
      jd.muX = 20.0;
      jd.muY = 0.0;
      world->CreateJoint(&jd);
      
    }

    return wheel;
  }

};

class TopDownCar : public Test
{
  MyCar *car;
 public:
  TopDownCar()
    {
      m_world->SetGravity(b2Vec2(0.0f, 0.0f));
		
      b2Body* ground;
      b2BodyDef bd;
      bd.position.Set(0.0f, 0.0f);
      ground = m_world->CreateBody(&bd);

      car = new MyCar(m_world, ground, 2.0, 4.0);
    }

  void Keyboard(unsigned char key)
  {
    switch (key)
      {
      case 'f':
	car->frontWheelDrive = !car->frontWheelDrive;
	break;

      case 'w':
	car->Accelerate(1.0);
	break;

      case 's':
	car->Accelerate(-1.0);
	break;

      case 'a':
	car->Steer(1.0f);
	break;

      case 'd':
	car->Steer(-1.0f);
	break;
      }    
  }

  void KeyboardUp(unsigned char key) 
  {
    car->Steer(0.0f);
  }

  virtual ~TopDownCar()
    {
      delete car;
    }

  static Test* Create()
  {
    return new TopDownCar;
  }

};


#endif
