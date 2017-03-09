/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#ifndef B2_FRICTION_JOINT_H
#define B2_FRICTION_JOINT_H

#include <Box2D/Dynamics/Joints/b2Joint.h>


/**
 * Explanation of 2D friction mod:
 * The purpose of this joint is to simulate friction in the absence of
 * contact events.  The standard use-case is top-down worlds, where we
 * want to model objects such as blocks or wheels interacting with the
 * (hypothetical) surface.  Gravity is set to zero in these scenarios
 * and there is no normal force, which leaves joints as the mechanism
 * for processing these forces inside the solver.  The underlying
 * friction model is viscous, since impules are proportional to velocity
 * (as with linear/angular damping).  To simulate coloumb friction, we'd
 * have to normalize the velocity vector "Cdot" in SolveVelocityConstraints
 * (not implemented).
 *
 * In order to make sense, this must assume that the linear friction
 * components are attached to one of the bodies.  When simulating
 * top-down car wheels, bodyA is the ground body and bodyB is a wheel.
 * The order of these is important, since the coordinate frame for
 * linear friction is attached to bodyB.
 *
 * Changes from Erin's version:
 * > added member variables for body orientations, since needed in solver
 * > added three friction coefficients for X, Y, and Theta, (w/ getter/setters)
 * > scaled angular impules update by muT
 * > rotated and scaled linear impulse by muX,muY
 *
 * jkscholz@gatech.edu
 * 11/6/2012
 */

/// Friction joint definition.
struct b2FrictionJointDef : public b2JointDef
{
	b2FrictionJointDef()
	{
		type = e_frictionJoint;
		localAnchorA.SetZero();
		localAnchorB.SetZero();
		maxForce = 0.0f;
		maxTorque = 0.0f;
		muX = 0;
		muY = 0;
		muT = 0;
	}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB;

	/// The maximum friction force in N.
	float32 maxForce;

	/// The maximum friction torque in N-m.
	float32 maxTorque;
	
	/// The orthogonal and angular friction components
	float32 muX;
	float32 muY;
	float32 muT;
};

/// Friction joint. This is used for top-down friction.
/// It provides 2D translational friction and angular friction.
class b2FrictionJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const;
	b2Vec2 GetAnchorB() const;

	b2Vec2 GetReactionForce(float32 inv_dt) const;
	float32 GetReactionTorque(float32 inv_dt) const;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Set the maximum friction force in N.
	void SetMaxForce(float32 force);

	/// Get the maximum friction force in N.
	float32 GetMaxForce() const;

	/// Set the maximum friction torque in N*m.
	void SetMaxTorque(float32 torque);

	/// Get the maximum friction torque in N*m.
	float32 GetMaxTorque() const;

	/// Get the X friction component
	float32 GetMuX() const;

	/// Set the X friction component
	void SetMuX(float32 muX);

	/// Get the Y friction component
	float32 GetMuY() const;

	/// Set the Y friction component
	void SetMuY(float32 muY);

	/// Get the T friction component
	float32 GetMuT() const;

	/// Set the T friction component
	void SetMuT(float32 muT);

	/// Dump joint to dmLog
	void Dump();

protected:

	friend class b2Joint;

	b2FrictionJoint(const b2FrictionJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data);
	void SolveVelocityConstraints(const b2SolverData& data);
	bool SolvePositionConstraints(const b2SolverData& data);

	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;

	// Solver shared
	b2Vec2 m_linearImpulse;
	float32 m_angularImpulse;
	float32 m_maxForce;
	float32 m_maxTorque;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	float32 m_invMassA;
	float32 m_invMassB;
	float32 m_invIA;
	float32 m_invIB;
	b2Mat22 m_linearMass;
	float32 m_angularMass;
	b2Rot m_qA, m_qB;

	/// Orthogonal friction components
	float32 m_muX;
	float32 m_muY;
	float32 m_muT;
	b2Mat22 m_muXY; // matrix representation of the linear friction components
};

#endif
