#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version Copyright (c) 2010 kne / sirkne at gmail dot com
# 
# This software is provided 'as-is', without any express or implied
# warranty.	 In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

from framework import *
from math import sqrt

class Car(object):
	"""docstring for Car"""
	# Constants
	carDims = (3, 1.5)
	rwDims = (0.6, 0.3)
	fwDims = (0.6, 0.2)
	frontWheelDrive = False
	accelerateForce = 500
	steeringRate = 5
	
	def __init__(self, world, ground, pos = (0,0), scale = 1.0):
		"""Creates a car in the provided world, and joined via friction joints to the
		provided ground
		@param world A b2World with a static ground body defined
		@param ground A static ground"""
		super(Car, self).__init__()
		# set the world
		self.world = world
		self.ground = ground
		self.accelerateForce *= scale**2
		
		# Make a chassis
		chassisDef = b2FixtureDef(
			shape = b2PolygonShape(box = (scale * self.carDims[0], 
										  scale * self.carDims[1])), 
			density = 0.1, 
			friction = 0.3)
		self.chassis = self.world.CreateDynamicBody(position = pos, fixtures = chassisDef)
				
		# Add the 4 wheels
		self.wheel_br = self._makewheel((-self.carDims[0], -self.carDims[1]), self.rwDims[0], self.rwDims[1], scale)
		self.wheel_bl = self._makewheel((-self.carDims[0], self.carDims[1]), self.rwDims[0], self.rwDims[1], scale)
		self.wheel_fr = self._makewheel((self.carDims[0], -self.carDims[1]), self.fwDims[0], self.fwDims[1], scale)
		self.wheel_fl = self._makewheel((self.carDims[0], self.carDims[1]), self.fwDims[0], self.fwDims[1], scale)
		
		# Add steering rack
		djd = Box2D.b2DistanceJointDef()
		djd.bodyA = self.wheel_fr
		djd.bodyB = self.wheel_fl
		djd.anchorA = self.wheel_fr.worldCenter + b2Vec2(-self.fwDims[0], 0)
		djd.anchorB = self.wheel_fl.worldCenter + b2Vec2(-self.fwDims[0], 0)
		self.world.CreateJoint(djd)
		
	def _makewheel(self, offset, length, width, scale = 1., muX = 0.001, muY = 0.8, muT = 0.0):
		front = False
		drive = False
		if offset[0] > 0:
			front = True
			if offset[1] > 0:
				drive = True			
		pos = Box2D.b2Mul(self.chassis.transform, b2Vec2(offset) * scale)
		length *= scale
		width *= scale
		boxdef = b2FixtureDef(
			shape = b2PolygonShape(box = (length, width)), 
			density = 1, 
			friction = 0.3)
		wheel = self.world.CreateDynamicBody(position = pos, fixtures = boxdef)
		
		# Make revolute joint between wheel and chassis
		rjd = Box2D.b2RevoluteJointDef()
		rjd.bodyA = self.chassis
		rjd.bodyB = wheel
		rjd.anchor = wheel.worldCenter
		rjd.enableLimit = True
		if front:
			rjd.lowerAngle = -0.25 * Box2D.b2_pi
			rjd.upperAngle = 0.25 * Box2D.b2_pi
		if drive:
 			rjd.enableMotor = True
			rjd.maxMotorTorque = 1000
			
		joint = self.world.CreateJoint(rjd)
		if drive:
			self.steeringJoint = joint
		
		# Make friction joint between wheel and ground
		hypothetical_gravity = 9.8
		self.world.CreateFrictionJoint(
			bodyA = self.ground,
			bodyB = wheel,
			localAnchorA = (0,0), 
			localAnchorB = (0,0), 
			collideConnected = True,
			maxForce = (4*wheel.mass + self.chassis.mass) * hypothetical_gravity,
			maxTorque = wheel.mass * length * width * hypothetical_gravity,
			muX = muX,
			muY = muY,
			muT = muT)
		return(wheel)
		
	def steer(self, dir = 0.):
		"""docstring for steer"""
		#self.chassis.ApplyTorque(omega)
		self.steeringJoint.motorSpeed = dir * self.steeringRate;
		
	def accelerate(self, fx = 0., fy = 0.):
		"""docstring for accelerate"""
		if self.frontWheelDrive:
			wheels = [self.wheel_fr, self.wheel_fl]
		else:
			wheels = [self.wheel_br, self.wheel_bl]
		for wheel in wheels:
			force = wheel.GetWorldVector(localVector = (fx, fy)) * self.accelerateForce
			wheel.ApplyForce(force, wheel.worldCenter)
		
class TopDownCar(Framework):
	name="TopDownCar"
	description="This demonstrates a top-down car. Use w, a, s, d to control the car, and t to toggle front wheel drive."
	def __init__(self):
		super(TopDownCar, self).__init__()
		self.world.gravity = (0.0, 0.0)

		# The boundaries
		self.ground = self.world.CreateBody(position=(0, 20))
		self.ground.CreateEdgeChain(
							[ (-20,-20),
							  (-20, 20),
							  ( 20, 20),
							  ( 20,-20),
							  (-20,-20) ]
							)
		self.car = Car(self.world, self.ground, pos = (0,10), scale = 0.5)

	def Keyboard(self, key):
		if not self.car:
			return
		if key==Keys.K_w:
			self.car.accelerate(1, 0)
		if key==Keys.K_s:
			self.car.accelerate(-1, 0)
		if key==Keys.K_a:
			self.car.steer(1)
		if key==Keys.K_d:
			self.car.steer(-1)
		elif key==Keys.K_t:
			self.car.frontWheelDrive = not self.car.frontWheelDrive
			print "Front wheel drive: ", self.car.frontWheelDrive
			
	def KeyboardUp(self, key):
		self.car.steer(0)

if __name__=="__main__":
	 main(TopDownCar)
