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

class TopDownFrictionJoint(Framework):
	name="TopDownFrictionJoint"
	description="Use w, a, and d to control the 'wheel'."
	def __init__(self):
		super(TopDownFrictionJoint, self).__init__()
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

		# Make a wheel
		self.wheel = self._makeWheel((0,10), muX = 0.0, muY = 0.8, muT = 0.0)

	def _makeWheel(self, pos, length = 1.0, width = 0.3, muX = 0.001, muY = 0.8, muT = 1.0):
		boxdef = b2FixtureDef(shape=b2PolygonShape(box=(length, width)), density=1, friction=0.3)
		wheel = self.world.CreateDynamicBody(position = pos, fixtures = boxdef)
		hypothetical_gravity = 9.8
		self.world.CreateFrictionJoint(
			bodyA = self.ground,
			bodyB = wheel,
			localAnchorA = (0,0), 
			localAnchorB = (0,0), 
			collideConnected = True,
			maxForce = wheel.mass * hypothetical_gravity,
			maxTorque = wheel.mass * length * width * hypothetical_gravity,
			muX = muX,
			muY = muY,
			muT = muT)
		return(wheel)

	def Keyboard(self, key):
		if not self.wheel:
			return
		if key==Keys.K_w:
			f = self.wheel.GetWorldVector(localVector = (100.0, 0.0))
			p = self.wheel.GetWorldPoint(localPoint = (0.0, 0.0))
			self.wheel.ApplyForce(f, p)
		if key==Keys.K_s:
			f = self.wheel.GetWorldVector(localVector = (-100.0, 0.0))
			p = self.wheel.GetWorldPoint(localPoint = (0.0, 0.0))
			self.wheel.ApplyForce(f, p)
		if key==Keys.K_a:
			f = self.wheel.GetWorldVector(localVector = (00.0, 100.0))
			p = self.wheel.GetWorldPoint(localPoint = (0.0, 0.0))
			self.wheel.ApplyForce(f, p)
		if key==Keys.K_d:
			f = self.wheel.GetWorldVector(localVector = (0.0, -100.0))
			p = self.wheel.GetWorldPoint(localPoint = (0.0, 0.0))
			self.wheel.ApplyForce(f, p)
		elif key==Keys.K_q:
			self.wheel.ApplyTorque(50.0)
		elif key==Keys.K_e:
			self.wheel.ApplyTorque(-50.0)

if __name__=="__main__":
	 main(TopDownFrictionJoint)
