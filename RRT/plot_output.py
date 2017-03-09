#!/usr/bin/env python
# Copyright (c) 2010, Georgia Tech Research Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#     * Redistributions of source code must retain the above
#       copyright notice, this list of conditions and the following
#       disclaimer.
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials
#       provided with the distribution.
#     * Neither the name of the Georgia Tech Research Corporation nor
#       the names of its contributors may be used to endorse or
#       promote products derived from this software without specific
#       prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
# IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
# TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.

from matplotlib import pyplot
from matplotlib.patches import Circle
import re
import sys

class RRTOutputParser(object):
	"""Class for parsing and plotting output of RRT"""
	def __init__(self, outfile):
		super(RRTOutputParser, self).__init__()
		self.outfile = outfile
		self.matchedTrees = None
		self.matchedPaths = None
		
	def parsePaths(self):
		"""Extracts the list of path segments"""
		f=open(self.outfile)
		lines=f.readlines()
		f.close()
		s="".join(lines)
		self.matchedPaths=re.findall('(<path>[^path]*</path>)',s,re.DOTALL)
		
	def plotPath(self, pathnum, color = 'b'):
		"""Plots the parsed path points"""
		if self.matchedPaths == None:
			self.parsePaths();
		segments = self.matchedPaths[pathnum].split("\n")[1:-1]

		xpairs = []
		ypairs = []
		for s in segments:
			pts=[float(p) for p in s.split(" ")]
			xpairs.append(pts[0])
			ypairs.append(pts[1])
			
		pyplot.plot(xpairs, ypairs, color, linewidth = 3, hold = True)

	def parseTrees(self):
		"""Extracts the list of tree segments"""
		f=open(self.outfile)
		lines=f.readlines()
		f.close()
		s="".join(lines)
		self.matchedTrees=re.findall('(<tree> \([0-9]* nodes\)[^tree]*</tree>)',s,re.DOTALL)

	def plotTree(self, treenum, color = 'b'):
		"""Plots the parsed tree points"""
		if self.matchedTrees == None:
			self.parseTrees();
		segments = self.matchedTrees[treenum].split("\n")[1:-1]
		
		xpairs = []
		ypairs = []
		for s in segments:
			pts = s.split(" -> ")
			vals = [p.split(' ') for p in pts]
			xpairs.append(float(vals[0][0]))
			xpairs.append(float(vals[1][0]))
			xpairs.append(None) # important: breaks plot cmd into separate segments
			ypairs.append(float(vals[0][1]))
			ypairs.append(float(vals[1][1]))
			ypairs.append(None) # important: breaks plot cmd into separate segments
		
		pyplot.plot(xpairs, ypairs, color, hold = True)
		
	def plotObstacles(self, clr = 'k'):
		"""Plots the obstacles"""
		obstacles = [
			[(1,1),0.5],
			[(2,2),0.7],
			[(3,3),0.5],
			[(3,1),0.65],
			[(1,3),0.65]
			]
			
		for o in obstacles:
			c=Circle(o[0], radius=o[1], color=clr)
			c.set_facecolor("#C0C0D0")
			ax=pyplot.gca()
			ax.add_patch(c)

if __name__ == '__main__':
	outfilename = sys.argv[1]
	ROP = RRTOutputParser(outfilename)
	ROP.plotObstacles('k')
	ROP.plotTree(0, 'b')
	if (len(ROP.matchedTrees) > 1):
		ROP.plotTree(1, 'r')
	ROP.plotPath(0, 'y')
	pyplot.show()