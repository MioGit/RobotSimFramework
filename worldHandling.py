# -*- coding: utf-8 -*-
# Python 2.7.8

"""
Author:
Isaac Sanchez Ruiz
appllgc@gmail.com

    Copyright (C) 2014  Isaac Sanchez Ruiz

This file is part of RobotSimFramework.

    RobotSimFramework is free software: you can redistribute it and/or modify
    it under the terms of the Lesser GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    RobotSimFramework is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the Lesser GNU General Public License
    along with RobotSimFramework.  If not, see <http://www.gnu.org/licenses/>.

Description:
Package with the code to represent the entire world, communicate with it and the various tools to create / handle it.
"""

# NOTE: all things starting with 'v' in this package are vectors and, therefore, I use Numpy arrays for them

import numpy as np
import copy
import math

# PARAMETERS

numberOfSamples = 20 # a constant number of samples is not a good idea, but for now I'll leave it this way (TODO)

# CLASSES

class Angle(object): # DEF: angle instance stored in radians

    def __init__(self, radians, relativeToAngle = None, **kwds):
        super(Angle, self).__init__(**kwds)
        if relativeToAngle is None:
            self.relativeToAngle = 0.0 # another angle, by default I use the number 0.0 which is not an Angle but Python doesn't care
        else:
            self.relativeToAngle = relativeToAngle
        self.radians = radians % (2*math.pi)

    def getRadians(self):
        return self.radians + self.relativeToAngle # the relative angle is only calculated when needed
        # For the previous thing I was trying to use lambda functions but they don't seem to work with the '%' operand, what a shame :(

    def setRadians(self, radians):
        self.radians = copy.copy(radians) % (2*math.pi)

    def __add__(self, angle):
        return (self.radians + angle.radians) % (2*math.pi)

    def __radd__(self, radians):
        return (self.radians + radians) % (2*math.pi)

    def __sub__(self, angle):
        return self.__rsub__(self, angle.getRadians())

    def __rsub__(self, radians):
        a1 = self.getRadians()
        a2 = radians
        sub = a1 - a2
        if abs(sub) > math.pi: # greater than 180 degrees
            if a1 > a2:
                return Angle(a1 - 2*math.pi * a2)
            else:
                return Angle(2*math.pi * a1 - a2)
        else:
            return Angle(sub)

class SpaceReference(object): # DEF: a point in a 2D space

    def __init__(self, v_position, **kwds): # TODO: add relativeTo functionality??
        # TODO: check arguments correctness
        super(SpaceReference, self).__init__(**kwds)
        self.v_position = v_position

class Orientable(object): # DEF: any orientable object in any Angle reference

    def __init__(self, orientation, **kwds):
        # TODO: check arguments correctness
        super(Orientable, self).__init__(**kwds)
        self.orientation = orientation # it has to be an Angle

class VolumeSpace(SpaceReference): # DEF: a space reference that CAN occupy a FINITE volume (a finite set of points in the 2D space)
# IMPORTANT: this class behaves as an empty volume by default, if you want a real volume, IMPLEMENT IT using inheritance

    def occupied(self, v_position): # DEF: this is a special feature in any VolumeSpace, it requires the space reference; if no space reference, occupied(v) is meaningless
        return False # I just said is empty... there is no possible position that can be occupied

    def boundaries(self): # required for performance optimisation, only meaningful if the volume is finite
        return None, None # returns the top-left and bottom-right corner boundary coords, since it's empty, it has no boundaries

    def intersects(self, otherVolume): # generic intersection code which is independent of the shape, it just requires occupied(v)
        # This code uses a probabilistic approach to solve the instersection problem
        topleft, bottomright = self.boundaries()
        samples = np.random.rand(numberOfSamples,2) * (bottomright - topleft) + topleft # array of 2D sample points inside the boundaries
        for pos in samples:
            if self.occupied(pos) and otherVolume.occupied(pos): # if a point lies inside both objects at the same time, they intersect
                return True
        return False

class RectangularVolume(VolumeSpace): # DEF: non-orientable rectangle volume paralell to the XY axis of its relative space reference

    def __init__(self, v_position_topleft, v_position_bottomright):
        # TODO: I do not check if this is done correctly, the corner bottomright must be to the right and to the bottom of topleft
        super(RectangularVolume, self).__init__(v_position_topleft)
        self.v_position_topleft = v_position_topleft
        self.v_position_bottomright = v_position_bottomright

    def occupied(self, v_position):
        inside_topleft = v_position[0] >= self.v_position_topleft[0] and v_position[1] <= self.v_position_topleft[1]
        inside_bottomright = v_position[0] <= self.v_position_bottomright[0] and v_position[1] >= self.v_position_bottomright[1]
        return inside_topleft and inside_bottomright

    def boundaries(self):
        return self.v_position_topleft, self.v_position_bottomright

class CircularVolume(VolumeSpace): # DEF: non-orientable circular volume

    def __init__(self, v_position, radius):
        # TODO: check arguments correctness (do not forget to check non negative radius)
        super(CircularVolume, self).__init__(v_position)
        self.radius = radius

    def occupied(self, v_position):
        distance = np.linalg.norm(v_position - self.v_position)
        return distance <= self.radius

    def boundaries(self):
        diagonal = np.array([self.radius, -self.radius]) # vector that points to the SE
        return self.v_position - diagonal, self.v_position + diagonal

    def intersects(self, otherVolume):
        if isinstance(otherVolume, CircularVolume):
            distance = np.linalg.norm(otherVolume.v_position - self.v_position)
            return distance <= otherVolume.radius + self.radius
        elif isinstance(otherVolume, RectangularVolume):
            """
            Regions:
            0 | 1 | 2
            ---------
            3 | 4 | 5
            ---------
            6 | 7 | 8
            """
            v_topleft, v_bottomright = otherVolume.boundaries()
            regions_036 = self.v_position[0] < v_topleft[0]
            regions_258 = self.v_position[0] > v_bottomright[0]
            regions_012 = self.v_position[1] > v_topleft[1]
            regions_678 = self.v_position[1] < v_bottomright[1]
            if regions_012:
                if regions_036:
                    if self.occupied(v_topleft):
                        return True
                elif regions_258:
                    if self.occupied(np.array([v_bottomright[0], v_topleft[1]])):
                        return True
                else: # regions 147
                    return self.v_position[1] - self.radius <= v_topleft[1]
            elif regions_678:
                if regions_036:
                    if self.occupied(np.array([v_topleft[0], v_bottomright[1]])):
                        return True
                elif regions_258:
                    if self.occupied(v_bottomright):
                        return True
                else: # regions 147
                    return self.v_position[1] + self.radius >= v_bottomright[1]
            else: # regions 345
                if regions_036:
                    return self.v_position[0] + self.radius >= v_topleft[0]
                elif regions_258:
                    return self.v_position[0] - self.radius <= v_bottomright[0]
                else: # region 4
                    return True
        else:
            return super(CircularVolume, self).intersects(otherVolume)

"""
class Wall2Dbitmap(VolumeReference):

    def __init__(self, bitmaps, v_position, scale, wallsOutside = False):
        self.bitmap = bitmaps # volume bitmaps
        # x and y are the top-left corner's coordinates in the bitmap
        self.v_position = v_position
        self.scale = scale # real units per bitmap-pixel square side length both in X and Y
        self.wallsOutside = wallsOutside # outside the bitmap I do not know if there are walls or not, so I use this boolean
        self.currentBitmap = 0 # for the animated bitmaps

    def occupied(self, v_pos):
        j = int((v_pos[0] - self.v_pos[0]) / self.scale)
        i = int(-(v_pos[1] - self.v_pos[1]) / self.scale)
        shape = self.bitmap.shape
        if i < 0 or i >= shape[1] or j < 0 or j >= shape[2]: # case outside of bitmap
            return self.wallsOutside
        return self.bitmap[self.currentBitmap, i, j] # True means inside, False means outside

    def boundaries(self):
        return self.v_position, np.array([self.bitmap.shape[1], self.bitmap.shape[2]]) * self.scale + self.v_position

    def stepAnimation(self): # next frame in the animation of the world
        if self.currentBitmap == self.bitmap.shape[0] - 1:
            self.currentBitmap = 0
        else:
            self.currentBitmap += 1

    def getBitmap(self):
        return self.bitmap[self.currentBitmap,:,:]
"""

class PhysicalEntity(object): # DEF: entity that has physical properties (minimum physical property in this case is space defined by VolumeSpace) that manages its behavior by itself

    def __init__(self, space, handler = None, **kwds):
        # TODO: check arguments correctness
        super(PhysicalEntity, self).__init__(**kwds)
        self.space = space # each physical object MUST have one VolumeSpace reference
        self.handler = handler # each physical object CAN be part of one ScenarioHandler (managed group of physical entities)

    def update(self, timeStep): # DEF: update = up-to-date = up to the current time (also known as "now")
        pass # LEFT to be defined by others

class ScenarioHandler(object): # DEF: this is a collection of tools to manage PhysicalEntity objects, it serves as a container of these objects as well and it handles the simulation time

    def __init__(self, staticObjects = [], dynamicObjects = []): # the lists can be empty since this is supposed to support dynamic changes
        # TODO: check arguments correctness
        self.time = 0.0
        self.staticObjects = staticObjects # this must be a list of PhysicalEntity objects that are not going to be updated by the handler
        self.dynamicObjects = dynamicObjects # this must be a list of PhysicalEntity objects that are going to be updated by the handler
        # IMPORTANT: remember that update = up-to-date = changes over TIME (in this case I use time steps instead of other forms of time)

    def allPhysicalEntities(self):
        return self.staticObjects + self.dynamicObjects

    def occupied(self, v_pos): # check if any object in this handler occupies the given position now
        for obj in self.staticObjects + self.dynamicObjects:
            if obj.space.occupied(v_pos):
                return True
        return False

    def occupiedBy(self, v_pos, objs): # is the position occupied by one of these objects now (some of them may not pertain to the scenario)
        allobjs = self.staticObjects + self.dynamicObjects
        for obj in objs: # I assume the list of objs is going to be smaller than all the objects in the scenario, therefore, this way is more optimised
            if obj in allobjs and obj.space.occupied(v_pos):
                return True
        return False

    def occupiedExcept(self, v_pos, objs): # same thing but the opposite (some of them may not pertain to the scenario)
        for obj in self.staticObjects + self.dynamicObjects: # same assumption as before
            if obj not in objs and obj.space.occupied(v_pos):
                return True
        return False

    def updateAll(self, timeStep):
        self.time += timeStep
        for obj in self.dynamicObjects:
            obj.update(timeStep)
