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
Package with the code for the car robot. But just physically.
"""

import numpy as np
import random as r
import math
import copy
import worldHandling as wh

# NOTE: all things starting with 'v' in this package are vectors and, therefore, I use Numpy arrays for them

# FUNCTIONS

def expErrorWithDistance(errordistancerate = 0.1): # exponential error that increases with the given distance
    def func_errorwithdistance(realdistance):
        if realdistance > 0.0:
            deviation = np.exp(errordistancerate * realdistance) - 1.0 # the minimum in the exponential is 1.0 and I want to assume there is no error if realdistance is 0.0
            return float(np.random.normal(0.0, deviation, 1)) + realdistance # error over real distance + real distance
        elif realdistance < 0.0:
            deviation = np.exp(errordistancerate * -realdistance) - 1.0
            return realdistance - float(np.random.normal(0.0, deviation, 1))
        else:
            return realdistance # which is 0.0
    return func_errorwithdistance

def percentageError(percentage = 0.1, noise = "mixed"):
    if noise == "chaotic": # linear error that increases with the given distance randomly (that's why it's noisy)
        return lambda realdistance: r.uniform(-1,1) * percentage * realdistance + realdistance
    elif noise == "mixed": # works sort of like a fractal
        error = r.uniform(-1,1) * percentage # random initial error which will give most of the error value
        return lambda realdistance: r.random() * error * realdistance + realdistance
    elif noise == "none": # gives a constant error to the given distance
        error = r.uniform(-1,1) * percentage
        return lambda realdistance: error * realdistance + realdistance
    else:
        raise Exception("Wrong error selected")

# CLASSES

class DiscreteBeamSensor(wh.SpaceReference, wh.Orientable):

    def __init__(self, v_startpos, orientation, steplength, maxdistance, func_errorwithdistance, handler, exceptions = []):
        # TODO: check arguments correctness
        super(DiscreteBeamSensor, self).__init__(v_position = v_startpos, orientation = orientation)
        self.steplength = steplength # remember it is a discrete beam check
        self.maxdistance = maxdistance # max distance it covers
        self.func_errorwithdistance = func_errorwithdistance
        self.handler = handler # it has a handler, but it ain't no physical object, it occupies no space and in this simulation: no space, no physics
        self.exceptions = exceptions
        self.lastHitPoint = self.updateHitPoint()

    def updateHitPoint(self):
        v_dir = np.array([math.cos(self.orientation.getRadians()), math.sin(self.orientation.getRadians())]) # already normalized
        v_pos = np.copy(self.v_position)
        v_dir = self.steplength * v_dir
        iterleft = self.maxdistance / self.steplength # number of iterations needed to cover the desired distance
        while iterleft > 0:
            if self.handler.occupiedExcept(v_pos, self.exceptions):
                iterleft = 0 # we stop the process cause there's a hit
            else:
                v_pos += v_dir
                iterleft -= 1
        self.lastHitPoint = v_pos # update the last one
        return v_pos

    def estimatedDistanceToHit(self):
        distance = np.linalg.norm(self.updateHitPoint() - self.v_position)
        return self.func_errorwithdistance(distance)


class Robot(wh.PhysicalEntity, wh.Orientable): # just the very basic rotating circular robot (works like a typical tank)

    def __init__(self, v_position, radius, orientation, handler, func_odometryerror, func_compasserror, frontsensors = [], leftsensors = [], rightsensors = []):
        # TODO: check arguments correctness
        super(Robot, self).__init__(space = wh.CircularVolume(v_position, radius), handler = handler, orientation = orientation)
        self.rotation = 0.0 # rads per second
        self.speed = 0.0 # units per second
        # TODO: treat the odometer and the compass as separate classes (they're sensors after all)
        self.func_odometryerror = func_odometryerror
        self.func_compasserror = func_compasserror
        self.v_pos_odometry = np.copy(v_position)
        self.orient_compass = copy.copy(orientation)
        self.frontsensors = frontsensors
        self.leftsensors = leftsensors
        self.rightsensors = rightsensors

    def update(self, timeStep): # I assume timeStep is in milliseconds
        v_previous = np.copy(self.space.v_position)
        dist_travel = self.speed * timeStep * 10**(-3)
        ang_rotated = self.rotation * timeStep * 10**(-3)
        self.space.v_position += dist_travel * np.array([math.cos(self.orientation.getRadians()), math.sin(self.orientation.getRadians())])
        self.orientation.setRadians(self.orientation.getRadians() + ang_rotated)
        self.orient_compass.setRadians(self.orient_compass.getRadians() + self.func_compasserror(ang_rotated)) # we compute the preceived rotation
        for obj in self.handler.allPhysicalEntities():
            if obj is not self and self.space.intersects(obj.space): # in case of collision with another object
                self.space.v_position[:] = v_previous[:] # copy previous "legal" values
                return # no need to check anything more nor to change the odometry measurements
        self.v_pos_odometry += self.func_odometryerror(dist_travel) * np.array([math.cos(self.orient_compass.getRadians()), math.sin(self.orient_compass.getRadians())]) # we also compute the perceived position if there was no hit


class SimplestRC(wh.PhysicalEntity): # just the very basic non-rotating RC (radio controlled) "vehicle" or whatever you wanna call it, it works just like a circle that moves in any direction

    def __init__(self, v_position, radius, handler):
        # TODO: check arguments correctness
        super(Robot, self).__init__(space = wh.CircularVolume(v_position, radius), handler = handler)
        self.rotation = 0.0 # rads per second
        self.v_speed = np.array([0., 0.]) # units per second

    def update(self, timeStep): # I assume timeStep is in milliseconds
        v_previous = np.copy(self.space.v_position)
        self.space.v_position += self.v_speed * 10**(-3)
        for obj in self.handler.allPhysicalEntities():
            if obj is not self and self.space.intersects(obj.space): # in case of collision with another object
                self.space.v_position[:] = v_previous[:] # copy previous "legal" values
                return # no need to check anything more nor to change the odometry measurements
