# -*- coding: utf-8 -*-
# Python 2.7.8

"""
Author:
Isaac Sanchez Ruiz
appllgc@gmail.com

    Copyright (C) 2014  Isaac Sanchez Ruiz

This file is part of SimpleRoboticsFramework.

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
Package with code suitable for my robots and sensors in robots.py.
"""

import random as r
import numpy as np
import networkx as nx
import math
import worldHandling as wh

import pdb

class SingleRobotPlanner(object): # basic planner class for one robot

    def __init__(self, robot, **kwd):
        self.robot = robot

    def nextMove(self): # the thingy that tells the robot where to go and basically, controls the robot
        pass # LEFT to be defined

class StupidPlanner(SingleRobotPlanner): # just for testing purposes

    def nextMove(self): # just tells it to go forward
        self.robot.speed = 3.

class AvoidHitPlanner(SingleRobotPlanner): # it adds some functions that help the robot to decide how to avoid hitting an obstacle

# Directly derived (fork) from MyBrain03.py that I created during the first assignment

    def __init__(self, robot, baseRotation = 0.2, baseSpeed = 50., dangerRotation = 0.6, epsilon_rotation = 0.01, epsilon_speed = 5., thresh_front = 18., thresh_side = 14., thresh_ignore = 100., **kwd):
        super(AvoidHitPlanner, self).__init__(robot = robot)

        # Speeds by default
        self.baseRotation = baseRotation # base rotation for the calculations
        self.baseSpeed = baseSpeed # same with the speed
        self.dangerRotation = dangerRotation
        self.epsilon_rotation = epsilon_rotation # epsilon for the inverse rotation formula that I use
        self.epsilon_speed = epsilon_speed # another epsilon but in this case to calculate the speed

        # Hit danger thresholds
        self.thresh_front = thresh_front # distance to front obstacle warning threshold
        self.thresh_side = thresh_side # distance to side obstacle warning threshold
        self.thresh_ignore = thresh_ignore # threshold to ignore sensor information

        # Text buffer to provide information
        self.textbuffer = ""

        # Robot's memory variables
        self.lastDangerRotation = 0.

    def updateRobot(self, speed, rotation): # this function is not really that necessary (just to improve the looks of the code)
        self.robot.speed = speed
        self.robot.rotation = rotation

    def inDanger(self, hit_front, hit_left, hit_right): # check if there exists some danger because of the obstacles
        if hit_front <= 0. or hit_left <= 0. or hit_right <= 0.: # any of the obstacles are way too close
            self.textbuffer += "Danger!!!;"
            return True
        return False

    def avoidDanger(self, hit_front, hit_left, hit_right): # just a function that makes the robot stop and diverge from the obstacle
        if self.lastDangerRotation != 0: # this means that the robot was already in danger and decided to change its rotation
            self.updateRobot(0.0, self.lastDangerRotation) # we keep rotating in that way to prevent indecisions
        elif hit_left > hit_right:
            self.updateRobot(0.0, self.dangerRotation) # rotate to the left if the obstacle is closer in the right side
            self.lastDangerRotation = self.dangerRotation
        else:
            self.updateRobot(0.0, -self.dangerRotation) # viceversa
            self.lastDangerRotation = -self.dangerRotation

    def avoidHitSmoothly(self, plannedSpeed, plannedRotation, hit_front, hit_left, hit_right):
        # Second filter: I just filter the decided movement to make it smoothly avoid hits according to the sensors.

        # We calculate various relationships that we are going to use to establish the smooth movement
        divideby_front = self.thresh_ignore - self.thresh_front # calculated in real-time in case it changes
        divedeby_side = self.thresh_ignore - self.thresh_side # same here
        relation_front = hit_front / divideby_front
        relation_left = hit_left / divedeby_side
        relation_right = hit_right / divedeby_side
        speed = plannedSpeed
        rotation = plannedRotation
        # First: adjust speed and rotation according to the front sensor, which is the most dangerous
        if relation_front < 1:
            self.textbuffer += "Obstacle front: " + str(relation_front) + ";"
            speed = plannedSpeed * relation_front + self.epsilon_speed # if no epsilon_speed, it may happen that it won't move
            # Second: adjust speed and rotation according to side sensors
        if relation_left < 1: # the obstacles on the left side are close
            self.textbuffer += "Obstacle left: " + str(relation_left) + ";"
            rotation += - self.baseRotation / (relation_left + self.epsilon_rotation) # This means: the closer you are, the more you rotate
            # TODO: create functions that are completely smooth, these ones are smooth but have discontinuities when plotted with the danger mode functions
        if relation_right < 1: # the obstacles on the right side are close
            self.textbuffer += "Obstacle right: " + str(relation_right) + ";"
            rotation += self.baseRotation / (relation_right + self.epsilon_rotation)
        # IMPORTANT NOTE: if the obstacles are equally close in both sides, it decides to go to the front, it does not slow down

        self.updateRobot(speed, rotation) # the actual movement and series of updates

    def readSensors(self):
        sensors_front = min([s.estimatedDistanceToHit() for s in self.robot.frontsensors])
        sensors_left = min([s.estimatedDistanceToHit() for s in self.robot.leftsensors])
        sensors_right = min([s.estimatedDistanceToHit() for s in self.robot.rightsensors])

        # We calculate the difference between the sensor's readings and out thresholds
        hit_front = sensors_front - self.thresh_front
        hit_left = sensors_left - self.thresh_side
        hit_right = sensors_right - self.thresh_side
        return hit_front, hit_left, hit_right

    def nextMove(self):

        self.textbuffer = "" # we clean the buffer before anything

        # Before deciding, we need to read the sensors
        hit_front, hit_left, hit_right = self.readSensors()

        # First filter: we check if the robot is too close to the obstacle, since this is an uncomfortable situation we want the robot to avoid the obstable and we do not care about its plans. This basically means that the robot stops and turns around until it feels comfortable.
        if self.inDanger(hit_front, hit_left, hit_right):
            self.avoidDanger(hit_front, hit_left, hit_right)
        else:
            # Second filter: provided by the following function
            self.avoidHitSmoothly(self.baseSpeed, 0.0, hit_front, hit_left, hit_right) # just moves forward

class SmoothWanderer(AvoidHitPlanner):

    def __init__(self, robot, baseRotation = 0.2, baseSpeed = 50., dangerRotation = 0.6, epsilon_rotation = 0.01, epsilon_speed = 5., thresh_front = 18., thresh_side = 14., thresh_ignore = 100., minRotation = 0.0, maxRotation = 0.5, thresh_distance = 100.):
        super(SmoothWanderer, self).__init__(robot = robot, baseRotation = baseRotation, baseSpeed = baseSpeed, dangerRotation = dangerRotation, epsilon_rotation = epsilon_rotation, epsilon_speed = epsilon_speed, thresh_front = thresh_front, thresh_side = thresh_side, thresh_ignore = thresh_ignore)

        # Boundaries to decide the rotation randomly
        self.minRotation = minRotation
        self.maxRotation = maxRotation

        # Specific planner treshold
        self.thresh_distance = thresh_distance # travelled distance threshold, once surpassed, the robot changes its direction

        # Specific planner memory attributes
        self.lastIdealSpeed = 0.0 # last ideal speed according to the planner
        self.lastIdealRotation = 0.0 # last ideal rotation according to the planner
        self.currentDistance = 0.0 # current travelled distance
        self.lastPosition = self.robot.v_pos_odometry # current position according to the robot's measurements

    def updateRobot(self, speed, rotation): # this function is not really that necessary (just to improve the looks of the code)
        self.robot.speed = speed
        self.robot.rotation = rotation
        self.currentDistance += np.linalg.norm(self.robot.v_pos_odometry - self.lastPosition) # we update the distance
        self.lastPosition = np.copy(self.robot.v_pos_odometry) # then we update the new position

    def nextMove(self):

        self.textbuffer = "" # we clean the buffer before anything

        # Before deciding, we need to read the sensors
        hit_front, hit_left, hit_right = self.readSensors()

        # First: check danger
        if self.inDanger(hit_front, hit_left, hit_right):
            self.avoidDanger(hit_front, hit_left, hit_right)
            self.lastIdealSpeed = 0. # there is no longer ideal speed nor rotation after a dangerous situation
            self.lastIdealRotation = 0.
        else:
            # Second: decide without danger
            if self.currentDistance >= self.thresh_distance or self.lastIdealSpeed == 0.0:
                # This is the case in which the robot has been doing the same thing for some time (distance) or the robot was not moving
                self.currentDistance = 0.0 # we restart the distance counter
                rotation = r.uniform(self.minRotation, self.maxRotation) # random rotation
                if r.random() <= 0.5: # 1/2 probability to rotate left or right
                    rotation = -rotation # to the left
                self.lastIdealSpeed = self.baseSpeed
                self.lastIdealRotation = rotation
                self.avoidHitSmoothly(self.baseSpeed, rotation, hit_front, hit_left, hit_right)
            else:
                # In this case, we use the robot's memory so that it continues wandering the way it was doing for a little longer
                self.avoidHitSmoothly(self.lastIdealSpeed, self.lastIdealRotation, hit_front, hit_left, hit_right)

rel_4nbs = np.array([[1,0],[0,1],[-1,0],[0,-1]]) # relative neighbours for 4-neighbour problems
rel_nbs_diag = np.array([[1,1],[-1,1],[-1,-1],[1,-1]]) # just the diagonals
diagonal = 1.4141 # diagonal distance

class GridPlanner(AvoidHitPlanner):

    def __init__(self, robot, baseRotation = 0.2, baseSpeed = 50., dangerRotation = 0.6, epsilon_rotation = 0.01, epsilon_speed = 5., thresh_front = 18., thresh_side = 14., thresh_ignore = 100., radius_objective = 30., grid_topleft = np.array([0.,0.]), grid_shape = (20,20), grid_side_length = 20., start = (0,0), objective = (0,0)):
        super(GridPlanner, self).__init__(robot = robot, baseRotation = baseRotation, baseSpeed = baseSpeed, dangerRotation = dangerRotation, epsilon_rotation = epsilon_rotation, epsilon_speed = epsilon_speed, thresh_front = thresh_front, thresh_side = thresh_side, thresh_ignore = thresh_ignore)
        self.radius_objective = radius_objective
        self.grid_coordsX = np.array([grid_topleft[0] + grid_side_length * i for i in range(0,grid_shape[0])])
        self.grid_coordsY = np.array([grid_topleft[1] + grid_side_length * j for j in range(0,grid_shape[1])])
        self.grid_graph = nx.Graph()
        for i in range(grid_shape[0]):
            for j in range(grid_shape[1]):
                if not self.gridObstacle((i,j)):
                    self.grid_graph.add_node((i,j)) # the nodes must be created first
        for i in range(grid_shape[0]):
            for j in range(grid_shape[1]):
                # The library NetworkX can be asked to add edges even if they do not exist, thus making the task easier
                self.grid_graph.add_edge((i,j), (i-1,j), weight = 1.)
                self.grid_graph.add_edge((i,j), (i,j-1), weight = 1.)
                self.grid_graph.add_edge((i,j), (i-1,j-1), weight = 1.4141)
                self.grid_graph.add_edge((i,j), (i+1,j-1), weight = 1.4141)
        def eucdist(a,b):
            a1, a2 = a
            b1, b2 = b
            return math.sqrt((a1 - b1)**2 + (a2 - b2)**2)
        self.path = nx.astar_path(self.grid_graph, start, objective, eucdist) # in ij coordinates referring to the grid_position arrays, it cannot visit the same place twice
        self.currentObj = 0

    def grid2WorldCoords(self, ij):
        return np.array([self.grid_coordsX[ij[0]],self.grid_coordsY[ij[1]]])

    def gridObstacle(self, ij):
        return self.robot.handler.occupiedExcept(self.grid2WorldCoords(ij), self.robot)

    def nextMove(self):

        self.textbuffer = "" # we clean the buffer before anything

        # Before deciding, we need to read the sensors
        hit_front, hit_left, hit_right = self.readSensors()

        # First: check danger
        if self.inDanger(hit_front, hit_left, hit_right):
            self.avoidDanger(hit_front, hit_left, hit_right)
        else:
            # Second: decide without danger using the known path
            if np.linalg.norm(self.robot.space.v_position - self.grid2WorldCoords(self.path[self.currentObj])) < self. radius_objective:
                self.textbuffer += "Weepee!!"
            else:
                v = self.grid2WorldCoords(self.path[self.currentObj]) - self.robot.space.v_position
                rotation = self.robot.orientation - wh.Angle(math.atan2(v[1],v[0]))
                self.avoidHitSmoothly(self.baseSpeed, rotation, hit_front, hit_left, hit_right)
