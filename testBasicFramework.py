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
Just tests for the sake of testing.
"""

import pygame
from pygame.locals import *
import sys
import worldHandling as wh
import robots as bot
import numpy as np
import math

import pdb

# WORLD PARAMETERS

handler = wh.ScenarioHandler()
rects = [wh.PhysicalEntity(wh.RectangularVolume(np.array([50.,10.]), np.array([80., -10.])), handler)]
circles = [wh.PhysicalEntity(wh.CircularVolume(np.array([50.,50.]), 20.), handler)]

# ROBOT PARAMETERS

robot_pos = np.array([0.,0.])
robot_orient = wh.Angle(0.)
robot_pos2 = np.array([-50.,0.])
robot_orient2 = wh.Angle(2.)
robot = bot.Robot(robot_pos, 10., robot_orient, handler, bot.percentageError(0.01, "mixed"), bot.percentageError(0.01, "mixed"))
robot2 = bot.Robot(robot_pos2, 10., robot_orient2, handler, bot.percentageError(0.5, "mixed"), bot.percentageError(0.5, "mixed"))
robotList = [robot, robot2]
sensors = [bot.DiscreteBeamSensor(robot_pos, wh.Angle(-1., robot_orient), 2., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot]), bot.DiscreteBeamSensor(robot_pos, wh.Angle(-0.6, robot_orient), 2., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot]), bot.DiscreteBeamSensor(robot_pos, wh.Angle(-0.2, robot_orient), 2., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot]), bot.DiscreteBeamSensor(robot_pos, wh.Angle(0.2, robot_orient), 2., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot]), bot.DiscreteBeamSensor(robot_pos, wh.Angle(0.6, robot_orient), 2., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot]), bot.DiscreteBeamSensor(robot_pos, wh.Angle(1., robot_orient), 2., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot]), bot.DiscreteBeamSensor(robot_pos2, wh.Angle(-0.4, robot_orient2), 2., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot2]), bot.DiscreteBeamSensor(robot_pos2, robot_orient2, 2., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot2]), bot.DiscreteBeamSensor(robot_pos2, wh.Angle(0.4, robot_orient2), 2., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot2])]

# UPDATE WORLD PARAMETERS

handler.staticObjects = rects + circles
handler.dynamicObjects = robotList

# SCREEN-RELATED PARAMETERS
# All measured in pixels and 8-bit RGB vectors

width = 800
height = 800

# Colors that I am going to use
c_occupied = (0, 0, 0)
c_bkg = (255, 255, 255)
c_beam = (0, 0, 255)
c_indicator = (255, 0, 255)
c_ghost = (0, 255, 0)
c_robot = (255, 0, 0)

# Text font
font = "monospace"
fontsize = 10

# Camera parameters in world units
camera_pos = [0., 0.]
camera_scale = 1. # world units per pixel

# FUNCTIONS FOR CAMERA-SCREEN CONVERSIONS

def worldunits2screenunits(units): # convert from world units to screen units (aka pixels)
    return int(units / camera_scale)

def world2screen(coords): # from world coords to screen coords conversion (XY to ij)
    topleft_pos = camera_pos + np.array([-width/2. * camera_scale, height/2. * camera_scale]) # it is calculated in real time in case the camera moves
    return [int((coords[0] - topleft_pos[0]) / camera_scale), int(-(coords[1] - topleft_pos[1]) / camera_scale)]

# FUNCTIONS FOR ACTUAL DRAWING USING PYGAME

def draw_world_stuff(): # draw in screen coords
    for rect in rects:
        corner1, corner2 = rect.space.boundaries()
        corner1 = world2screen(corner1)
        corner2 = world2screen(corner2)
        pygame.draw.rect(screen, c_occupied, [corner1[0], corner1[1], corner2[0] - corner1[0], corner2[1] - corner1[1]])
    for circ in circles:
        circ = circ.space
        pygame.draw.circle(screen, c_occupied, world2screen(circ.v_position), worldunits2screenunits(circ.radius))
    for r in robotList:
        robot_pos = world2screen(r.space.v_position)
        robot_dir_v = world2screen(np.array([math.cos(r.orientation.getRadians()), math.sin(r.orientation.getRadians())]) * r.space.radius + r.space.v_position)
        pygame.draw.circle(screen, c_robot, robot_pos, worldunits2screenunits(r.space.radius))
        pygame.draw.line(screen, (0,0,0), robot_pos, robot_dir_v)

def draw_virtual_stuff():
    for s in sensors:
        ini = world2screen(s.v_position)
        end = world2screen(s.lastHitPoint)
        est = world2screen(np.array([math.cos(s.orientation.getRadians()), math.sin(s.orientation.getRadians())]) * s.estimatedDistanceToHit() + s.v_position)
        pygame.draw.line(screen, c_beam, ini, end)
        pygame.draw.circle(screen, c_indicator, est, 2)
    for r in robotList:
        ghost_pos = world2screen(r.v_pos_odometry)
        ghost_dir_v = world2screen(np.array([math.cos(r.orient_compass.getRadians()), math.sin(r.orient_compass.getRadians())]) * r.space.radius + r.v_pos_odometry)
        pygame.draw.circle(screen, c_ghost, ghost_pos, worldunits2screenunits(r.space.radius), 1)
        pygame.draw.line(screen, c_ghost, ghost_pos, ghost_dir_v)

def draw_text_stuff():
    text_robot_coords = text_renderer.render("Robot1: " + str(robot.space.v_position), False, (0,0,0))
    screen.blit(text_robot_coords, (0,0))

# FUNCTIONS TO HANDLE MOTION

def reset_robot_stuff():
    robot.speed = 0.0
    robot.rotation = 0.0
    robot2.speed = 0.0
    robot2.rotation = 0.0

# MAIN EXECUTION THREAD

# Pygame initiation ritual
pygame.init()
pygame.display.init()
pygame.font.init()
screen = pygame.display.set_mode((width,height))
text_renderer = pygame.font.SysFont(font, fontsize)
clock = pygame.time.Clock()
clock.tick() # start the timer

# Draw layer by layer
screen.fill(c_bkg) # background
draw_world_stuff() # first layer: real things (aka world)
draw_virtual_stuff() # second layer: virtual UI aids
draw_text_stuff() # third layer: text
pygame.display.update() # part of the ritual

# Main loop
while True:
    # First: check input, but before, clean previous manipulations
    reset_robot_stuff()
    keyboard_status = pygame.key.get_pressed()
    if keyboard_status[K_ESCAPE]:
        pygame.quit()
        sys.exit(0)
    if keyboard_status[K_UP]:
        robot.speed = 30.
    if keyboard_status[K_DOWN]:
        robot.speed = -30.
    if keyboard_status[K_LEFT]:
        robot.rotation = 1.
    if keyboard_status[K_RIGHT]:
        robot.rotation = -1.
    if keyboard_status[K_w]:
        robot2.speed = 30
    if keyboard_status[K_s]:
        robot2.speed = -30
    if keyboard_status[K_a]:
        robot2.rotation = 1.
    if keyboard_status[K_d]:
        robot2.rotation = -1.
    pygame.event.pump() # clean input
    # Second: update scene
    elapsed_time = clock.tick()
    handler.updateAll(elapsed_time) # we let the handler do this "update" here
    # Third: redraw all changes
    screen.fill(c_bkg) # background
    draw_world_stuff() # first layer: real things (aka world)
    draw_virtual_stuff() # second layer: virtual UI aids
    draw_text_stuff() # third layer: text
    pygame.display.update()
