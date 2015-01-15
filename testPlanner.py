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
Just tests for the wanderer behavior. This is needed so that the robot explores the map.
"""

import pygame
from pygame.locals import *
import sys
import worldHandling as wh
import robots as bot
import planner as pl
import numpy as np
import math

import pdb

# WORLD PARAMETERS

handler = wh.ScenarioHandler()
outsidewalls = [wh.PhysicalEntity(wh.RectangularVolume(np.array([-400.,300.]), np.array([-390., -300.])), handler), wh.PhysicalEntity(wh.RectangularVolume(np.array([-389.,300.]), np.array([389., 290.])), handler), wh.PhysicalEntity(wh.RectangularVolume(np.array([390.,300.]), np.array([400., -300.])), handler), wh.PhysicalEntity(wh.RectangularVolume(np.array([-389.,-290.]), np.array([389., -300.])), handler)]
rects = outsidewalls + [wh.PhysicalEntity(wh.RectangularVolume(np.array([200.,10.]), np.array([379.,0.])), handler), wh.PhysicalEntity(wh.RectangularVolume(np.array([190.,240.]), np.array([199.,0.])), handler), wh.PhysicalEntity(wh.RectangularVolume(np.array([0.,250.]), np.array([199.,241.])), handler)]
circles = [wh.PhysicalEntity(wh.CircularVolume(np.array([-50.,50.]), 10.), handler), wh.PhysicalEntity(wh.CircularVolume(np.array([-50.,100.]), 10.), handler), wh.PhysicalEntity(wh.CircularVolume(np.array([-100.,50.]), 10.), handler), wh.PhysicalEntity(wh.CircularVolume(np.array([-200.,200.]), 10.), handler), wh.PhysicalEntity(wh.CircularVolume(np.array([-200.,50.]), 10.), handler), wh.PhysicalEntity(wh.CircularVolume(np.array([-50.,200.]), 10.), handler)]

# ROBOT PARAMETERS

robot_pos = np.array([-100.,200])
robot_orient = wh.Angle(1.)
robot = bot.Robot(robot_pos, 10., robot_orient, handler, bot.percentageError(0.1, "mixed"), bot.percentageError(0.1, "mixed"))
robotList = [robot]
leftsensors = [bot.DiscreteBeamSensor(robot_pos, wh.Angle(0.6, robot_orient), 4., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot]), bot.DiscreteBeamSensor(robot_pos, wh.Angle(1., robot_orient), 4., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot])]
rightsensors = [bot.DiscreteBeamSensor(robot_pos, wh.Angle(-1., robot_orient), 4., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot]), bot.DiscreteBeamSensor(robot_pos, wh.Angle(-0.6, robot_orient), 4., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot])]
frontsensors = [bot.DiscreteBeamSensor(robot_pos, wh.Angle(0.2, robot_orient), 4., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot]), bot.DiscreteBeamSensor(robot_pos, wh.Angle(-0.2, robot_orient), 4., 200., bot.expErrorWithDistance(0.015), handler, exceptions = [robot])]
robot.frontsensors = frontsensors
robot.leftsensors = leftsensors
robot.rightsensors = rightsensors
myplanner = pl.GridPlanner(robot)

# UPDATE WORLD PARAMETERS

handler.staticObjects = rects + circles
handler.dynamicObjects = robotList

# SCREEN-RELATED PARAMETERS
# All measured in pixels and 8-bit RGB vectors

width = 800
height = 600

# Colors that I am going to use
c_occupied = (0, 0, 0)
c_bkg = (255, 255, 255)
c_beam = (0, 0, 255)
c_text = (200, 100, 0)
c_ghost = (0, 255, 0)
c_robot = (255, 0, 0)

# Text font
font = "monospace"
fontsize = 10

# Text position
text_pos = [10., 10.]

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
    for s in leftsensors + rightsensors + frontsensors:
        ini = world2screen(s.v_position)
        end = world2screen(s.updateHitPoint())
        pygame.draw.line(screen, c_beam, ini, end)

def draw_text_stuff():
    lines = myplanner.textbuffer.split(';')
    for i in range(0, len(lines)):
        t = text_renderer.render(lines[i], False, c_text)
        screen.blit(t, (text_pos[0],fontsize * i + text_pos[1]))

# FUNCTIONS TO HANDLE MOTION

def reset_robot_stuff():
    for r in robotList:
        r.speed = 0.0
        r.rotation = 0.0

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
    keyboard_status = pygame.key.get_pressed()
    if keyboard_status[K_ESCAPE]:
        pygame.quit()
        sys.exit(0)
    pygame.event.pump() # clean input
    # First: clean previous manipulations before any sort of decision with the planner
    reset_robot_stuff()
    myplanner.nextMove()
    # Second: update scene
    elapsed_time = clock.tick(60) # limit frames to 60
    handler.updateAll(elapsed_time) # we let the handler do this "update" here
    # Third: redraw all changes
    screen.fill(c_bkg) # background
    draw_world_stuff() # first layer: real things (aka world)
    draw_virtual_stuff() # second layer: virtual UI aids
    draw_text_stuff() # third layer: text
    pygame.display.update()
