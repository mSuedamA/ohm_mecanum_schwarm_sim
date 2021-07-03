#!/usr/bin/env python3

# ------------------------------------------------------------
# Author:      Stefan May
# Date:        20.4.2020
# Description: Pygame-based robot simulator application
# ------------------------------------------------------------

import pygame
import rospy
from ohm_mecanum_simulator import Ohm_Mecanum_Simulator
from math import pi

pygame.init()

# Screen size
size = width, height = 1200, 900

# Drawing surface
surface = pygame.display.set_mode(size, pygame.HWSURFACE | pygame.DOUBLEBUF)

sim = Ohm_Mecanum_Simulator(surface, "ohm_mecanum_sim", "Ohm Mecanum Simulator")

#add
sim.spawn_robot(7.5, 6.5, 1*pi/2, 0, "part0")
sim.spawn_robot(6.5, 5.5, 1*pi/2, 1, "part1")
sim.spawn_robot(8.5, 5.5, 1*pi/2, 2, "part2")
sim.spawn_robot(7, 6, 1*pi/2, 3, "part3")
sim.spawn_robot(7, 5, 1*pi/2, 4, "part4")
sim.spawn_robot(6, 5.5, 1*pi/2, 5, "part5")
sim.spawn_robot(8, 5, 1*pi/2, 6, "part6")


border = 5
sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])
#border = 300
#sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])

sim.run()
rospy.spin()
