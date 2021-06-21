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

#sim.spawn_robot(2, 2, 0, "robot1")
#sim.spawn_robot(5, 7, 0, "robot2")
#add
sim.spawn_robot(4, 4, 0, 1, "part1")
sim.spawn_robot(1, 4, 0, 2, "part2")
sim.spawn_robot(7, 4, 0, 3, "part3")
sim.spawn_robot(7, 2, 0, 4, "part4")
sim.spawn_robot(10, 4, 0, 5, "part5")
sim.spawn_robot(10, 2, 0, 6, "part6")
sim.spawn_robot(7, 7, 1*pi/4, 0, "part0")

border = 5
sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])
#border = 300
#sim.add_rectangle_pixelcoords([border, border], [width-border, height-border])

sim.run()
rospy.spin()
