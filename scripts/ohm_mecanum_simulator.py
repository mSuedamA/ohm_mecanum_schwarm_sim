#!/usr/bin/env python3

# ------------------------------------------------------------
# Author:      Stefan May
# Date:        20.4.2020
# Description: Pygame-based robot simulator with ROS interface
# ------------------------------------------------------------

from re import S
from threading import current_thread
from numpy.core.numeric import False_
import numpy as np
from numpy.lib.polynomial import polyint
import rospy
import pygame
import sys
from std_srvs.srv import SetBool, SetBoolResponse
from robot import Robot
from A_star import A_star, Node
from ohm_mecanum_sim.srv import Spawn, Kill, SpawnRequest, SpawnResponse, KillRequest, KillResponse, Dispersion, DispersionResponse, Assemble, AssembleResponse
from std_msgs.msg import String
import random
from math import pi
import time
import threading

class Ohm_Mecanum_Simulator:

    def __init__(self, surface, rosname, windowtitle):
        self._surface = surface
        self._meter_to_pixel = 100
        self._robots = []
        self._line_segment_obstacles = []
        self._verbose = False
        self._to_crash = False
        self._arrive = []
        self.map_points = [[0,0],[0,10],[16,10],[16,0],[0,0]]
        rospy.init_node(rosname, anonymous=True)
        pygame.display.set_caption(windowtitle)
    
    def __del__(self):
        pass

    def service_callback_spawn(self, req):
        self.spawn_robot(req.x, req.y, req.theta, req.num, req.name)
        response = SpawnResponse(req.x, req.y, req.theta, req.num, req.name)
        return response

    def service_callback_kill(self, req):
        self.kill_robot(req.name)
        response = KillResponse(True)
        return response

    def service_callback_verbose(self, req):
        self._verbose = req.data
        if(self._verbose):
            msg = "Verbosity increased"
        else:
            msg = "Verbosity decreased"
        return SetBoolResponse(True, msg)

    def service_callback_pos(self, req):
        for i in self._robots:
            print(i.get_points())

    def service_callback_dispersion(self, req):
        #destination_coords = [[7, 4, pi/2], [6, 5, pi/2]]
        destination_coords = [[7, 4, pi/2], [6, 5, pi/2], [3, 4, pi/2], [7, 6, pi/2], [7, 2, pi/2], [5, 3, pi/2], [9, 5, pi/2]]
        #destination_coords = [[7, 2, pi/2], [36, 7, pi/2], [5, 4, pi/2], [7, 6, pi/2], [4, 8, pi/2], [3, 3, pi/2], [9, 5, pi/2]]
        goal_points = []
        for i in range(0, len(destination_coords)):
            self._arrive[i] = False
            goal_points.append(self._robots[i].get_points())
            goal_distance = [destination_coords[i][0]-self._robots[i]._coords[0],destination_coords[i][1]-self._robots[i]._coords[1]]
            for j in goal_points[i]:
                j[0] += goal_distance[0]
                j[1] += goal_distance[1]
        for i in range(0, len(destination_coords)):
            if self.check_collision(goal_points[i],self.map_points):
                print("Can't reach this destination: Out of the map!")
                return
            for j in range(i+1, len(destination_coords)):
                if self.check_collision(goal_points[i],goal_points[j]):
                    print("Can't reach this destination: Some parts overlap!")
                    return
        threads = []
        for i in range(0, len(destination_coords)):
            threads.append(threading.Thread(target=self.move_to_point, args=(i,destination_coords[i])))
        i = len(destination_coords)-1
        while(i>=0):
            #self.move_to_point(i,destination_coords)
            threads[i].start()
            i -= 1
        for i in range(0, len(destination_coords)):
            threads[i].join()
        #self.move_to_point(destination_coords, -1)
        response = DispersionResponse()
        return response

    def service_callback_assemble(self, req):
        #destination_coords = [[5.5, 4.5, pi/2], [5, 4, pi/2]]
        destination_coords = [[7, 4.5, pi/2], [6.5, 4, pi/2], [8.5, 4, pi/2], [7.5, 5, pi/2], [7, 3.5, pi/2], [6, 4, pi/2], [8, 3.5, pi/2]]
        threads = []
        for i in range(0, len(destination_coords)):
            threads.append(threading.Thread(target=self.move_to_point, args=(i,destination_coords[i])))
            threads[i].start()
        for i in range(0, len(destination_coords)):
            threads[i].join()
        
        #self.move_to_point(destination_coords, 1)
        response = AssembleResponse()
        return response

    def move_step(self, robot, path):
        step = len(path)
        i = step-1
        while(i>=0):
            move = [path[i][0]-robot._coords[0],path[i][1]-robot._coords[1]]
            if(move[0]==0 and move[1]==0): 
                i = i-1
                continue
            #print(robot._num,":",move)
            points = robot.get_points()
            for j in range(0,len(points)):
                points[j][0] += move[0]
                points[j][1] += move[1]
            for r in self._robots:
                if r == robot : continue
                elif self.check_collision(points,r.get_points()) == True:
                    time.sleep(0.5)
                    if self.check_collision(points,r.get_points()) == True: 
                        print(robot._num,"can't move",move)
                        return
            robot.step_move(move)
            i = i-1
    
    def adjust(self, robot, destination):
        p_distance = []
        min_dis = 999
        for r in self._robots:
            if r == robot:
                p_distance.append(999)
                continue
            dx = destination[0]-r._coords[0]
            dy = destination[1]-r._coords[1]
            p_distance.append(abs(dx)+abs(dy))
            min_dis = min(abs(dx)+abs(dy),min_dis)
        for i in range(0,len(self._robots)):
            if min_dis == p_distance[i]:
                if(self._arrive[i]==False): 
                    time.sleep(1)
                    return
                des_i = [self._robots[i]._coords[0],self._robots[i]._coords[1]]
                i_points = self._robots[i].get_points()
                randdes = [0, 0]
                while(True):
                    randdes = [random.randint(-2,2),random.randint(-2,2)]
                    for j in i_points:
                        j[0] += randdes[0]
                        j[1] += randdes[1]
                    if self.check_collision(i_points,self.map_points): continue
                    for j in self._robots:
                        if j._num == i: continue
                        if self.check_collision(i_points,j.get_points()): continue
                    break
                self.move_to_point(i,[des_i[0]+randdes[0],des_i[1]+randdes[1]])
                self.move_to_point(robot._num,destination)
                self.move_to_point(i,des_i)

    def move_to_point(self, i, destination_coords):
        r=self._robots[i]
        fail_time = 0
        if len(destination_coords)<3 : destination_coords.append(pi/2)
        while(True):
            dx = destination_coords[0]-r._coords[0]
            dy = destination_coords[1]-r._coords[1]
            dz = destination_coords[2]-r._theta
            if (abs(dx)<0.05 and abs(dy)<0.05 and abs(dz)<0.01 ): 
                self._arrive[i] = True
                print(i,": arrive!")
                return
            R = A_star(r,destination_coords,self._robots)
            if(R.start()):
                road = R.pathlist
                self.move_step(r,road)
            else:
                fail_time += 1
                if(fail_time>=7):
                    print(i,": it may need more adjust!")
                    return
                if(fail_time>=5):
                    self.adjust(r,destination_coords)
                    continue
                time.sleep(0.1)


    def move_to_point_origin(self, destination_coords, queue):
        while(True):
            arrive = True
            if(queue == 1): i = 0
            if(queue == -1): i = len(self._robots)-1
            while(i>=0 and i<=len(self._robots)-1):
                r=self._robots[i]
                dx = destination_coords[r._num][0]-r._coords[0]
                dy = destination_coords[r._num][1]-r._coords[1]
                dz = destination_coords[r._num][2]-r._theta
                if (abs(dx)>0.05 or abs(dy)>0.05 or abs(dz)>0.01 ):
                    arrive = False
                    R = A_star(r,destination_coords[i],self._robots)
                    if(R.start()):
                        road = R.pathlist
                        self.move_step(r,road)
                i += queue
            if arrive == True: 
                return
            
        while(False):
            arrive = True
            for r in self._robots:
                dx = destination_coords[r._num][0]-r._coords[0]
                dy = destination_coords[r._num][1]-r._coords[1]
                dz = destination_coords[r._num][2]-r._theta
                if (abs(dx)>0.05 or abs(dy)>0.05 or abs(dz)>0.01 ):
                    arrive = False
                    ##################################################
                    #check collision

                    x,y,z = 0,0,0
                    if(dx!=0): x = min(abs(dx), 1)*dx/abs(dx)
                    if(dy!=0): y = min(abs(dy), 1)*dy/abs(dy)
                    if(dz!=0): z = min(abs(dz), 5)*dz/abs(dz)
                    r.step_move(x, y, z)

                    while(False and self.check_new_pos(r)):
                        r.step_move(-dx1, -dy1, -dz1)
                        dz1 = dz1 + pi/60
                        if((dz1-dz)>=pi): 
                            print("stop")
                            return
                        r.step_move(dx1, dy1, dz1)
            if(arrive): 
                print("succeed")
                return
    
    def spawn_robot(self, x, y, theta, num, name):
        self._robots.append(Robot(x, y, theta, num, name))
        self._arrive.append(True)

    def kill_robot(self, name):
        for r in self._robots:
            if(r._name == name):
                r.stop()
                self._robots.remove(r)

    def add_line_segment_pixelcoords(self, coords1, coords2):
        line_segment = (self.transform_to_robotcoords(coords1), self.transform_to_robotcoords(coords2))
        self.add_line_segment_obstacle(line_segment)

    def add_rectangle_pixelcoords(self, coords1, coords2):
        line_segment = (self.transform_to_robotcoords([coords1[0], coords1[1]]), self.transform_to_robotcoords([coords1[0], coords2[1]]))
        self.add_line_segment_obstacle(line_segment)
        line_segment = (self.transform_to_robotcoords([coords1[0], coords2[1]]), self.transform_to_robotcoords([coords2[0], coords2[1]]))
        self.add_line_segment_obstacle(line_segment)
        line_segment = (self.transform_to_robotcoords([coords2[0], coords2[1]]), self.transform_to_robotcoords([coords2[0], coords1[1]]))
        self.add_line_segment_obstacle(line_segment)
        line_segment = (self.transform_to_robotcoords([coords2[0], coords1[1]]), self.transform_to_robotcoords([coords1[0], coords1[1]]))
        self.add_line_segment_obstacle(line_segment)

    def add_line_segment_obstacle(self, line_segment):
        self._line_segment_obstacles.append(line_segment)

    def transform_to_pixelcoords(self, coords):
        pixelcoords  = [ coords[0] * self._meter_to_pixel,
                        (self._surface.get_height() - coords[1] * self._meter_to_pixel) ]
        return pixelcoords

    def transform_to_robotcoords(self, coords):
        pixelcoords  = [ coords[0] / self._meter_to_pixel,
                         (-coords[1] + self._surface.get_height()) / self._meter_to_pixel]
        return pixelcoords

    def exit_simulation(self):
        print("Exit simulation")
        for r in self._robots:
            r.stop()
            del r
        sys.exit()

    def get_distance(self, r):
        # Determine distances to other robots
        dist_to_obstacles  = []
        for obstacle in self._robots:
            if(obstacle != r):
                obstacle_points = obstacle.get_points()
                for i in range(1, len(obstacle_points)):
                    dist_to_obstacles = r.get_distance_to_line_obstacle(obstacle_points[i-1], obstacle_points[i],  dist_to_obstacles)
                #obstacle_coords = obstacle.get_coords()
                #dist_to_obstacles = r.get_distance_to_circular_obstacle(obstacle_coords, obstacle.get_obstacle_radius(),  dist_to_obstacles)
        # Determine distances to line segments
        for obstacle in self._line_segment_obstacles:
            dist_to_obstacles = r.get_distance_to_line_obstacle(obstacle[0], obstacle[1], dist_to_obstacles)
        return dist_to_obstacles        

    def cross(self, p1,p2,p3):
        x1=p2[0]-p1[0]
        y1=p2[1]-p1[1]
        x2=p3[0]-p1[0]
        y2=p3[1]-p1[1]
        return x1*y2-x2*y1 

    def IsIntersec(self, p1,p2,p3,p4):
        if(max(p1[0],p2[0])>=min(p3[0],p4[0]) and max(p3[0],p4[0])>=min(p1[0],p2[0]) and max(p1[1],p2[1])>=min(p3[1],p4[1]) and max(p3[1],p4[1])>=min(p1[1],p2[1])):
            if(self.cross(p1,p2,p3)*self.cross(p1,p2,p4)<=0 and self.cross(p3,p4,p1)*self.cross(p3,p4,p2)<=0):
                return True
        return False

    def check_collision(self, points1, points2):
        for i in range(1, len(points1)):
            for j in range(1, len(points2)):
                if self.IsIntersec(points1[i-1],points1[i],points2[j-1],points2[j]):
                    return True
        return False

    #def check_collision(self, r, obstacle):
        dist_to_obstacles  = self.get_distance(r)
        min_dist = 9999
        for i in range(0, len(dist_to_obstacles)):
            if(dist_to_obstacles[i]<min_dist):
                min_dist = dist_to_obstacles[i]
        if(min_dist<0):
                print(dist_to_obstacles)
                print("Crash!")
                return True
        return False
        
    def run(self):
        bg_color = (64, 64, 255)
        rospy.Service('/spawn', Spawn, self.service_callback_spawn)
        rospy.Service('/kill', Kill, self.service_callback_kill)
        rospy.Service('/verbose', SetBool, self.service_callback_verbose)
        rospy.Service('/dispersion', Dispersion, self.service_callback_dispersion)
        rospy.Service('/assemble', Assemble, self.service_callback_assemble)
        rospy.Service('/pos', Assemble, self.service_callback_pos)
        rate = rospy.Rate(50)

        clock = pygame.time.Clock()
        clock.tick(360)

        while 1:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit_simulation()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_c and pygame.key.get_mods() & pygame.KMOD_CTRL:
                        self.exit_simulation()

            self._surface.fill(bg_color)
            
            # Draw obstacles
            for obstacle in self._line_segment_obstacles:
                pixel_segment_start = self.transform_to_pixelcoords(obstacle[0])
                pixel_segment_end = self.transform_to_pixelcoords(obstacle[1])
                pygame.draw.line(self._surface, pygame.Color(0, 0, 0), pixel_segment_start, pixel_segment_end, 3)
        
            # Convert robot coordinates for displaying all entities in pixel coordinates
            for r in self._robots:

                r.acquire_lock()
                # Draw robot symbol
                coords      = r.get_coords()
                pixel_robot = self.transform_to_pixelcoords(coords)
                rect        = r.get_rect()
                rect.center = pixel_robot
                rect.move(pixel_robot)
                self._surface.blit(r.get_image(), rect)

                pos_sensor = r.get_pos_tof()
                pos_hitpoint = r.get_far_tof()

                # Determine distances to other robots
                dist_to_obstacles  = self.get_distance(r)

                r.publish_tof(dist_to_obstacles)

                r.release_lock()

                min_dist = 9999
                for i in range(0, len(dist_to_obstacles)):
                    if(dist_to_obstacles[i]<min_dist):
                        min_dist = dist_to_obstacles[i]
                if(min_dist<0):
                    print(dist_to_obstacles)
                    print("Crash!")
                    #r.reset_pose()
                elif (r._coords[0] < 0 or r._coords[1] < 0 or r._coords[0] > self._surface.get_width()/self._meter_to_pixel or r._coords[1] > self._surface.get_height()/self._meter_to_pixel):
                    #print(dist_to_obstacles)
                    print("Crash!")
                    r.reset_pose()

                # Draw ToF beams
                pos_hitpoint = r.get_hit_tof(dist_to_obstacles)
                for i in range(0,r.get_tof_count()):
                    pixel_sensor = self.transform_to_pixelcoords(pos_sensor[i])
                    pixel_hitpoint = self.transform_to_pixelcoords(pos_hitpoint[i])
                    pygame.draw.line(self._surface, pygame.Color(255, 0, 0), pixel_sensor, pixel_hitpoint)          

            pygame.display.update()

            rate.sleep()
