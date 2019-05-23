#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Neil Pradhan}
# {npradhan@kth.se}


import random
from random import *
import math
from math import *
from numpy import *
from dubins import Car


car = Car()  



def solution(car):
    start = Node(car.x0, car.y0, 0.0, 0.1)
    Goal = Node(car.xt, car.yt, 0, 0)
    controls,times = final_objective(start,Goal,car.xlb,car.xub,car.ylb,car.yub,car.obs)

    return controls, times





class Node():
    def __init__(self,x,y,theta,phi):
        self.x = x
        self.y = y
        self.theta = theta
        self.phi = phi
        self.parent = None

def final_objective(start_point,goal_point,car_xl,car_xu,car_yl,car_yu,car_obs):
    i=0
    node_list = [start_point]
    while True:
        random_point = [uniform(0.0,20.0),uniform(0.0,10.0)]
        i=i+1
        if i==20:
           random_point = [goal_point.x,goal_point.y]
           i=0
        nearest_node = get_nearest_node(random_point,node_list)
        vel_actual = [cos(nearest_node.theta),sin(nearest_node.theta)]
        vel_desired = [(random_point[0] - nearest_node.x),(random_point[1] - nearest_node.y)]
        phi = steering_angle_phi(vel_actual,vel_desired)
        xl = [nearest_node.x]
        yl = [nearest_node.y]
        thetal = [nearest_node.theta]
        c = 0
        while c <= 1:
            xn , yn , thetan = car.step(xl[-1],yl[-1],thetal[-1],phi)
            xl.append(xn)
            yl.append(yn)
            thetal.append(thetan)
            c = c + 0.01  
        if collision(xn,yn,car_obs) ==False:
           new_node = Node(xn,yn,thetan,phi)
           new_node.parent = nearest_node
           node_list.append(new_node)
           if distance(new_node.x,new_node.y,goal_point.x,goal_point.y) <= 0.5:
                print("Goal Reached!")
                break
    l=len(node_list)-1             
    controls = []
    while node_list[l] != node_list[0]:
        controls.append(node_list[l].phi)
        l = node_list.index(node_list[l].parent)

    controls.reverse()
    times = range(0,(len(controls)+1),1)
    return controls,times               

def get_nearest_node(r,node_list):
    d_list = [(sqrt((r[0] - node.x)**2 + (r[1] - node.y)**2)) for node in node_list]
    nearest_node = node_list[d_list.index(min(d_list))]
    return nearest_node
            
            
def steering_angle_phi(delta_x,delta_y):
    delta_x = array(delta_x)
    delta_y = array(delta_y)
    delta_x = [delta_x[0],delta_x[1],0]
    delta_y = [delta_y[0],delta_y[1],0]
    cos_angle = (dot(delta_x,delta_y))/( (linalg.norm(delta_x))*(linalg.norm(delta_y)) )
    angle = acos(cos_angle)
    sign = (dot(cross(delta_x,delta_y),[0,0,1]))/( (linalg.norm(delta_x))*(linalg.norm(delta_y)) )
    if angle < -0.25*pi:
        angle = -0.25*pi
    if angle > 0.25*pi:
        angle = 0.25*pi
    if sign >0:
        angle = angle
    if sign < 0:
        angle = -angle
    return angle            
            
def distance(x1,y1,x2,y2):
    d = sqrt((x2-x1)**2 + (y2-y1)**2)
    return d

def collision(x,y,obs):
    for(obsx,obsy,obsr) in obs:
        d = distance(x,y,obsx,obsy)
        if x < 0.0 or x > 20.0 or y < 0.0 or y > 10.0:
            return True #collision occured
        if obsr == 0.2:
            if d <= 0.8:
                #print("Collision")
                return True
        elif d<= 1.5*obsr:
           # print("Collision")
            return True
    return False     
