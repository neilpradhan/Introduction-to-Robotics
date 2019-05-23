#!/usr/bin/env python3

"""
    # {student full name}Neil Pradhan
    # {student id}
    # {student email}npradhan@kth.se
"""

# Python standard library
from math import cos, sin, atan2, fabs

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()


        """
        Fill in your solution here
        """
        
        x=pose.pose.position.x
        y=pose.pose.position.y
        z=pose.pose.position.z
        
        minimum = scan.range_min
        maximum = scan.range_max
        ranges = scan.ranges 
        b=scan.angle_min -scan.angle_increment
        save_x =[]
        save_y = []
        list_of_tiles=[]
        list_x = []
        list_y =[]
        l=[]
        for range1 in ranges:     
                b = b+scan.angle_increment
                chk_min =range1>minimum
                chk_max = range1<maximum
                if (chk_min & chk_max):
                        
                    #laser frame
                    px=range1*cos(b)
                    py=range1*sin(b)
                    #world frame
                    wx = range1*(cos(robot_yaw+b))+x
                    wy = range1*(sin(robot_yaw+b))+y
                        
                    #map frame
               
                        
                    mx = wx-origin.position.x
                    my = wy-origin.position.y
                        
                    index_x = int(mx/resolution)
                    index_y = int(my/resolution)
                    
                    l.append((index_x,index_y))
                    
                    
                    save_x.append(index_x)
                    save_y.append(index_y)
                    
                    start_x = int((x-origin.position.x)/resolution)
                    start_y = int((y-origin.position.y)/resolution)
                    end = (index_x,index_y)
                    start = (start_x,start_y)
                    
                    list_of_tiles=self.raytrace(start,end)
                    
                    for j in range(len(list_of_tiles)):
                        x_tiles = list_of_tiles[j][0]    
                        y_tiles = list_of_tiles[j][1]
                        list_x.append(x_tiles)
                        list_y.append(y_tiles)
                        self.add_to_map(grid_map,x_tiles,y_tiles,self.free_space)
                        
        for i  in range(len(l)): 
            x_1=l[i][0]
            y_1=l[i][1]             
            self.add_to_map(grid_map,x_1,y_1,self.occupied_space)    
        
                           
          
                
                
        xmax=max(max(save_x), max(list_x))
        xmin=min(min(save_x), min(list_x))
        ymin=min(min(save_y), min(list_y))
        ymax=max(max(save_y), max(list_y))

        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = xmin
        # The minimum y index in 'grid_map' that has been updated
        update.y = ymin
        # Maximum x index - minimum x index + 1
        update.width = xmax-xmin+1
        # Maximum y index - minimum y index + 1
        update.height = ymax-ymin+1
        # The map data inside the rectangle, in row-major order.
        update.data = []
        
        for y in range(ymin,ymax+1):
            for x in range(xmin,xmax+1):
                update.data.append(grid_map[x, y])
        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """
        for x in range(grid_map.get_width()):
            for y in range(grid_map.get_height()):
                if grid_map[x,y]==self.occupied_space:
                    for i in range(x-self.radius,x+self.radius+1):
                        for j in range(y-self.radius,y+self.radius+1):
                            if (grid_map[i,j] != self.occupied_space) and ((x-i)**2+(y-j)**2)**(0.5)<=self.radius :
                                self.add_to_map(grid_map,i,j,self.c_space)
                    
        
        # Return the inflated map
        return grid_map
