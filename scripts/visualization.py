#! /usr/bin/env python3

import numpy as np
import rospy
import struct

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point32
from sensor_msgs import point_cloud2

class Visualization(object):
    def __init__(self) -> None:
        self.pointcloud = PointCloud2()
        self.points = []
        self.channels = []
        self.z_diff = 7.5
        self.frame = "map"
    
    # set color map
    def Jet_colormap(self, gray_value):       # input: gray value 0~1 output: R,G,B
        cmap_value = int(gray_value * 255)
        if cmap_value < 32 :
            Red = 0
            Green = 0
            Blue = 128+4*cmap_value
        
        elif cmap_value == 32:
            Red = 0
            Green = 0
            Blue = 255
        
        elif  cmap_value > 32 and cmap_value < 96:
            Red = 0
            Green = 4+4*(cmap_value-33)
            Blue = 255
        
        elif cmap_value == 96:
            Red = 2
            Green = 255
            Blue = 254

        elif  cmap_value > 96 and cmap_value < 159:
            Red = 6+4*(cmap_value-97)
            Green = 255
            Blue = 250-4*(cmap_value-97)

        elif cmap_value == 159:
            Red = 254
            Green = 255
            Blue = 1

        elif  cmap_value > 159 and cmap_value < 224:
            Red = 255
            Green = 252-4*(cmap_value-160)
            Blue = 0

        else:
            Red = 252-4*(cmap_value-224)
            Green = 0
            Blue = 0
        
        return int(Red),int(Green),int(Blue)

    #calculate every cell's coordinate values
    def MatToCoor(self, robot_pose, info_map, resolution):
        self.points.clear()
        self.channels.clear()
        p = Point32()
        num = info_map.shape[0]
        for row in info_map:
            for cell in row:
                x = robot_pose[0] + (np.argwhere(info_map == cell)[0][1]-num/2)*resolution
                y = robot_pose[1] + (np.argwhere(info_map == cell)[0][0]-num/2)*resolution
                z = self.z_diff
                # print('cell value ',cell)
                r,g,b = self.Jet_colormap(cell)
                a = 255
                # print (r, g, b, a)
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                # print (hex(rgb))
                pt = [x, y, z, rgb]
                self.points.append(pt)
        
    # return the pointcloud2 object
    def CloudObject(self, robot_pose, info_map, resolution):
        self.MatToCoor(robot_pose, info_map, resolution)
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                # PointField('rgb', 12, PointField.UINT32, 1),
                PointField('rgba', 12, PointField.UINT32, 1),
                ]
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame
        pc2 = point_cloud2.create_cloud(header, fields, self.points)
        return pc2
        
