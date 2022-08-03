#! /usr/bin/env python3

import numpy as np
from math import fabs
# from nn_interface import map2inform
import nn_interface
import os, sys


class map_processor:
    def __init__(self):
        file_path = os.path.dirname(os.path.realpath(__file__))
        self.info_generator = nn_interface.map2inform(file_path+'/nn_model/best_model_params.pth')
        print('Network Ready!')

    # convert map list data into map matrix
    def map_matrix_gen(self, map_list, width, height):
        map_list = np.array(map_list).reshape([height,width])
        map_matrix = np.flip(map_list,axis=0)
        return map_matrix.copy()
    
    # convert map matrix into map list data
    def get_maplist(self, map_mat, map_size):
        
        map_list = np.flip(map_mat,axis=0)
        map_list = np.reshape(map_list,map_size[0]*map_size[1])
        return map_list

    # param: raw_pose: [x,y] robot pose in the map, relative to map_origin
    #        map_origin: [x,y] map origin point
    #        resolution: map resolution
    #        map_size: [height,width]    
    def get_pose_coor(self, raw_pose, map_origin, resolution, map_size):
        col = int((raw_pose[0]+fabs(map_origin[0]))/resolution)
        row = int((raw_pose[1]+fabs(map_origin[1]))/resolution)
        row = map_size[0] - row
        return row,col

    def get_local_map(self, map_mat, raw_pose ,map_origin, resolution, map_size, window_size):
        pose = self.get_pose_coor(raw_pose, map_origin, resolution, map_size)
        
        local_map = np.zeros(window_size)
        # Bound check
        if pose[0]-window_size[0]/2 < 0:
            start_row = 0
        else: start_row = pose[0]-int(window_size[0]/2)

        if pose[1]-window_size[1]/2 < 0:
            start_col = 0
        else: start_col = pose[1]-int(window_size[1]/2)
        
        local_map = map_mat[start_row:pose[0]+int(window_size[0]/2), start_col:pose[1]+int(window_size[1]/2)]
        # Padding
        if pose[0]-window_size[0]/2 < 0:
            padding_mat = -np.ones([-(pose[0]-window_size[0]/2),local_map.shape[1]])
            local_map = np.r_[padding_mat,local_map]

        if pose[0]+window_size[0]/2 > map_size[0]:
            padding_mat = -np.ones([(pose[0]+window_size[0]/2)-map_size[0],local_map.shape[1]])
            local_map = np.r_[local_map,padding_mat]
        
        if pose[1]-window_size[1]/2 < 0:
            padding_mat = -np.ones([local_map.shape[0],-(pose[1]-int(window_size[1]/2))])
            local_map = np.c_[padding_mat,local_map]

        if pose[1]+window_size[1]/2 > map_size[1]:
            padding_mat = -np.ones([local_map.shape[0],(pose[1]+int(window_size[1]/2))-map_size[1]])
            local_map = np.c_[local_map,padding_mat]
        
        return local_map.copy()

    # obtain the network output
    def get_info_map(self, local_map):
        info_map = self.info_generator.get_inform(local_map)
        info_map = np.flip(info_map,axis=0)
        print(info_map)
        return info_map
        
    def get_info_maplist(self, info_map, window_size):
        info_map_list = np.flip(info_map,axis=0)
        info_map_ = np.reshape(info_map_list,window_size[0]*window_size[1])
        return info_map_