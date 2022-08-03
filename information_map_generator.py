import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp2d
from mpl_toolkits.mplot3d import Axes3D
from cmath import e, log,sqrt
import util

class infomation_map_generator(object):
    def __init__(self, raw_map , loc_dilate=15, info_dilate=15):
        self.width = raw_map.shape[1]
        self.height = raw_map.shape[0]
        self.loc_dilate = loc_dilate
        self.info_dilate = info_dilate
        
    def inflate_map(self,raw_map,inflation_radius):
        self.inflated_map = raw_map.copy()
        for row in range(self.width):
            for col in range(self.height):
                # self.inflated_map[row,col] = 200
                if(raw_map[row,col]> 70):
                    for m in range(row-inflation_radius,row+inflation_radius+1):
                        for n in range(col-inflation_radius,col+inflation_radius+1):
                            if (m>=0 and m<self.width) and (n>=0 and n<self.height):
                                self.inflated_map[m,n] = 100
        return self.inflated_map
    
    def collision_check(self,map_mat,start_point,end_point,check_step=1):
        x_1,y_1 = start_point
        x_2,y_2 = end_point
        x,y = x_1,y_1
        distance = sqrt((x_2-x_1)**2+(y_2-y_1)**2).real
        if map_mat[x_1,y_1]==100 or map_mat[x_2,y_2]==100:
            return False
        if distance > 0:
            sin_theta = (x_2-x_1)/distance
            cos_theta = (y_2-y_1)/distance
            while sqrt((x-x_1)**2+(y-y_1)**2).real < distance:
                if map_mat[int(round(x)),int(round(y))]==100:
                    return False
                x += check_step*sin_theta
                y += check_step*cos_theta
            return True
        else:
            return True

    def compute_frontier_occu(self, inflated_map):
        frontier_time1 = time.clock()

        frontier_list = []
        occupied_list = []
        for row in range(self.width):
            for col in range(self.height):
                # for construct unknown_map
                if inflated_map[row,col]==0:
                    for m in range(row-1,row+2):
                        for n in range(col-1,col+2):
                            if m <0 or m>=self.width or n<0 or n>=self.height:
                                continue

                            if inflated_map[m,n] < 0:
                                if [row,col] not in frontier_list:
                                    frontier_list.append([row,col])
                # for construct localization map                    
                elif inflated_map[row,col]==100:
                    for m in range(row-1,row+2):
                        for n in range(col-1,col+2):
                            if m <0 or m>=self.width or n<0 or n>=self.height:
                                continue
                            if inflated_map[m,n] != 100:
                                if [row,col] not in occupied_list:
                                    occupied_list.append([row,col])
        frontier_time2 = time.clock()
        # print('compute frontier & occu time: ',frontier_time2-frontier_time1)
        return np.asarray(frontier_list), np.asarray(occupied_list)

    def circle_func(self,x,y,x0,y0,radius):
        return (x-x0)**2 + (y-y0)**2 - radius**2

    def extract_point(self, pointO_r,pointO_c, radius):
        circle_point = []
        for r in range(pointO_r-radius, pointO_r+radius+1):
            for c in range(pointO_c-radius, pointO_c+radius+1):
                # print(r,c)
                if (r>=0 and r<self.width) and (c>=0 and c<self.height):
                    if self.circle_func(r,c,pointO_r,pointO_c,radius)<=0 and self.raw_map[r,c]< 60:
                        circle_point.append([r,c])
        return circle_point

    def get_dual_map(self, raw_map):
        self.inflated_map = self.inflate_map(raw_map,2)
        self.frontier_list,self.occupied_list, = self.compute_frontier_occu(self.inflated_map)
        unknown_map = np.zeros([self.width,self.height])
        localization_map = np.zeros([self.width,self.height])
        dual_map_time1 = time.clock()
        for row in range(self.width):
            for col in range(self.height):
                count = 0
                if len(self.frontier_list) > 3:
                    frontier = self.frontier_list[np.linalg.norm(self.frontier_list-np.array([[row,col]]),axis=1)<self.info_dilate]
                    unknown_map[row,col] = 0 #frontier.shape[0]
                    for points in frontier:
                        if not self.collision_check(self.inflated_map,points,[row,col]):
                            count += 1
                        else:
                            unknown_map[row,col] += 1*e**(-0.1*(sqrt((points[0]-row)**2+(points[1]-col)**2).real))
                    # unknown_map[row,col] -= count

                if len(self.occupied_list) > 0:
                    # localization_map[row,col] = 0
                    localization_map[row,col] = self.occupied_list[np.linalg.norm(self.occupied_list-np.array([[row,col]]),axis=1)<self.loc_dilate].shape[0]
                    # occupied = self.occupied_list[np.linalg.norm(self.occupied_list-np.array([[row,col]]),axis=1)<self.loc_dilate]
                    # print(occupied)
                    # for points in occupied:
                        # if self.inflated_map[row,col] != -101 and self.inflated_map[row,col] != 100:
                        #     localization_map[row,col] += 1*e**(-0.05*(sqrt((points[0]-row)**2+(points[1]-col)**2).real))
                        # else:
                        #     unknown_map[row,col] += 1*e**(-0.1*(sqrt((points[0]-row)**2+(points[1]-col)**2).real))
                    localization_map[self.inflated_map==100] = 0
        dual_map_time2 = time.clock()
        # print('dual map time: ',dual_map_time2-dual_map_time1)

        if np.max(unknown_map)!=0:
            unknown_map = unknown_map/np.max(unknown_map)*1

        if np.max(localization_map)!=0:
            localization_map = localization_map/np.max(localization_map)*1
        return unknown_map , localization_map

    def get_information_map(self,raw_map,w1,w2):
        self.unknown_map, self.localization_map = self.get_dual_map(raw_map)
        self.information_map = w1*self.unknown_map + w2*self.localization_map
        return self.information_map

# map_color= {'uncertain':-50, 'free':0, 'obstacle':100}
# def extract_map(file_name):
#     map_test = np.load(file_name)
#     map_proc = np.zeros_like(map_test)
#     map_proc[map_test == 100] = map_color['obstacle']
#     map_proc[map_test == 0] = map_color['free']
#     map_proc[map_test == -101] = map_color['uncertain']
#     return map_proc

# def get_local_map(total_map,position,window_length=50):
#     local_map = np.zeros([window_length,window_length])
#     x,y = int(round(position[0])),int(round(position[1]))
#     # for m in range(x-window_length/2,x+(window_length/2)+1):
#     #     for n in range(y-window_length/2,y+(window_length/2)+1):
#     local_map = total_map[y-window_length/2:y+(window_length/2),x-window_length/2:x+(window_length/2)]
#     return local_map

# if __name__ == '__main__':
#     map1 = extract_map('test_map.npy')
#     # start_time1 = time.clock()
#     local_map = get_local_map(map1,[231,434],100)
#     map_width,map_height = local_map.shape[0],local_map.shape[1]
#     info_gerator = infomation_map_generator(local_map,)
#     plt.matshow(map1,cmap = 'gray')
#     # plt.show()

#     fig1 = plt.figure()
#     ax1 = plt.subplot(2,3,1)
#     ax1.set_title('Raw_map')
#     ax1.matshow(local_map,cmap = 'gray')
#     #! output information map
#     info_gerator.get_information_map(local_map,0.5,0.5)
#     ax2 = plt.subplot(2,3,2)
#     ax2.set_title('Unkown_information')
#     ax2.matshow(info_gerator.unknown_map,cmap='jet')

#     #! output localization map
#     ax3 = plt.subplot(2,3,3)
#     ax3.set_title('Localization_information')
#     ax3.matshow(info_gerator.localization_map,cmap='bwr')
#     # plt.show()

#     #! output Information map
#     ax4 = plt.subplot(2,3,4)
#     ax4.set_title('Information_Map')
#     ax4.matshow(info_gerator.information_map,cmap='jet')
#     # plt.show()
#     # end_time1 = time.clock()
#     # print(str(end_time1-start_time1))
#     #! output interpolated map
#     # start_time2 = time.clock()
#     x = np.arange(0,map_width,1)  # row
#     y = np.arange(0,map_height,1)   # col
#     Y,X = np.meshgrid(x,y)
#     # points = np.hstack((X.flatten()[:,None],Y.flatten()[:,None]))
#     z = info_gerator.information_map.flatten()
    
#     new_x = np.arange(0,map_width,0.5)
#     new_y = np.arange(0,map_height,0.5)
#     Y1,X1 = np.meshgrid(new_x,new_y)
    
#     z1 = interp2d(x,y,z,kind='cubic')
#     new_z = z1(new_x,new_y)
#     # end_time2 = time.clock()
#     # print(str(end_time2-start_time2))

#     plt.matshow(new_z,cmap='jet')
#     plt.show()