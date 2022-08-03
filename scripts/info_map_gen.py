#!/usr/bin/env python3

from socket import  *
import threading
from queue import Queue
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from raw_map_proc import map_processor
import raw_map_proc
# from visualization import Visualization
import visualization

def mapCB(msg):
    # map_queque.put(msg)
    # raw_map = map_queque.get()
    resolution = msg.info.resolution
    map_size = [msg.info.height,msg.info.width]
    map_origin = [msg.info.origin.position.x,msg.info.origin.position.y]
    map_list = msg.data
    window_size = [100,100]
    
    listener = tf.TransformListener()
    local_tf = tf.TransformBroadcaster(queue_size=100)
    listener.waitForTransform('/map','/base_link',rospy.Time(), rospy.Duration(0.2))
    try:
        (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))


        map_mat = processor.map_matrix_gen(map_list,map_size[1],map_size[0])
        #! Local map
        #* Obtain local map
        local_map = processor.get_local_map(map_mat, [trans[0],trans[1]], map_origin, resolution, map_size, [100,100])
        local_map_data = processor.get_info_maplist(local_map, [100,100]).astype(int).tolist()
        local_map_object = OccupancyGrid
        local_map_object = msg
        local_map_object.info.height = window_size[0]
        local_map_object.info.width = window_size[1]
        local_map_object.info.origin.position.x = trans[0]-window_size[0]/2*resolution
        local_map_object.info.origin.position.y = trans[1]-window_size[1]/2*resolution
        local_map_object.info.origin.position.z = 2.5
        local_map_object.data = local_map_data
        localmapPub.publish(local_map_object)


        #* Obtain infomation map
        #! Information map
        info_map = processor.get_info_map(local_map)
        poincloud = PointCloud2()
        poincloud = vis_infomap.CloudObject([trans[0],trans[1]], info_map, resolution)
        infoPub.publish(poincloud)
        info_map[info_map<0.1] = 0
        info_map = (101*info_map-1).astype(int)
        info_map_data = processor.get_info_maplist(info_map, [100,100]).astype(int).tolist()
        info_map_object = OccupancyGrid
        info_map_object = msg
        info_map_object.info.height = window_size[0]
        info_map_object.info.width = window_size[1]
        info_map_object.info.origin.position.x = trans[0]-window_size[0]/2*resolution
        info_map_object.info.origin.position.y = trans[1]-window_size[1]/2*resolution
        info_map_object.info.origin.position.z = 5.0
        info_map_object.data = info_map_data
        infomapPub.publish(info_map_object)

        #* Broadcaste tf: map->info_map
        tf_info_map = TransformStamped()
        tf_info_map.header.frame_id = "map"
        tf_info_map.header.stamp = msg.header.stamp
        tf_info_map.child_frame_id = "info_map"
        tf_info_map.transform.translation.x = trans[0]
        tf_info_map.transform.translation.y = trans[1]
        tf_info_map.transform.translation.z = 0.0
        qtn = tf.transformations.quaternion_from_euler(0,0,0)
        tf_info_map.transform.rotation.x = qtn[0]
        tf_info_map.transform.rotation.y = qtn[1]
        tf_info_map.transform.rotation.z = qtn[2]
        tf_info_map.transform.rotation.w = qtn[3]

        local_tf.sendTransform((trans[0],trans[1],0.0),
                                qtn,
                                rospy.Time.now(),
                                "info_map",
                                "map")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print('Transform Error!')

def spinThread():
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node("information_map_generator",anonymous=True)
    infomapPub = rospy.Publisher("infomation_map", OccupancyGrid, queue_size=10)
    localmapPub = rospy.Publisher("local_map", OccupancyGrid, queue_size=10)
    infoPub = rospy.Publisher("vis_infomap", PointCloud2, queue_size=10)
    # markerPub = rospy.Publisher("test_pose", Marker, queue_size=10)
    mapSub = rospy.Subscriber("map", OccupancyGrid, mapCB, queue_size=10)
    map_queque = Queue()
    processor = raw_map_proc.map_processor()
    vis_infomap = visualization.Visualization()
    print('Information Map computing!!')
    print('Server is Ready!')
    #! Create subthread for callback func
    # add_thread = threading.Thread(target = spinThread)
    # add_thread.start()
    # --------- Main Loop --------- #
    while not rospy.is_shutdown():
        rospy.spin()
        # raw_map = map_queque.get()
        # resolution = raw_map.info.resolution
        # map_size = [raw_map.info.height,raw_map.info.width]
        # map_origin = [raw_map.info.origin.position.x,raw_map.info.origin.position.y]
        # map_list = raw_map.data
        # window_size = [100,100]
        
        # listener = tf.TransformListener()
        # local_tf = tf.TransformBroadcaster(queue_size=100)
        # listener.waitForTransform('/map','/base_link',rospy.Time(), rospy.Duration(0.2))
        # try:
        #     (trans,rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))


        #     map_mat = processor.map_matrix_gen(map_list,map_size[1],map_size[0])
        #     #! Local map
        #     #* Obtain local map
        #     local_map = processor.get_local_map(map_mat, [trans[0],trans[1]], map_origin, resolution, map_size, [100,100])
        #     local_map_data = processor.get_info_maplist(local_map, [100,100]).astype(int).tolist()
        #     local_map_object = OccupancyGrid
        #     local_map_object = raw_map
        #     local_map_object.info.height = window_size[0]
        #     local_map_object.info.width = window_size[1]
        #     local_map_object.info.origin.position.x = trans[0]-window_size[0]/2*resolution
        #     local_map_object.info.origin.position.y = trans[1]-window_size[1]/2*resolution
        #     local_map_object.info.origin.position.z = 2.5
        #     local_map_object.data = local_map_data
        #     localmapPub.publish(local_map_object)


        #     #* Obtain infomation map
        #     #! Information map
        #     info_map = processor.get_info_map(local_map)
        #     poincloud = PointCloud2()
        #     poincloud = vis_infomap.CloudObject([trans[0],trans[1]], info_map, resolution)
        #     infoPub.publish(poincloud)
        #     # info_map[info_map<0.1] = 0
        #     # info_map = (101*info_map-1).astype(int)
        #     # info_map_data = processor.get_info_maplist(info_map, [100,100]).astype(int).tolist()
        #     # info_map_object = OccupancyGrid
        #     # info_map_object = raw_map
        #     # info_map_object.info.height = window_size[0]
        #     # info_map_object.info.width = window_size[1]
        #     # info_map_object.info.origin.position.x = trans[0]-window_size[0]/2*resolution
        #     # info_map_object.info.origin.position.y = trans[1]-window_size[1]/2*resolution
        #     # info_map_object.info.origin.position.z = 5.0
        #     # info_map_object.data = info_map_data
        #     # infomapPub.publish(info_map_object)

        #     #* Broadcaste tf: map->info_map
        #     tf_info_map = TransformStamped()
        #     tf_info_map.header.frame_id = "map"
        #     tf_info_map.header.stamp = raw_map.header.stamp
        #     tf_info_map.child_frame_id = "info_map"
        #     tf_info_map.transform.translation.x = trans[0]
        #     tf_info_map.transform.translation.y = trans[1]
        #     tf_info_map.transform.translation.z = 0.0
        #     qtn = tf.transformations.quaternion_from_euler(0,0,0)
        #     tf_info_map.transform.rotation.x = qtn[0]
        #     tf_info_map.transform.rotation.y = qtn[1]
        #     tf_info_map.transform.rotation.z = qtn[2]
        #     tf_info_map.transform.rotation.w = qtn[3]

        #     local_tf.sendTransform((trans[0],trans[1],0.0),
        #                             qtn,
        #                             rospy.Time.now(),
        #                             "info_map",
        #                             "map")
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     print('Transform Error!')
        