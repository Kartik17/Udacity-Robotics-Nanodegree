#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    pcl_cloud = ros_to_pcl(pcl_msg)
    
    # TODO: Voxel Grid Downsampling
    # Create a make_voxel_grid_filter object
    voxel_filter = pcl_cloud.make_voxel_grid_filter()
    # Set Voxel size
    LEAF_SIZE = 0.006
    voxel_filter.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
    # Apply the voxel filter 
    cloud_filtered = voxel_filter.filter()
    # Convert the point cloud to ros type
    voxel_ros = pcl_to_ros(cloud_filtered)
    # PUblish the voxel_ros to pcl_voxel_pub publisher
    pcl_voxel_pub.publish(voxel_ros)

    # TODO: PassThrough Filter
    # Create a make_passthrough_filter object
    pass_through = cloud_filtered.make_passthrough_filter()
    # Set the filter axis to z
    filter_axis = 'z'
    pass_through.set_filter_field_name(filter_axis)
    # Set the axis minimum and axis maximum 
    axis_min,axis_max = 0.6,0.9
    # Set filter limits in the object of make_passthrough_filter()
    pass_through.set_filter_limits(axis_min,axis_max)
    # Apply the pass through filter
    cloud_filtered = pass_through.filter()


    pass_through = cloud_filtered.make_passthrough_filter()
    # Set the filter axis to y
    filter_axis = 'y'
    pass_through.set_filter_field_name(filter_axis)
    # Set the axis minimum and axis maximum 
    axis_min,axis_max = -0.42,0.42
    # Set filter limits in the object of make_passthrough_filter()
    pass_through.set_filter_limits(axis_min,axis_max)
    # Apply the pass through filter
    cloud_filtered = pass_through.filter()

    # Convert the pcl pontcloud to ros datatype
    passthrough_ros = pcl_to_ros(cloud_filtered)
    # Publish the ros type to pcl_passthrough_pub publisher
    pcl_passthrough_pub.publish(passthrough_ros)

    # TODO: Statistical Outlier Filtering
    # Create a make_stastical_outlier_filter object
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    # Set the number of points to be considered to 50
    outlier_filter.set_mean_k(50)
    # Set the threshold 
    x = 0.1
    outlier_filter.set_std_dev_mul_thresh(x)
    # Apply the outlier filter 
    cloud_filtered = outlier_filter.filter()
    # Convert the cloud type to ros
    outlier_ros = pcl_to_ros(cloud_filtered)
    # Publish the ros type to pcl_outliers_pub publisher
    pcl_outliers_pub.publish(outlier_ros)
    

    # TODO: RANSAC Plane Segmentation
    # Create a make_segmenter object
    seg = cloud_filtered.make_segmenter()
    # Set the model type to Plane
    seg.set_model_type(pcl.SACMODEL_PLANE)
    # Set the method type to Ransac
    seg.set_method_type(pcl.SAC_RANSAC)
    # Set the maximum distance threshold
    max_dist = 0.0075
    seg.set_distance_threshold(max_dist)
    # Apply the segment() method to get the inliers
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    cloud_objects = cloud_filtered.extract(inliers,negative =True)
    cloud_table = cloud_filtered.extract(inliers,negative =False)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    # Apply the make_kdtree() 
    tree = white_cloud.make_kdtree()

    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set the cluster Tolerance 
    ec.set_ClusterTolerance(0.01)
    # Set the Minimum and Maximum cluster size 
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(60000)

    ec.set_SearchMethod(tree)

    # Extract the cluster indices
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Get random color for each cluster and assign the cluster an individual color so as to visualize in RViz
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],white_cloud[indice][1],white_cloud[indice][2], rgb_to_float(cluster_color[j])])

    # Create a Point Cloud of the masked clusters, assign it the values using the generated list
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)


    # Convert the PointCloud to ros type
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)

    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)


# Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects = []
    detected_objects_labels = []
    for index,indices in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(indices)
        pcl_cluster_ros = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        # Extract the Color and Normal histogram data to use as our feature vector
        chists = compute_color_histograms(pcl_cluster_ros, using_hsv = True)
        normals = get_normals(pcl_cluster_ros)
        nhists = compute_normal_histograms(normals)
        # Concatenate the color and normal histogram
        feature = np.concatenate((chists,nhists))

        # Make the prediction
        # Make prediction using our classifier, by inputing the feature vector created above
        predict = clf.predict(scaler.transform((feature).reshape(1,-1)))
        label = encoder.inverse_transform(predict)[0]
        # Add the label to the list
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[indices[0]])
        label_pos[2] += .4
        objects_marker_pub.publish(make_label(label,label_pos, index))        

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = pcl_cluster_ros
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects), detected_objects_labels))
    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    
    try:
        pr2_mover(detected_objects,cloud_table)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list,cloud_table):

    # TODO: Initialize variables
    label_centroid = {}
    label_cloud = {}
    test_scene_num = Int32()
    dict_list = []

    # TODO: Get/Read parameters
    # Get the data of the pick list from '/object_list'
    object_list_param = rospy.get_param('/object_list')
    # Get the data of the dropbox  from '/dropbox'
    dropbox_list_param = rospy.get_param('/dropbox')
    # Set the test world number : 1,2, or 3 accordingly
    test_scene_num.data = 3

    # TODO: Parse parameters into individual variables
    #object_name.data = object_list_param[:]['name']
    #object_group = object_list_param[:]['group']

    # Loop through the object_list and create two dictionaries.
    # One for mapping label to centroid , other for mapping label to array of the point cloud data
    for object in object_list:
        point_arr = ros_to_pcl(object.cloud).to_array()
        label_cloud[object.label] = point_arr
        label_centroid[object.label] = np.mean(point_arr,axis = 0)[0:3]

    # Loop through the pick list and send request to the pick_place service
    for i in xrange(len(object_list_param)):
        object_name = String()
        # Assign the object name
        object_name.data = object_list_param[i]['name']
        # Assign the object group
        object_group = object_list_param[i]['group']
        # Initialize a POint Cloud XYZRGB datatype
        object_avoid = pcl.PointCloud_PointXYZRGB()
        # Create a Pose object 
        pick_pose = Pose()
        # Convert the pointcloud data of table to array
        avoid_object_list = cloud_table.to_array()

        # Send point cloud data to avoid collision to topic "/pr2/3D_map/points"
        '''for list_dict in object_list_param[i+1:]:
            avoid_object_list = np.concatenate((label_cloud[list_dict['name']],avoid_object_list))
        object_avoid.from_list(avoid_object_list)    
        ros_avoid_obj = pcl_to_ros(object_avoid)
        collision_avoidance_pub.publish(ros_avoid_obj)'''
        # TODO: Get the PointCloud for a given object and obtain it's centroid
        # Try to get the centroid of the object in the pick list using the dictionary I made earlier
        # Assign the pic_pose object the x,y,z coordinates, otherwise Key Error
        try:
            centroid = label_centroid[object_name.data]
        except KeyError:
            continue
        pick_pose.position.x = np.asscalar(centroid[0])
        pick_pose.position.y = np.asscalar(centroid[1])
        pick_pose.position.z = np.asscalar(centroid[2])
        

        # TODO: Create 'place_pose' for the object
        place_pose = Pose()

        # TODO: Assign the arm to be used for pick_place
        arm = String()
        # Check for the object group color and assign the arm data accordingly
        if object_group == 'green':
            arm.data = 'right'
            place_pose.position.x = dropbox_list_param[1]['position'][0]
            place_pose.position.y = dropbox_list_param[1]['position'][1]
            place_pose.position.z = dropbox_list_param[1]['position'][2]
        elif object_group == 'red':
            arm.data = 'left'
            place_pose.position.x = dropbox_list_param[0]['position'][0]
            place_pose.position.y = dropbox_list_param[0]['position'][1]
            place_pose.position.z = dropbox_list_param[0]['position'][2]
        else:
            print('NO GROUP ASSIGNED')

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num,arm,object_name,pick_pose,place_pose)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')
        '''
        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num,object_name,arm,pick_pose,place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    '''
    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_3.yaml', dict_list)
    



if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous = True)

    # Create Subscriber to bring in the camera data from the topic '/pr2/world/points'
    pcl_sub = rospy.Subscriber('/pr2/world/points',pc2.PointCloud2,pcl_callback,queue_size =1)

    # Create Publishers to publish data to respective topics
    ros_pub = rospy.Publisher('/pcl_view', PointCloud2,queue_size = 1)
    pcl_outliers_pub = rospy.Publisher('/pcl_outliers', PointCloud2, queue_size = 1)
    pcl_voxel_pub = rospy.Publisher('/pcl_voxel', PointCloud2, queue_size = 1)
    pcl_passthrough_pub = rospy.Publisher('/pcl_passthrough', PointCloud2, queue_size = 1)
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size = 1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size = 1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size = 1)
    objects_marker_pub = rospy.Publisher('/object_markers', Marker, queue_size = 1)
    detected_objects_pub = rospy.Publisher('/detected_objects',DetectedObjectsArray,queue_size=1)

    collision_avoidance_pub = rospy.Publisher('/avoid_objects',PointCloud2,queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model.sav','rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
