#!/usr/bin/env python

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

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    ''' pcl_callback function recieves input in the form of ros msg type which is ROS PointCloud2 message and converts
        it to a pcl PointXYZRGB (pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud). WE do this in order to apply functions 
        of pcl library to POint cloud data for building a perception pipeline. ''' 
    pcl_data = ros_to_pcl(pcl_msg)


    # TODO: Voxel Grid Downsampling
    ''' A 2D picture/image is a collection of lot of pixels similartly the smallest element of point cloud data type is a voxel(Volume ELement).
        We first build make_voxel_grid_filter object, next determine the voxel/leaf size, then use the set_leaf_size method to set the voxel/leaf size,
        Finally, apply the method filter().  '''     
    vox = pcl_data.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    ''' Pass through Filter is similarly to cropping an image. FIrst, we created a make_passthrough_filter object, defined axes along which
        to perform pass through, then set the min and max axis limits, then used the method set_filter_field_name and set_filter_limits
        to set the filter axis and axis limits, and finally use the method filter() to obtain the filtered data.'''     

    pass_through = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    pass_through.set_filter_field_name(filter_axis)
    axis_min,axis_max = 0.6,1.1
    pass_through.set_filter_limits(axis_min,axis_max)
    cloud_filtered = pass_through.filter()

    # TODO: RANSAC Plane Segmentation
    ''' RANSAC or Random Sample Consensus is  an algorithm that is used to identify points in the POInt cloud that belongs to a certain shape.
        Here we are using it to remove the table from the point cloud data. RANSAC algorithm divides the data points in to outliers and inliers
        based on the model we set. As table is similar to a plane we have set the model type to SACMODEL_PLANE. We then defined the distance threshold
        , which is actually the distance that algorithm takes into account while identifying the data. The RANSAC algo. gives us the indices only
        we then have to apply the extract method to get the point cloud data.'''         
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_dist = 0.0375
    seg.set_distance_threshold(max_dist)
    inliers, coefficients = seg.segment()

    # TODO: Extract inliers and outliers
    cloud_objects = cloud_filtered.extract(inliers,negative =True)
    cloud_table = cloud_filtered.extract(inliers,negative =False)

    # TODO: Euclidean Clustering
    ''' Here we DBSCAN algo. to find out the cluster in our object workspace. First, we convert the cloud_objects into XYZ data format.
        Then we set the kd_tree, and define various paramters like cluster tolerance(), min and max cluster size. Finallly we use the extract
        method to get the cluster indices( which probably a list of list of the points of the predicted clusters) of the different objects. 
        We can visualize the result on RViz, using the code described next.'''     

    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(1000)

    ec.set_SearchMethod(tree)

    cluster_indices = ec.Extract() #probabaly a list of list

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    ''' A 2D picture/image is a collection of lot of pixels similartly the smallest element of point cloud data type is a voxel(Volume ELement).
        We first build make_voxel_grid_filter object, next determine the voxel/leaf size, then use the set_leaf_size method to set the voxel/leaf size,
        Finally, apply the method filter().  '''     
    
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],white_cloud[indice][1],white_cloud[indice][2], rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

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

    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        pcl_cluster_ros = pcl_to_ros(pcl_cluster)

        chists = compute_color_histograms(pcl_cluster_ros, using_hsv=True)
        normals = get_normals(pcl_cluster_ros)
        nhists = compute_normal_histograms(normals)

        # Compute the associated feature vector
        feature_vector = np.concatenate((chists,nhists)) 

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature_vector.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        objects_marker_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = pcl_cluster_ros
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}').format(len(detected_objects))
    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering',anonymous = True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber('sensor_stick/point_cloud',pc2.PointCloud2,pcl_callback,queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size = 1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size = 1)
    pcl_cluster_pub = rospy.Publisher('/pcl_cluster', PointCloud2, queue_size = 1)
    objects_marker_pub = rospy.Publisher('/object_markers', Marker, queue_size = 1)
    detected_objects_pub = rospy.Publisher('/detected_objects',DetectedObjectsArray,queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
