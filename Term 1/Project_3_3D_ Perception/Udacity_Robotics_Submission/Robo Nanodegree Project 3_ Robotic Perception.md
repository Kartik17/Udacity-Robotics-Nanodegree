# UDACITY Robo Nanodegree Project 3: Robotic Perception

In this project of Robotics Nanodegree the problem that students were given to tackle was to develop a perception pipeline for the pr2 robot. The objective of the pipeline was to make the robot differentiate between various objects kept on a table using methods of image processing and machine learning. The overall problem was divided into three part/ exercises which were combined together as one to complete the pipeline.
  - Exercise 1: Pipeline for filtering and RANSAC plane fitting
  - Exercise 2: Cluster segmentation
  - Exercise 3: Object Recongnition


# Exercise 1

In this exercise we develop a pipeline for filtering and RANSAC plane fitting, so as to segregrate the objects and the table. The output cloud will be used as an input for cluster segmentation.

- Initializing the ROS Node, every node has a name, so let our code communicate with other nodes we have to provide this information. The parameter `anonymous = True` add a random nuber at the end of the node's name so as to prevent two nodes from having the same name.
```py
if __name__ == '__main__':
    # TODO: ROS node initialization
    rospy.init_node('clustering',anonymous = True)
```
- After we have initialized the node we have to suscribe to the topic which the camera node publishes to. The topic name is '/pr2/world/points', the data type is ROS PointCloud2, and on receiving the data the node call the `pcl_callback()` function so as to start the perception pipeline. Queue Size determines the size of the outgoing message queue used for asynchronous publishing. Here Queue Size is set to 1. 

Choosing Good Queue Size (For Reference), as per ROS.org:
> For sending just one message at a fixed rate it is fine to use a queue size as small as the frequency of the publishing.
>  For sending multiple messages in a burst you should make sure that the queue size is big enough to contain all those messages. Otherwise it is likely to lose messages

```py
# TODO: Create Subscribers
pcl_sub = rospy.Subscriber('/pr2/world/points',pc2.PointCloud2,pcl_callback,queue_size =1)
```

- The PointCloud2 data is sent to the pcl_callbcak function, where the first step is to convert it from ros format to pcl(Point Cloud Library) PointCloudXYZRGB format.
```py
def pcl_callback(pcl_msg):
 # TODO: Convert ROS msg to PCL data
    pcl_cloud = ros_to_pcl(pcl_msg)
```
- **Voxel Grid Filter:** A 2D picture/image is a collection of lot of pixels similartly the smallest element of point cloud data type is a voxel(Volume ELement). We first build make_voxel_grid_filter object, next determine the voxel/leaf size, then use the set_leaf_size method to set the `voxel/leaf size`, Finally, apply the `method filter()`. To determine I started off with a random number of 0.01 and lowered the value till I settled on for 0.006. If I lowered the value even more then my object recongnition wasn't able to identify book in last(third) test world. I also published`(/pcl_voxel)` the cloud obtained after voxel filtering for visualizing it in RViz.
    
```py    
    # TODO: Voxel Grid Downsampling
    voxel_filter = pcl_cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.006
    voxel_filter.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
    cloud_filtered = voxel_filter.filter()
    voxel_ros = pcl_to_ros(cloud_filtered)
    pcl_voxel_pub.publish(voxel_ros)
```    
![Voxel](https://www.dropbox.com/s/y3wipqs26t2tm8o/IMG_20180716_211126.jpg?raw=true)

- **PassThrough Filter**: Pass through Filter is similarly to cropping an image. First, we created a `make_passthrough_filter()` object, defined axes along which to perform pass through, then set the min and max axis limits, then used the method set_filter_field_name and set_filter_limits to set the filter axis and axis limits, and finally use the `method filter()` to obtain the filtered data. As per the test environment I used pass through filter twice , first along z axis, then along the y axis. The values of axis minimum and maximum for z axis is set for 0.6,0.9, and for y axis I set the values to -0.42 and 0.42 becasue of the table symmetry. The values were arrived at after several trials. Lastly I published `(/pcl_passthrough)` the obtained point cloud for visualization in RViz.
```py   
    # TODO: PassThrough Filter
    pass_through = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    pass_through.set_filter_field_name(filter_axis)
    axis_min,axis_max = 0.6,0.9
    pass_through.set_filter_limits(axis_min,axis_max)
    cloud_filtered = pass_through.filter()


    pass_through = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    pass_through.set_filter_field_name(filter_axis)
    axis_min,axis_max = -0.42,0.42
    pass_through.set_filter_limits(axis_min,axis_max)
    cloud_filtered = pass_through.filter()

    passthrough_ros = pcl_to_ros(cloud_filtered)
    pcl_passthrough_pub.publish(passthrough_ros)    
```    
![Passthrough](https://www.dropbox.com/s/aojrwq4bra3ojo3/IMG_20180716_211154.jpg?raw=true)    
- **Statistical Outlier Filtering**: Statistical Outlier Filter is used to filter noise from the 3D Point Cloud data. First, I created a `make_statistical_outlier_filter()` object, where I assigned the method `set_mean_k()` to 50, which means that the filter will take into account 50 neighbouing points. I assigned the method `set_std_dev_mul_thresh()` the value of 0.1, the filter will ignore the values that above mean the 0.1. Lastly I published `(/pcl_outliers)` the obtained point cloud for visualization in RViz. I intentionally did statistical outlier filtering after Voxel and Pass through filters as I noticed that voxel filter also generated some outliers points.    
 ``` py   
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 0.1
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filtered = outlier_filter.filter()
    outlier_ros = pcl_to_ros(cloud_filtered)
    pcl_outliers_pub.publish(outlier_ros)
```  
![Outliers](https://www.dropbox.com/s/8oq2k28n010bpgc/IMG_20180716_211221.jpg?raw=true)
- **RANSAC Plane Segmentation**: RANSAC or Random Sample Consensus is an algorithm that is used to identify points in the Point cloud that belongs to a certain shape. Here we are using it to remove the table from the point cloud data. First, I make a `pcl.Segmentation object` using the method of the `pcl.PointCloud` class. RANSAC algorithm divides the data points in to outliers and inliers based on the model `(set_method_type)` we set. As table is similar to a plane we have set the model type to `SACMODEL_PLANE`. We then defined the distance threshold by assigning the class method `seg.set_distance_threshold(threshold_value)` some value, which is actually the distance that algorithm takes into account while identifying the data. The RANSAC algorithm gives us the indices only we then have to apply the extract method to get the point cloud data. 

As per Ros.org,
> make_segmenter(self)
Return a pcl.Segmentation object with this object set as the input-cloud
    
```py   
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_dist = 0.0375
    seg.set_distance_threshold(max_dist)
    inliers, coefficients = seg.segment() 
```      
- **Extract Table and Objects**: The method `seg.segmemt()` gives us the indices of the segmented plane. The pcl.PointCloud method extract takes these indices and gives us the desired point cloud that are inliers and outliers. The inliers Point cloud is the table and the outliers point cloud represent the objects.
```py   
    cloud_objects = cloud_filtered.extract(inliers,negative =True)
    cloud_table = cloud_filtered.extract(inliers,negative =False)
```
- **Objects**
![Objects](https://www.dropbox.com/s/b9okegsv84cpodq/IMG_20180716_211303.jpg?raw=true)
- **Table**
![Table](https://www.dropbox.com/s/ix5yz7ofgsidlsn/IMG_20180716_211330.jpg?raw=true)
# Exercise 2

- **DBSCAN Clustering**: After getting the point cloud data of the objects , clustering of individual point cloud data of objects needs to be done. For segmentation useing clustering, DBSCAN(Density-Based Spatial Clustering of Applications with Noise) algorithm is used. The algorithm is sometimes also called “Euclidean Clustering”, because the decision of whether to place a point in a particular cluster is based upon the “Euclidean distance” between that point and other cluster members. To implement the algorithm in the pipeline, I converted the XYZRGB data to XYZ as the algorithm to make use of the method `make_kdtree()`(PCL's Euclidean Clustering algorithm requires a point cloud with only spatial information). Next I set the cluster tolerance, max and min. cluster size. The value of cluster tolerance was found to work best for 0.01. The max. and min. size of cluster is 100 and 60000 respectively.
> The k-d tree data structure is used in the Euclidian Clustering algorithm to decrease the computational burden of searching for neighboring points. While other efficient algorithms/data structures for nearest neighbor search exist, PCL's Euclidian Clustering algorithm only supports k-d trees. 
   
```py
white_cloud = XYZRGB_to_XYZ(cloud_objects)
tree = white_cloud.make_kdtree()
ec = white_cloud.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance(0.01)
ec.set_MinClusterSize(100)
ec.set_MaxClusterSize(60000)

ec.set_SearchMethod(tree)
cluster_indices = ec.Extract()
```
- **Cluster-Mask Point Cloud**: To visualize the clusters generated color mask was generated. Data of all indices was stored in `cluster_indices` which is a list of list of indices of all cluster segmented. The function `get_color_list()` was used to generate random color for all the clusters. Using a nested for loop, an empty list was filled with list of cluster indices with points and an unique rgb values. After which a Point cloud XYZRGB datatype was filled using the `from_list()` method.
```py
cluster_color = get_color_list(len(cluster_indices))
color_cluster_point_list = []

for j, indices in enumerate(cluster_indices):
    for i, indice in enumerate(indices):
        color_cluster_point_list.append([white_cloud[indice][0],white_cloud[indice][1],white_cloud[indice][2], rgb_to_float(cluster_color[j])])

cluster_cloud = pcl.PointCloud_PointXYZRGB()
cluster_cloud.from_list(color_cluster_point_list)
ros_cluster_cloud = pcl_to_ros(cluster_cloud)
```
![Cluster](https://www.dropbox.com/s/wclx5k244lpt6hi/IMG_20180716_211101.jpg?raw=true)

# Exercise 3

- **Object Recongnition**: After determining the object clusters I had to identify the object with its true label. To perform the task of Object Recongnition a machine learning algorithm `Support Vector Machine ` is used. `SVM` is a supervised learning which means that one has to train it before using it to perform predictive tasks. The feature vector for the current ML model are the `Color Histogram` and `Normal Histogram`. The array obtained from the functions are concatenated and saved in a file `training.sav`. After training the algorithm with a certain set of model, we use the trained iinformation to make prediction on the test worlds.

```py
# Classify the clusters! (loop through each detected cluster one at a time)
detected_objects = []
detected_objects_labels = []
for index,indices in enumerate(cluster_indices):
    # Grab the points for the cluster
    pcl_cluster = cloud_objects.extract(indices)
    pcl_cluster_ros = pcl_to_ros(pcl_cluster)
    # Compute the associated feature vector
    chists = compute_color_histograms(pcl_cluster_ros, using_hsv = True)
    normals = get_normals(pcl_cluster_ros)
    nhists = compute_normal_histograms(normals)
    feature = np.concatenate((chists,nhists))

    # Make the prediction
    predict = clf.predict(scaler.transform((feature).reshape(1,-1)))
    label = encoder.inverse_transform(predict)[0]
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
l
```
- **Test World 1**: The paramaters to tune to increase the accuracy of the preiction were: 
    - `bin_size`: The information extracted from the `color/normal histrogram` function is different channels of the histogram. The user/ programmer has the option to control the numer of bins so as to increase the information stored during training. For `Test World 1`, I used `bins = 64`. 
    - `Sample orientation Considered`: 80
    - `Using HSV`: Colorspace `HSV` is more robust to lighting than `RGB`, therefore it has been used to increase the prediction accuracy.
    - `Kernel`: Kernel are similarity functions to find similarity between the data points. For the test world 2 I have used the `Linear` kernel. The kernel function can be any of the following:
        - Linear
        - polynomial
        - rbf
        - Sigmoid
    - `Regularization`: Regularization or C, is the penalty factor. If set too large, we have a high penalty for nonseparable points and we may overfit. If it is too small, we have underfitting. For test world 1 I have used `C = 0.01`
![With Normalization](https://www.dropbox.com/s/32we7rsp7y91ay3/test_1_normal.png?raw=true)

![W/O Normalization](https://www.dropbox.com/s/ebnke49zt94y5nv/test_1_wo_normal.png?raw=true)

- **Test World 2**: TThe paramaters to tune to increase the accuracy of the preiction were: 
    - `bin_size`: The information extracted from the `color/normal histrogram` function is different channels of the histogram. The user/ programmer has the option to control the numer of bins so as to increase the information stored during training. For `Test World 2`, I used `bins = 64`. 
    - `Sample orientation Considered`: 80
    - `Using HSV`: Colorspace `HSV` is more robust to lighting than `RGB`, therefore it has been used to increase the prediction accuracy.
    - `Kernel`: Kernel are similarity functions to find similarity between the data points. For the test world 2 I have used the `Linear` kernel. The kernel function can be any of the following:
        - Linear
        - polynomial
        - rbf
        - Sigmoid
    - `Regularization`: Regularization or C, is the penalty factor. If set too large, we have a high penalty for nonseparable points and we may overfit. If it is too small, we have underfitting. For test world 2 I have used `C = 0.01`

![With Normalization](https://www.dropbox.com/s/d9ddtey4xy0g20v/test_2_normal.png?raw=true)

![W/O Normalization](https://www.dropbox.com/s/94va9lckshs0bia/test_2_wo_normal.png?raw=true)
- **Test World 3**: The paramaters to tune to increase the accuracy of the preiction were: 
    - `bin_size`: The information extracted from the `color/normal histrogram` function is different channels of the histogram. The user/ programmer has the option to control the numer of bins so as to increase the information stored during training. For `Test World 3`, I used `bins = 64`. 
    - `Sample orientation Considered`: 80
    - `Using HSV`: Colorspace `HSV` is more robust to lighting than `RGB`, therefore it has been used to increase the prediction accuracy.
    - `Kernel`: Kernel are similarity functions to find similarity between the data points. For the test world 3 I have used the `Linear` kernel. The kernel function can be any of the following:
        - Linear
        - polynomial
        - rbf
        - Sigmoid  
    - `Regularization`: Regularization or C, is the penalty factor. If set too large, we have a high penalty for nonseparable points and we may overfit. If it is too small, we have underfitting. For test world 3 I have used `C = 0.01`.

![With Normalization](https://www.dropbox.com/s/jjmxuqwlijcexmm/test_3_normal.png?raw=true)

![W/O Normalization](https://www.dropbox.com/s/qgpg856t17u2uhk/test_3_wo_normal.png?raw=true)
- **Computing Color and Normal Histogram**: To compute the normal and color histogram I had to complete the `features.py` script so that the function `compute_color_histograms(cloud, using_hsv = True)` and `compute_normal_histograms(normal_cloud)` return the respective results. 

```py
def compute_color_histograms(cloud, using_hsv = True):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud (cloud data)
    for point in pc2.read_points(cloud, skip_nans=True):
        # 3 index contains the color value
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])
    
    # TODO: Compute histograms
    channel1_hist = np.histogram(channel_1_vals, bins = 64, range = (0,256))
    channel2_hist = np.histogram(channel_2_vals, bins = 64, range = (0,256))
    channel3_hist = np.histogram(channel_3_vals, bins = 64, range = (0,256))
    # TODO: Concatenate and normalize the histograms
    hist_features = np.concatenate((channel1_hist[0],channel2_hist[0],channel3_hist[0])).astype(np.float64)
    normed_features = hist_features/np.sum(hist_features)

    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    # normed_features = np.random.random(96) 
    return normed_features 
    
def compute_normal_histograms(normal_cloud):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud, field_names = ('normal_x', 'normal_y', 'normal_z'),skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # TODO: Compute histograms of normal values (just like with color)
    norm_x_hist = np.histogram(norm_x_vals, bins = 64, range = (-1,1))
    norm_y_hist = np.histogram(norm_y_vals, bins = 64, range = (-1,1))
    norm_z_hist = np.histogram(norm_z_vals, bins = 64, range = (-1,1))

    # TODO: Concatenate and normalize the histograms
    hist_features = np.concatenate((norm_x_hist[0],norm_y_hist[0],norm_z_hist[0])).astype(np.float64)
    normed_features = hist_features/np.sum(hist_features)

    # Generate random features for demo mode.  
    # Replace normed_features with your feature vector
    # normed_features = np.random.random(96)

    return normed_features
```

# PR2_mover()
- On identifying the object label, the next step is to output the results to `.yaml` file. The file should contains information on:
    - Object Name (Obtained from the pick list)
    - Arm Name (Based on the group of an object)
    - Pick Pose (Centroid of the recognized object)
    - Place Pose (Not a requirement for passing but needed to make the PR2 happy)
    - Test Set Number
- I first initialized the variables required for the task, like initializing the Int32() object for test set number. Then using a for loop I assign the centroid to the respective label and store it in the dictionary `label_centroid`. After this, I looped through the pick up list which is obtained through the rospy method `get_param('/object_list')`. In the loop I initialize a `String()` ros type and assign it the data of the object name which is to be picked. Using `Pose()` class I make two objects `pick_pose` and `place_pose`. For `pick_pose` I assign it the value of the centroid through the dictionary I made earlier. For the `place_pose` I assign the values based on the color information, if the object group is green then `arm.data = right` else when the object group is red then `arm.data = left`. To make a yaml dictionary I pass through the data to the `make_yaml_dict()` function. Then pass the variables `test_scene_num, object_name, arm, pick_pose, place_pose` as a service request to `pick_place_routine()`.

```py
def pr2_mover(object_list,cloud_table):

    # TODO: Initialize variables
    label_centroid = {}
    label_cloud = {}
    test_scene_num = Int32()
    dict_list = []

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')
    test_scene_num.data = 2

    # TODO: Parse parameters into individual variables
    #object_name.data = object_list_param[:]['name']
    #object_group = object_list_param[:]['group']

    for object in object_list:
        point_arr = ros_to_pcl(object.cloud).to_array()
        label_cloud[object.label] = point_arr
        label_centroid[object.label] = np.mean(point_arr,axis = 0)[0:3]

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    for i in xrange(len(object_list_param)):
        object_name = String()
        object_name.data = object_list_param[i]['name']
        object_group = object_list_param[i]['group']
        object_avoid = pcl.PointCloud_PointXYZRGB()
        pick_pose = Pose()
        avoid_object_list = cloud_table.to_array()

        # Send point cloud data to avoid collision to topic "/pr2/3D_map/points"
        for list_dict in object_list_param[i+1:]:
            avoid_object_list = np.concatenate((label_cloud[list_dict['name']],avoid_object_list))
        object_avoid.from_list(avoid_object_list)    
        ros_avoid_obj = pcl_to_ros(object_avoid)
        collision_avoidance_pub.publish(ros_avoid_obj)
        # TODO: Get the PointCloud for a given object and obtain it's centroid
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

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num,arm,object_name,pick_pose,place_pose)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')
        
        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num,object_name,arm,pick_pose,place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_2.yaml', dict_list)
```


# Main Function
After I have subscribed to the topic `/pr2/world/points`, I initialized all the publishers as there in the code below. Some of the publishers are there only for visualization purpose. Next, I read the data from the `model.sav` file, assign `LabelEncoder()`. And till ` rospy.is_shutdown()` is not true keep the node running.

```py
if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous = True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points',pc2.PointCloud2,pcl_callback,queue_size =1)

    # TODO: Create Publishers
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

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav','rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
```
# Results
- **Test World 1**
![Test1](https://www.dropbox.com/s/bsn08b152uzinr2/IMG_20180715_105140.jpg?raw=true)

- **Test World 2**
![Test2](https://www.dropbox.com/s/yuhnc9yvdaa6wup/Test_2.jpg?raw=true)

- **Test World 3**

![Test3](https://www.dropbox.com/s/4765mmf8zyg00xm/IMG_20180716_212132%281%29.jpg?raw=true)
![Test3](https://www.dropbox.com/s/t5yut7d11njmj3i/IMG_20180716_212128%281%29.jpg?raw=true)

# Future Work

1. Try the Test World problem 4, to see how the pipeline works there.
2. Complete the pick and place problem by adding collision avoidance
3. Try to implement different Machine Learning Algorithm, to see their performance relative to SVM.