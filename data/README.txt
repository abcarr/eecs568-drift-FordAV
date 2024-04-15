Created a filtered Sample-Data.bag file from the FordAV repo using the following command

rosbag filter ../Sample-Data/Sample-Data.bag ../Sample-Data/Sample-Data_filtered.bag "topic != '/image_front_left' and topic != '/lidar_blue_scan' and topic != '/lidar_green_scan' and topic != '/lidar_red_scan' and topic != '/lidar_yellow_scan'"


drososmj@Drosos:~/src/final_project/AVData$ rosbag info ../Sample-Data/Sample-Data_filtered.bag
path:        ../Sample-Data/Sample-Data_filtered.bag
version:     2.0
duration:    27.3s
start:       Aug 04 2017 00:48:43.03 (1501822123.03)
end:         Aug 04 2017 00:49:10.30 (1501822150.30)
size:        5.5 MB
messages:    34687
compression: none [7/7 chunks]
types:       geometry_msgs/PoseStamped    [d3812c3cbc69362b77dc0b19b345f8f5]
             geometry_msgs/Vector3Stamped [7b324c7325e683bf02a9b14b01090ec7]
             sensor_msgs/Imu              [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/NavSatFix        [2d3a8cd499b9b4a0249fb98fd05cfa48]
             sensor_msgs/TimeReference    [fded64a0265108ba86c3d38fb11c0c16]
             tf2_msgs/TFMessage           [94810edda583a504dfda3829e70d7eec]
topics:      /gps                 4896 msgs    : sensor_msgs/NavSatFix
             /gps_time            4896 msgs    : sensor_msgs/TimeReference
             /imu                 4896 msgs    : sensor_msgs/Imu
             /pose_ground_truth   4858 msgs    : geometry_msgs/PoseStamped
             /pose_localized       539 msgs    : geometry_msgs/PoseStamped
             /pose_raw            4872 msgs    : geometry_msgs/PoseStamped
             /tf                  4858 msgs    : tf2_msgs/TFMessage
             /velocity_raw        4872 msgs    : geometry_msgs/Vector3Stamped




rosbag filter 2017-10-26-V2-Log4.bag 2017-10-26-V2-Log4_filtered.bag "topic != '/image_front_left' and topic != '/lidar_blue_scan' and topic != '/lidar_green_scan' and topic != '/lidar_red_scan' and topic != '/lidar_yellow_scan' and t.secs<=1508990476.39"

drososmj@Drosos:~/src/final_project/eecs568-drift-FordAV$
drososmj@Drosos:~/src/final_project/eecs568-drift-FordAV$ rosbag info data/2017-10-26-V2-Log4_filtered_trimmed.bag
path:        data/2017-10-26-V2-Log4_filtered_trimmed.bag
version:     2.0
duration:    1:00s (60s)
start:       Oct 26 2017 00:00:16.39 (1508990416.39)
end:         Oct 26 2017 00:01:16.00 (1508990477.00)
size:        12.7 MB
messages:    80549
compression: none [16/16 chunks]
types:       geometry_msgs/PoseStamped    [d3812c3cbc69362b77dc0b19b345f8f5]
             geometry_msgs/Vector3Stamped [7b324c7325e683bf02a9b14b01090ec7]
             sensor_msgs/Imu              [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/NavSatFix        [2d3a8cd499b9b4a0249fb98fd05cfa48]
             sensor_msgs/TimeReference    [fded64a0265108ba86c3d38fb11c0c16]
             tf2_msgs/TFMessage           [94810edda583a504dfda3829e70d7eec]
topics:      /gps                 11368 msgs    : sensor_msgs/NavSatFix
             /gps_time            11368 msgs    : sensor_msgs/TimeReference
             /imu                 11368 msgs    : sensor_msgs/Imu
             /pose_ground_truth   11302 msgs    : geometry_msgs/PoseStamped
             /pose_localized       1205 msgs    : geometry_msgs/PoseStamped
             /pose_raw            11318 msgs    : geometry_msgs/PoseStamped
             /tf                  11302 msgs    : tf2_msgs/TFMessage
             /velocity_raw        11318 msgs    : geometry_msgs/Vector3Stamped
