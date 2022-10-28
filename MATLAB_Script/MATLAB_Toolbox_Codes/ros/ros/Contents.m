% ROS Toolbox
% Version 1.5 (R2022a) 13-Nov-2021
%
% Network Connection and Exploration
%   rosinit            - Initialize the ROS system
%   rosshutdown        - Shut down the ROS system
%   rosaction          - Get information about actions in the ROS network
%   rosmsg             - Get information about messages and message types
%   rosnode            - Get information about nodes in the ROS network
%   rosservice         - Get information about services in the ROS network
%   rostopic           - Get information about topics in the ROS network
%   rosparam           - Get and set values on the parameter server
%   rosdevice          - Connect to remote ROS device
%   ros2device         - Connect to remote ROS 2 device
%
%   ros2               - Retrieve information about ROS 2 network
%
% Publishers and Subscribers
%   rosmessage         - Create a ROS message
%   rostype            - View available ROS message types
%   rospublisher       - Create a ROS publisher
%   rossubscriber      - Create a ROS subscriber
%
%   ros2message        - Create a ROS 2 message structure
%   ros2node           - Create a ROS 2 node on the specified network
%   ros2publisher      - Create a ROS 2 publisher
%   ros2subscriber     - Create a ROS 2 subscriber
%
% Services and Actions
%   rossvcclient       - Create a ROS service client
%   rossvcserver       - Create a ROS service server
%   ros2svcclient      - Create a ROS 2 service client
%   ros2svcserver      - Create a ROS 2 service server
%   rosactionclient    - Create a ROS action client
%   rosactionserver    - Create a ROS action server
%   rosActionServerExecuteGoalFcn - Return a function handle for action server callback
%
% ROS Log Files and Transformations
%   rosbag             - Open and parse a rosbag log file
%   rosbagreader       - Access rosbag log file information
%   rosbagwriter       - Write messages to rosbag log files
%   rostime            - Access ROS time functionality
%   rosrate            - Execute fixed-frequency loop using ROS time
%   rosduration        - Create a ROS duration object
%   rostf              - Receive, send, and apply ROS transformations
%
% ROS 2 Log Files and Transformations
%   ros2bag            - Open and parse a ros2bag log file
%
% ROS Custom Message Support
%   rosgenmsg          - Generate custom messages from ROS definitions
%   ros2genmsg         - Generate custom messages from ROS 2 definitions
%   rosAddons          - Install add-ons for ROS Toolbox
%
% ROS Velodyne Interpretation
%   velodyneROSMessageReader    - Create a Velodyne ROS message reader object
%
% Convenience Functions for Specialize Messages
%   rosApplyTransform           - Apply transform to struct message entities
%   rosPlot                     - Plot LiDAR scan data or point cloud data
%   rosReadAllFieldNames        - Return all field names in a PointCloud2 struct message
%   rosReadBinaryOccupancyGrid  - Return a BinaryOccupancyGrid object given OccupancyGrid struct message
%   rosReadCartesian            - Return Cartesian (XY) coordinates for a LaserScan struct message
%   rosReadField                - Read data based on given field name in a PointCloud2 struct message
%   rosReadImage                - Convert a Image struct message into a MATLAB image
%   rosReadLidarScan            - Return an object for 2D lidar scan from a LaserScan struct message
%   rosReadOccupancyGrid        - Return an occupancyMap object from a OccupancyGrid struct message
%   rosReadOccupancyMap3D       - Return an occupancyMap3D object from a Octomap struct message
%   rosReadQuaternion           - Return the quaternion from a struct message
%   rosReadRGB                  - Return the RGB color matrix from a PointCloud2 struct message
%   rosReadScanAngles           - Return the scan angles from a PointCloud2 struct message
%   rosReadXYZ                  - Return the (x,y,z) coordinates from a PointCloud2 struct message
%   rosShowDetails              - Print the details of a struct message recursively
%   rosWriteBinaryOccupancyGrid - Write BinaryOccupancyMap to a OccupancyGrid struct message
%   rosWriteCameraInfo          - Write data from stereoParameters or cameraParameters structure to ROS message
%   rosWriteImage               - Write a MATLAB image to a Image struct message
%   rosWriteOccupancyGrid       - Write OccupancyMap to a OccupancyGrid struct message
%
% <a href="matlab:demo('toolbox','ROS')">View examples</a> for ROS Toolbox.

% Copyright 2013-2021 The MathWorks, Inc.
