function slamMapBuilder(varargin)
%SLAMMAPBUILDER Build a map from lidar scans and odometry.
%   slamMapBuilder opens a graph-SLAM-based map builder MATLAB app. The app
%   allows the user to load lidar scans and odometry data (optionally)
%   from various source, configure the SLAM algorithm and build a map
%   incrementally from loaded scan and odometry data. It also provides a
%   convenient interface to modify the unsatisfied scan-matching results
%   manually.
%
%   The sensor data source can also be specified programmatically.
%
%   slamMapBuilder(BAG) parses scans and tf data from a rosbag, where BAG
%   is the file path of the rosbag. ROS Toolbox license is required to use
%   this syntax.
%
%   slamMapBuilder(SESSIONFILE) opens an app session as previously saved in
%   SESSIONFILE. An app session file is created through the Save Session
%   button in the app toolstrip.
%
%   slamMapBuilder(SCANS) loads preprocessed lidar scan data from workspace,
%   where SCANS is a 1-D array or cell array of lidar scan objects.
%
%   slamMapBuilder(SCANS, POSES) loads preprocessed lidar scan data and
%   odometry data from workspace, where SCANS is a 1-D array or cell array 
%   of lidarScan objects and POSES is either a 1-D cell array of 1-by-3 
%   double vectors (x, y, theta), or a N-by-3 double matrix, where N is the
%   number of poses. N must match the number of entries in SCANS.
%
%   See also lidarSLAM, optimizePoseGraph

%   Copyright 2018-2020 The MathWorks, Inc.


    nav.slamapp.internal.SLAMMapBuilder(varargin{:});


end
