function [ptClouds, configurations, sensorPIDs, clusters] = lidarDetect(scene)
%LIDARDETECT Collect point clouds from all the lidar sensors in the scenario
%    ptClouds = LIDARDETECT(s) reports the point clouds from all lidar
%    sensors mounted on every platform in the trackingScenario, s.
%  
%    [...,configs] = LIDARDETECT(...), additionally, returns a struct
%    array of the configurations of each sensor at the detection
%    time.
%
%    [...,sensorConfigPIDs] = LIDARDETECT(...), additionally, returns a
%    column vector of all platform IDs that correspond to the configs.
%   
%    [...,clusters] = LIDARDETECT(...) additionally, returns a cell array
%    of cluster outputs from each lidar sensor.
%   
%    ptClouds is a cell array of point clouds, where each element is a
%    N-by-3 or P-by-Q-by-3 matrix defining the locations of the point
%    cloud. It is defined as a N-by-3 if HasOrganizedOutput property of the
%    sensor is set to false.
%  
%    configs is an array of struct, where each element defines the
%    configuration of the sensor. The fields of the struct are defined
%    <a href="matlab:help fusion.internal.interfaces.DataStructures/sensorConfigStruct">here</a>.
%   
%    clusters is a cell array of ground truth segmentation labels. Each
%    element of the cell array defines the cluster labels for the
%    corresponding point cloud.
%
%   % Example: Obtain point clouds from 2 platforms in the scenario
%   s = rng(0); % For repeatable result
%   ts = trackingScenario('UpdateRate', 1);
%   plat1 = platform(ts);
%   plat1.Trajectory.Position = [0,0,0];
%   lidar1 = monostaticLidarSensor('SensorIndex',1);
%   plat1.Sensors = lidar1;
%   plat2 = platform(ts);
%   plat2.Trajectory.Position = [100,0,0];
%   lidar2 = monostaticLidarSensor('SensorIndex',2);
%   plat2.Sensors = lidar2;
%   
%   advance(ts);
%
%   [ptClouds, configurations, sensorPIDs, clusters] = lidarDetect(ts);
%
%   % Return the random number generator to its previous state
%   rng(s)
%
%   See also trackingScenario, <a href="matlab:help('fusion.scenario.Platform/lidarDetect')">lidarDetect</a>,
%   detect.

%   Copyright 2020 The MathWorks, Inc.

narginchk(1,1);

time = scene.SimulationTime;
platforms = scene.Platforms;
nPlat = numel(platforms);

ptClouds = cell(0,1);
configurations = [];
sensorPIDs = [];
clusters = {};

for iPlat = 1:nPlat
    platform = platforms{iPlat};
    [pc, conf, clus] = platform.lidarDetect(time);
    ptClouds = [ptClouds;pc];
    sensorPIDs = [sensorPIDs;repmat(platform.PlatformID,size(conf))]; %#ok<*AGROW>
    configurations = [configurations;conf];
    clusters = [clusters;clus];
end

