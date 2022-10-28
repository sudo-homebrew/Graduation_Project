function [data, info] = towerPickUpRequest
%TowerPickUp gives an empty data for jsk_pcl_ros/TowerPickUpRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/TowerPickUpRequest';
[data.TowerIndex, info.TowerIndex] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'jsk_pcl_ros/TowerPickUpRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'tower_index';
