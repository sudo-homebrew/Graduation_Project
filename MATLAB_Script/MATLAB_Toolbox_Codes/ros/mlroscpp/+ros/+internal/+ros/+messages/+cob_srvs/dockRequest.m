function [data, info] = dockRequest
%Dock gives an empty data for cob_srvs/DockRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_srvs/DockRequest';
[data.FrameId, info.FrameId] = ros.internal.ros.messages.ros.char('string',0);
[data.Poses, info.Poses] = ros.internal.ros.messages.geometry_msgs.pose;
info.Poses.MLdataType = 'struct';
info.Poses.MaxLen = NaN;
info.Poses.MinLen = 0;
data.Poses = data.Poses([],1);
[data.StopTopic, info.StopTopic] = ros.internal.ros.messages.ros.char('string',0);
[data.StopMessageField, info.StopMessageField] = ros.internal.ros.messages.ros.char('string',0);
[data.StopCompareValue, info.StopCompareValue] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.DistThreshold, info.DistThreshold] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'cob_srvs/DockRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,15);
info.MatPath{1} = 'frame_id';
info.MatPath{2} = 'poses';
info.MatPath{3} = 'poses.position';
info.MatPath{4} = 'poses.position.x';
info.MatPath{5} = 'poses.position.y';
info.MatPath{6} = 'poses.position.z';
info.MatPath{7} = 'poses.orientation';
info.MatPath{8} = 'poses.orientation.x';
info.MatPath{9} = 'poses.orientation.y';
info.MatPath{10} = 'poses.orientation.z';
info.MatPath{11} = 'poses.orientation.w';
info.MatPath{12} = 'stop_topic';
info.MatPath{13} = 'stop_message_field';
info.MatPath{14} = 'stop_compare_value';
info.MatPath{15} = 'dist_threshold';
