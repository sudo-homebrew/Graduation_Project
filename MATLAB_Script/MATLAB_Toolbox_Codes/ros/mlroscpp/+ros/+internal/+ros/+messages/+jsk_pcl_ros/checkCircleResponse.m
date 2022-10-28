function [data, info] = checkCircleResponse
%CheckCircle gives an empty data for jsk_pcl_ros/CheckCircleResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_pcl_ros/CheckCircleResponse';
[data.Clicked, info.Clicked] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Index, info.Index] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Msg, info.Msg] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'jsk_pcl_ros/CheckCircleResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'clicked';
info.MatPath{2} = 'index';
info.MatPath{3} = 'msg';
