function [data, info] = matchCloudsResponse
%MatchClouds gives an empty data for ethzasl_icp_mapper/MatchCloudsResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'ethzasl_icp_mapper/MatchCloudsResponse';
[data.Transform, info.Transform] = ros.internal.ros.messages.geometry_msgs.transform;
info.Transform.MLdataType = 'struct';
info.MessageType = 'ethzasl_icp_mapper/MatchCloudsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'transform';
info.MatPath{2} = 'transform.translation';
info.MatPath{3} = 'transform.translation.x';
info.MatPath{4} = 'transform.translation.y';
info.MatPath{5} = 'transform.translation.z';
info.MatPath{6} = 'transform.rotation';
info.MatPath{7} = 'transform.rotation.x';
info.MatPath{8} = 'transform.rotation.y';
info.MatPath{9} = 'transform.rotation.z';
info.MatPath{10} = 'transform.rotation.w';
