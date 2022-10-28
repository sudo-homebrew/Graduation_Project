function [data, info] = uRDFConfiguration
%URDFConfiguration gives an empty data for baxter_core_msgs/URDFConfiguration

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/URDFConfiguration';
[data.Time, info.Time] = ros.internal.ros.messages.ros.time;
info.Time.MLdataType = 'struct';
[data.Link, info.Link] = ros.internal.ros.messages.ros.char('string',0);
[data.Joint, info.Joint] = ros.internal.ros.messages.ros.char('string',0);
[data.Urdf, info.Urdf] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'baxter_core_msgs/URDFConfiguration';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'time';
info.MatPath{2} = 'time.sec';
info.MatPath{3} = 'time.nsec';
info.MatPath{4} = 'link';
info.MatPath{5} = 'joint';
info.MatPath{6} = 'urdf';
