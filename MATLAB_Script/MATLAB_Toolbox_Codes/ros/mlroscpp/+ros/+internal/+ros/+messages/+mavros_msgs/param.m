function [data, info] = param
%Param gives an empty data for mavros_msgs/Param

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/Param';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.ParamId, info.ParamId] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.mavros_msgs.paramValue;
info.Value.MLdataType = 'struct';
[data.ParamIndex, info.ParamIndex] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.ParamCount, info.ParamCount] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'mavros_msgs/Param';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'param_id';
info.MatPath{8} = 'value';
info.MatPath{9} = 'value.integer';
info.MatPath{10} = 'value.real';
info.MatPath{11} = 'param_index';
info.MatPath{12} = 'param_count';
