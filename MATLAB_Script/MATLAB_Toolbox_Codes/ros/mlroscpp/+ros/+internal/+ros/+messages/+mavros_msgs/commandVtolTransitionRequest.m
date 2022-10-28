function [data, info] = commandVtolTransitionRequest
%CommandVtolTransition gives an empty data for mavros_msgs/CommandVtolTransitionRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/CommandVtolTransitionRequest';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.STATEMC, info.STATEMC] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.STATEFW, info.STATEFW] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'mavros_msgs/CommandVtolTransitionRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'STATE_MC';
info.MatPath{8} = 'STATE_FW';
info.MatPath{9} = 'state';
