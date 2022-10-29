function [data, info] = pressureState
%PressureState gives an empty data for pr2_msgs/PressureState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_msgs/PressureState';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.LFingerTip, info.LFingerTip] = ros.internal.ros.messages.ros.default_type('int16',NaN);
[data.RFingerTip, info.RFingerTip] = ros.internal.ros.messages.ros.default_type('int16',NaN);
info.MessageType = 'pr2_msgs/PressureState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'l_finger_tip';
info.MatPath{8} = 'r_finger_tip';
