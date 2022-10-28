function [data, info] = actuatorControl
%ActuatorControl gives an empty data for mavros_msgs/ActuatorControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/ActuatorControl';
[data.PX4MIXFLIGHTCONTROL, info.PX4MIXFLIGHTCONTROL] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.PX4MIXFLIGHTCONTROLVTOLALT, info.PX4MIXFLIGHTCONTROLVTOLALT] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.PX4MIXPAYLOAD, info.PX4MIXPAYLOAD] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.PX4MIXMANUALPASSTHROUGH, info.PX4MIXMANUALPASSTHROUGH] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.GroupMix, info.GroupMix] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Controls, info.Controls] = ros.internal.ros.messages.ros.default_type('single',8);
info.MessageType = 'mavros_msgs/ActuatorControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'PX4_MIX_FLIGHT_CONTROL';
info.MatPath{2} = 'PX4_MIX_FLIGHT_CONTROL_VTOL_ALT';
info.MatPath{3} = 'PX4_MIX_PAYLOAD';
info.MatPath{4} = 'PX4_MIX_MANUAL_PASSTHROUGH';
info.MatPath{5} = 'header';
info.MatPath{6} = 'header.seq';
info.MatPath{7} = 'header.stamp';
info.MatPath{8} = 'header.stamp.sec';
info.MatPath{9} = 'header.stamp.nsec';
info.MatPath{10} = 'header.frame_id';
info.MatPath{11} = 'group_mix';
info.MatPath{12} = 'controls';
