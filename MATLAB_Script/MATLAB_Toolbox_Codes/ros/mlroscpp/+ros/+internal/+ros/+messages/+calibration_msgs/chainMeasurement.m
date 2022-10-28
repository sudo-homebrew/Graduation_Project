function [data, info] = chainMeasurement
%ChainMeasurement gives an empty data for calibration_msgs/ChainMeasurement

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'calibration_msgs/ChainMeasurement';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.ChainId, info.ChainId] = ros.internal.ros.messages.ros.char('string',0);
[data.ChainState, info.ChainState] = ros.internal.ros.messages.sensor_msgs.jointState;
info.ChainState.MLdataType = 'struct';
info.MessageType = 'calibration_msgs/ChainMeasurement';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'chain_id';
info.MatPath{8} = 'chain_state';
info.MatPath{9} = 'chain_state.header';
info.MatPath{10} = 'chain_state.header.seq';
info.MatPath{11} = 'chain_state.header.stamp';
info.MatPath{12} = 'chain_state.header.stamp.sec';
info.MatPath{13} = 'chain_state.header.stamp.nsec';
info.MatPath{14} = 'chain_state.header.frame_id';
info.MatPath{15} = 'chain_state.name';
info.MatPath{16} = 'chain_state.position';
info.MatPath{17} = 'chain_state.velocity';
info.MatPath{18} = 'chain_state.effort';
