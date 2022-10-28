function [data, info] = setDynamicTFRequest
%SetDynamicTF gives an empty data for dynamic_tf_publisher/SetDynamicTFRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'dynamic_tf_publisher/SetDynamicTFRequest';
[data.Freq, info.Freq] = ros.internal.ros.messages.ros.default_type('single',1);
[data.CurTf, info.CurTf] = ros.internal.ros.messages.geometry_msgs.transformStamped;
info.CurTf.MLdataType = 'struct';
info.MessageType = 'dynamic_tf_publisher/SetDynamicTFRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,19);
info.MatPath{1} = 'freq';
info.MatPath{2} = 'cur_tf';
info.MatPath{3} = 'cur_tf.header';
info.MatPath{4} = 'cur_tf.header.seq';
info.MatPath{5} = 'cur_tf.header.stamp';
info.MatPath{6} = 'cur_tf.header.stamp.sec';
info.MatPath{7} = 'cur_tf.header.stamp.nsec';
info.MatPath{8} = 'cur_tf.header.frame_id';
info.MatPath{9} = 'cur_tf.child_frame_id';
info.MatPath{10} = 'cur_tf.transform';
info.MatPath{11} = 'cur_tf.transform.translation';
info.MatPath{12} = 'cur_tf.transform.translation.x';
info.MatPath{13} = 'cur_tf.transform.translation.y';
info.MatPath{14} = 'cur_tf.transform.translation.z';
info.MatPath{15} = 'cur_tf.transform.rotation';
info.MatPath{16} = 'cur_tf.transform.rotation.x';
info.MatPath{17} = 'cur_tf.transform.rotation.y';
info.MatPath{18} = 'cur_tf.transform.rotation.z';
info.MatPath{19} = 'cur_tf.transform.rotation.w';
