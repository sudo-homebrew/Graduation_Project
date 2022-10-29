function [data, info] = setJointPropertiesRequest
%SetJointProperties gives an empty data for gazebo_msgs/SetJointPropertiesRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/SetJointPropertiesRequest';
[data.JointName, info.JointName] = ros.internal.ros.messages.ros.char('string',0);
[data.OdeJointConfig, info.OdeJointConfig] = ros.internal.ros.messages.gazebo_msgs.oDEJointProperties;
info.OdeJointConfig.MLdataType = 'struct';
info.MessageType = 'gazebo_msgs/SetJointPropertiesRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'joint_name';
info.MatPath{2} = 'ode_joint_config';
info.MatPath{3} = 'ode_joint_config.damping';
info.MatPath{4} = 'ode_joint_config.hiStop';
info.MatPath{5} = 'ode_joint_config.loStop';
info.MatPath{6} = 'ode_joint_config.erp';
info.MatPath{7} = 'ode_joint_config.cfm';
info.MatPath{8} = 'ode_joint_config.stop_erp';
info.MatPath{9} = 'ode_joint_config.stop_cfm';
info.MatPath{10} = 'ode_joint_config.fudge_factor';
info.MatPath{11} = 'ode_joint_config.fmax';
info.MatPath{12} = 'ode_joint_config.vel';
