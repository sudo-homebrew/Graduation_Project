function [data, info] = setPhysicsPropertiesRequest
%SetPhysicsProperties gives an empty data for gazebo_msgs/SetPhysicsPropertiesRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/SetPhysicsPropertiesRequest';
[data.TimeStep, info.TimeStep] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MaxUpdateRate, info.MaxUpdateRate] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Gravity, info.Gravity] = ros.internal.ros.messages.geometry_msgs.vector3;
info.Gravity.MLdataType = 'struct';
[data.OdeConfig, info.OdeConfig] = ros.internal.ros.messages.gazebo_msgs.oDEPhysics;
info.OdeConfig.MLdataType = 'struct';
info.MessageType = 'gazebo_msgs/SetPhysicsPropertiesRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'time_step';
info.MatPath{2} = 'max_update_rate';
info.MatPath{3} = 'gravity';
info.MatPath{4} = 'gravity.x';
info.MatPath{5} = 'gravity.y';
info.MatPath{6} = 'gravity.z';
info.MatPath{7} = 'ode_config';
info.MatPath{8} = 'ode_config.auto_disable_bodies';
info.MatPath{9} = 'ode_config.sor_pgs_precon_iters';
info.MatPath{10} = 'ode_config.sor_pgs_iters';
info.MatPath{11} = 'ode_config.sor_pgs_w';
info.MatPath{12} = 'ode_config.sor_pgs_rms_error_tol';
info.MatPath{13} = 'ode_config.contact_surface_layer';
info.MatPath{14} = 'ode_config.contact_max_correcting_vel';
info.MatPath{15} = 'ode_config.cfm';
info.MatPath{16} = 'ode_config.erp';
info.MatPath{17} = 'ode_config.max_contacts';
