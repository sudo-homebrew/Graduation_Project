function [data, info] = getKinematicSolverInfoResponse
%GetKinematicSolverInfo gives an empty data for iai_kinematics_msgs/GetKinematicSolverInfoResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'iai_kinematics_msgs/GetKinematicSolverInfoResponse';
[data.KinematicSolverInfo, info.KinematicSolverInfo] = ros.internal.ros.messages.iai_kinematics_msgs.kinematicSolverInfo;
info.KinematicSolverInfo.MLdataType = 'struct';
info.MessageType = 'iai_kinematics_msgs/GetKinematicSolverInfoResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'kinematic_solver_info';
info.MatPath{2} = 'kinematic_solver_info.joint_names';
info.MatPath{3} = 'kinematic_solver_info.limits';
info.MatPath{4} = 'kinematic_solver_info.limits.joint_name';
info.MatPath{5} = 'kinematic_solver_info.limits.has_position_limits';
info.MatPath{6} = 'kinematic_solver_info.limits.min_position';
info.MatPath{7} = 'kinematic_solver_info.limits.max_position';
info.MatPath{8} = 'kinematic_solver_info.limits.has_velocity_limits';
info.MatPath{9} = 'kinematic_solver_info.limits.max_velocity';
info.MatPath{10} = 'kinematic_solver_info.limits.has_acceleration_limits';
info.MatPath{11} = 'kinematic_solver_info.limits.max_acceleration';
info.MatPath{12} = 'kinematic_solver_info.link_names';
