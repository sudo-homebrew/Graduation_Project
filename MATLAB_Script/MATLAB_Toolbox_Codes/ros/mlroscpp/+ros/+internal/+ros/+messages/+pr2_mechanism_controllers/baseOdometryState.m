function [data, info] = baseOdometryState
%BaseOdometryState gives an empty data for pr2_mechanism_controllers/BaseOdometryState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_mechanism_controllers/BaseOdometryState';
[data.Velocity, info.Velocity] = ros.internal.ros.messages.geometry_msgs.twist;
info.Velocity.MLdataType = 'struct';
[data.WheelLinkNames, info.WheelLinkNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.DriveConstraintErrors, info.DriveConstraintErrors] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.LongitudinalSlipConstraintErrors, info.LongitudinalSlipConstraintErrors] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'pr2_mechanism_controllers/BaseOdometryState';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'velocity';
info.MatPath{2} = 'velocity.linear';
info.MatPath{3} = 'velocity.linear.x';
info.MatPath{4} = 'velocity.linear.y';
info.MatPath{5} = 'velocity.linear.z';
info.MatPath{6} = 'velocity.angular';
info.MatPath{7} = 'velocity.angular.x';
info.MatPath{8} = 'velocity.angular.y';
info.MatPath{9} = 'velocity.angular.z';
info.MatPath{10} = 'wheel_link_names';
info.MatPath{11} = 'drive_constraint_errors';
info.MatPath{12} = 'longitudinal_slip_constraint_errors';
