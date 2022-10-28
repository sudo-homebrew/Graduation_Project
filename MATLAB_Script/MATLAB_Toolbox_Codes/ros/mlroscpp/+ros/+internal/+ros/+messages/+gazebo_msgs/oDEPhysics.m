function [data, info] = oDEPhysics
%ODEPhysics gives an empty data for gazebo_msgs/ODEPhysics

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/ODEPhysics';
[data.AutoDisableBodies, info.AutoDisableBodies] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.SorPgsPreconIters, info.SorPgsPreconIters] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.SorPgsIters, info.SorPgsIters] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.SorPgsW, info.SorPgsW] = ros.internal.ros.messages.ros.default_type('double',1);
[data.SorPgsRmsErrorTol, info.SorPgsRmsErrorTol] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ContactSurfaceLayer, info.ContactSurfaceLayer] = ros.internal.ros.messages.ros.default_type('double',1);
[data.ContactMaxCorrectingVel, info.ContactMaxCorrectingVel] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Cfm, info.Cfm] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Erp, info.Erp] = ros.internal.ros.messages.ros.default_type('double',1);
[data.MaxContacts, info.MaxContacts] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'gazebo_msgs/ODEPhysics';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'auto_disable_bodies';
info.MatPath{2} = 'sor_pgs_precon_iters';
info.MatPath{3} = 'sor_pgs_iters';
info.MatPath{4} = 'sor_pgs_w';
info.MatPath{5} = 'sor_pgs_rms_error_tol';
info.MatPath{6} = 'contact_surface_layer';
info.MatPath{7} = 'contact_max_correcting_vel';
info.MatPath{8} = 'cfm';
info.MatPath{9} = 'erp';
info.MatPath{10} = 'max_contacts';
