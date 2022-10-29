function [data, info] = getRoutePlanResponse
%GetRoutePlan gives an empty data for geographic_msgs/GetRoutePlanResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geographic_msgs/GetRoutePlanResponse';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Status, info.Status] = ros.internal.ros.messages.ros.char('string',0);
[data.Plan, info.Plan] = ros.internal.ros.messages.geographic_msgs.routePath;
info.Plan.MLdataType = 'struct';
info.MessageType = 'geographic_msgs/GetRoutePlanResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'success';
info.MatPath{2} = 'status';
info.MatPath{3} = 'plan';
info.MatPath{4} = 'plan.header';
info.MatPath{5} = 'plan.header.seq';
info.MatPath{6} = 'plan.header.stamp';
info.MatPath{7} = 'plan.header.stamp.sec';
info.MatPath{8} = 'plan.header.stamp.nsec';
info.MatPath{9} = 'plan.header.frame_id';
info.MatPath{10} = 'plan.network';
info.MatPath{11} = 'plan.network.uuid';
info.MatPath{12} = 'plan.segments';
info.MatPath{13} = 'plan.segments.uuid';
info.MatPath{14} = 'plan.props';
info.MatPath{15} = 'plan.props.key';
info.MatPath{16} = 'plan.props.value';
