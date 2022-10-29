function [data, info] = sendExpFrontierRequest
%SendExpFrontier gives an empty data for adhoc_communication/SendExpFrontierRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/SendExpFrontierRequest';
[data.DstRobot, info.DstRobot] = ros.internal.ros.messages.ros.char('string',0);
[data.Topic, info.Topic] = ros.internal.ros.messages.ros.char('string',0);
[data.Frontier, info.Frontier] = ros.internal.ros.messages.adhoc_communication.expFrontier;
info.Frontier.MLdataType = 'struct';
info.MessageType = 'adhoc_communication/SendExpFrontierRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'dst_robot';
info.MatPath{2} = 'topic';
info.MatPath{3} = 'frontier';
info.MatPath{4} = 'frontier.frontier_element';
info.MatPath{5} = 'frontier.frontier_element.id';
info.MatPath{6} = 'frontier.frontier_element.detected_by_robot_str';
info.MatPath{7} = 'frontier.frontier_element.detected_by_robot';
info.MatPath{8} = 'frontier.frontier_element.robot_home_position_x';
info.MatPath{9} = 'frontier.frontier_element.robot_home_position_y';
info.MatPath{10} = 'frontier.frontier_element.x_coordinate';
info.MatPath{11} = 'frontier.frontier_element.y_coordinate';
