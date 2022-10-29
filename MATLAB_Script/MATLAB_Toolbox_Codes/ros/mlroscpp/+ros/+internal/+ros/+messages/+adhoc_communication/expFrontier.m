function [data, info] = expFrontier
%ExpFrontier gives an empty data for adhoc_communication/ExpFrontier

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'adhoc_communication/ExpFrontier';
[data.FrontierElement, info.FrontierElement] = ros.internal.ros.messages.adhoc_communication.expFrontierElement;
info.FrontierElement.MLdataType = 'struct';
info.FrontierElement.MaxLen = NaN;
info.FrontierElement.MinLen = 0;
data.FrontierElement = data.FrontierElement([],1);
info.MessageType = 'adhoc_communication/ExpFrontier';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'frontier_element';
info.MatPath{2} = 'frontier_element.id';
info.MatPath{3} = 'frontier_element.detected_by_robot_str';
info.MatPath{4} = 'frontier_element.detected_by_robot';
info.MatPath{5} = 'frontier_element.robot_home_position_x';
info.MatPath{6} = 'frontier_element.robot_home_position_y';
info.MatPath{7} = 'frontier_element.x_coordinate';
info.MatPath{8} = 'frontier_element.y_coordinate';
