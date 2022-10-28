function [data, info] = multiTouch
%MultiTouch gives an empty data for jsk_gui_msgs/MultiTouch

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/MultiTouch';
[data.Touches, info.Touches] = ros.internal.ros.messages.jsk_gui_msgs.touch;
info.Touches.MLdataType = 'struct';
info.Touches.MaxLen = NaN;
info.Touches.MinLen = 0;
data.Touches = data.Touches([],1);
info.MessageType = 'jsk_gui_msgs/MultiTouch';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'touches';
info.MatPath{2} = 'touches.finger_id';
info.MatPath{3} = 'touches.x';
info.MatPath{4} = 'touches.y';
info.MatPath{5} = 'touches.image_width';
info.MatPath{6} = 'touches.image_height';
