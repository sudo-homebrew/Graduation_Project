function [data, info] = shapeFeedback
%ShapeFeedback gives an empty data for turtle_actionlib/ShapeFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'turtle_actionlib/ShapeFeedback';
info.MessageType = 'turtle_actionlib/ShapeFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
