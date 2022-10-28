function [data, info] = gAMSParams
%GAMSParams gives an empty data for applanix_msgs/GAMSParams

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/GAMSParams';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.AntennaSeparation, info.AntennaSeparation] = ros.internal.ros.messages.ros.default_type('single',1);
[data.BaselineVector, info.BaselineVector] = ros.internal.ros.messages.geometry_msgs.point32;
info.BaselineVector.MLdataType = 'struct';
[data.MaxHeadingErrorRms, info.MaxHeadingErrorRms] = ros.internal.ros.messages.ros.default_type('single',1);
[data.HeadingCorrection, info.HeadingCorrection] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'applanix_msgs/GAMSParams';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'antenna_separation';
info.MatPath{3} = 'baseline_vector';
info.MatPath{4} = 'baseline_vector.x';
info.MatPath{5} = 'baseline_vector.y';
info.MatPath{6} = 'baseline_vector.z';
info.MatPath{7} = 'max_heading_error_rms';
info.MatPath{8} = 'heading_correction';
