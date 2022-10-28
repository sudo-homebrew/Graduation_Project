function [data, info] = generateGraspsResult
%GenerateGraspsResult gives an empty data for cob_grasp_generation/GenerateGraspsResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_grasp_generation/GenerateGraspsResult';
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.NumGrasps, info.NumGrasps] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'cob_grasp_generation/GenerateGraspsResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'success';
info.MatPath{2} = 'num_grasps';
