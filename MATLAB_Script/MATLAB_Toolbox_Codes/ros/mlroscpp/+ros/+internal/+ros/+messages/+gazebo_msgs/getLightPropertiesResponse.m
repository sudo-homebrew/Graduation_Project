function [data, info] = getLightPropertiesResponse
%GetLightProperties gives an empty data for gazebo_msgs/GetLightPropertiesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/GetLightPropertiesResponse';
[data.Diffuse, info.Diffuse] = ros.internal.ros.messages.std_msgs.colorRGBA;
info.Diffuse.MLdataType = 'struct';
[data.AttenuationConstant, info.AttenuationConstant] = ros.internal.ros.messages.ros.default_type('double',1);
[data.AttenuationLinear, info.AttenuationLinear] = ros.internal.ros.messages.ros.default_type('double',1);
[data.AttenuationQuadratic, info.AttenuationQuadratic] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.StatusMessage, info.StatusMessage] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gazebo_msgs/GetLightPropertiesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'diffuse';
info.MatPath{2} = 'diffuse.r';
info.MatPath{3} = 'diffuse.g';
info.MatPath{4} = 'diffuse.b';
info.MatPath{5} = 'diffuse.a';
info.MatPath{6} = 'attenuation_constant';
info.MatPath{7} = 'attenuation_linear';
info.MatPath{8} = 'attenuation_quadratic';
info.MatPath{9} = 'success';
info.MatPath{10} = 'status_message';
