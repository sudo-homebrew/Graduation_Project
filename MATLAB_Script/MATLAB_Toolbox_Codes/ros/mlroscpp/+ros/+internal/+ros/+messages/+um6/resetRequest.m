function [data, info] = resetRequest
%Reset gives an empty data for um6/ResetRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'um6/ResetRequest';
[data.ZeroGyros, info.ZeroGyros] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ResetEkf, info.ResetEkf] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.SetMagRef, info.SetMagRef] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.SetAccelRef, info.SetAccelRef] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'um6/ResetRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'zero_gyros';
info.MatPath{2} = 'reset_ekf';
info.MatPath{3} = 'set_mag_ref';
info.MatPath{4} = 'set_accel_ref';
