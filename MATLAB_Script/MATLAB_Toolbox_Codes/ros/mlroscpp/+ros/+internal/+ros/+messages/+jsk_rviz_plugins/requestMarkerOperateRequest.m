function [data, info] = requestMarkerOperateRequest
%RequestMarkerOperate gives an empty data for jsk_rviz_plugins/RequestMarkerOperateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_rviz_plugins/RequestMarkerOperateRequest';
[data.Operate, info.Operate] = ros.internal.ros.messages.jsk_rviz_plugins.transformableMarkerOperate;
info.Operate.MLdataType = 'struct';
info.MessageType = 'jsk_rviz_plugins/RequestMarkerOperateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'operate';
info.MatPath{2} = 'operate.BOX';
info.MatPath{3} = 'operate.CYLINDER';
info.MatPath{4} = 'operate.TORUS';
info.MatPath{5} = 'operate.MESH_RESOURCE';
info.MatPath{6} = 'operate.INSERT';
info.MatPath{7} = 'operate.ERASE';
info.MatPath{8} = 'operate.ERASEALL';
info.MatPath{9} = 'operate.ERASEFOCUS';
info.MatPath{10} = 'operate.COPY';
info.MatPath{11} = 'operate.type';
info.MatPath{12} = 'operate.action';
info.MatPath{13} = 'operate.frame_id';
info.MatPath{14} = 'operate.name';
info.MatPath{15} = 'operate.description';
info.MatPath{16} = 'operate.mesh_resource';
info.MatPath{17} = 'operate.mesh_use_embedded_materials';
