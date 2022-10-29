function [data, info] = mapFeature
%MapFeature gives an empty data for geographic_msgs/MapFeature

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geographic_msgs/MapFeature';
[data.Id, info.Id] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.Id.MLdataType = 'struct';
[data.Components, info.Components] = ros.internal.ros.messages.uuid_msgs.uniqueID;
info.Components.MLdataType = 'struct';
info.Components.MaxLen = NaN;
info.Components.MinLen = 0;
data.Components = data.Components([],1);
[data.Props, info.Props] = ros.internal.ros.messages.geographic_msgs.keyValue;
info.Props.MLdataType = 'struct';
info.Props.MaxLen = NaN;
info.Props.MinLen = 0;
data.Props = data.Props([],1);
info.MessageType = 'geographic_msgs/MapFeature';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'id';
info.MatPath{2} = 'id.uuid';
info.MatPath{3} = 'components';
info.MatPath{4} = 'components.uuid';
info.MatPath{5} = 'props';
info.MatPath{6} = 'props.key';
info.MatPath{7} = 'props.value';
