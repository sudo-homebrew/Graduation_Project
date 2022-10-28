function allMsgsMap = getAllMessageInfoMapForModel(model, varargin)
%This function is for internal use only. It may be removed in the future.

% GETALLMESSAGEINFOMAPFORMODEL Get a map of all the ROS messages and buses
% with variable size properties
%
% See also ROS.SLROS.INTERNAL.SIM.CREATEROSMSGINFOMAP
% Copyright 2020 The MathWorks, Inc.

    validateattributes(model, {'char'}, {'nonempty'});
    p = inputParser;
    addRequired(p, 'modelName', @(x)validateattributes(x, {'char'}, {'nonempty'}));
    addParameter(p, 'BusUtilityObject', ros.slros.internal.bus.Util, @(x)isa(x, 'ros.slros.internal.bus.Util'));
    parse(p, model,varargin{:});
    model = p.Results.modelName;
    busUtilObj = p.Results.BusUtilityObject;
    topLevelMsgTypes = busUtilObj.getBlockLevelMessageTypesInModel(model);
    allMsgsMap = containers.Map;
    for i=1:numel(topLevelMsgTypes)
        map = ros.slros.internal.sim.createROSMsgInfoMap(struct(...
            'MessageType', topLevelMsgTypes{i}, ...
            'ModelName', model, ...
            'MapKeyType', 'msgtype', ...
            'Recurse', true), ...
                                                         'BusUtilityObject', busUtilObj);
        requiredMsgs = keys(map);
        for j=1:numel(requiredMsgs)
            if ~isKey(allMsgsMap, requiredMsgs{j})
                allMsgsMap(requiredMsgs{j}) = map(requiredMsgs{j});
            end
        end
    end
end
