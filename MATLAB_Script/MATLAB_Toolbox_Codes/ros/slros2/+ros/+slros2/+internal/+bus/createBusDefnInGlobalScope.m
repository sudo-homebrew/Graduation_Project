function createBusDefnInGlobalScope(emptyRosMsg, model)
%This function is for internal use only. It may be removed in the future.

%createBusDefnInGlobalScope - Create Simulink bus object in global scope
%
%   createBusDefnInGlobalScope(MSG, MODEL) creates the Simulink bus
%   objects in the global scope (base workspace or data dictionary attached
%   to model) corresponding to the ROS2 message MSG and
%   any nested messages inside it, using the variable-length array size
%   information from MODEL. MSG should  be an empty message, since the only
%   way to determine if a property is a variable-length array is to check
%   if its value is [].
%
%   createBusDefnInGlobalScope(MSG) creates the bus object using default
%   sizes for variable-length arrays.
%
%   Note:
%   * If MODEL is a Simulink block library, the default sizes are
%     applied.

%   Copyright 2019-2021 The MathWorks, Inc.
    if exist('model', 'var') && ~isempty(model)
        assert(ischar(model));
        isLibraryContext = strcmpi(get_param(bdroot(model), 'BlockDiagramType'), 'library');
        nullModel = isLibraryContext || bdIsSubsystem(model);
    else
        nullModel = true;
    end

    if nullModel
        model = '';
        varlenArrayStore = [];
        truncateAction = ros.slros.internal.bus.VarlenArraySizeStore.getDefaultTruncateAction();
    else
        varlenArrayStore = ros.slros.internal.bus.VarlenArraySizeStore(model, 'BusUtilityObject', ros.slros2.internal.bus.Util);
        truncateAction = varlenArrayStore.getTruncateAction();
    end

    requiredBuses = ros.slros.internal.bus.getBusDefnForROSMsg(emptyRosMsg, model, @ros.slros.internal.bus.defineSimulinkBus);
    createVarlenInfoBus = false;
    for i = 1:numel(requiredBuses)
        bus = requiredBuses(i).Bus;
        elemInfo = ros.slros.internal.bus.BusItemInfo( bus.Description );
        msgType = elemInfo.MsgType;
        busname = ros.slros2.internal.bus.Util.rosMsgTypeToBusName(msgType);

        varlenInfo = ros.slros.internal.bus.VarLenArrayInfo(bus);
        if varlenInfo.hasVarLenArrayProperties()
            if ~isempty(varlenArrayStore)
                % applyMaxLengths will apply user-customizations if present
                varlenArrayStore.applyMaxLengths(varlenInfo);
            else
                if ~nullModel
                    ros.slros.internal.bus.VarlenArraySizeStore.applyDefaultMaxLengths(varlenInfo);
                end
            end
            updatedBus = updateVarlenArrayLimitsInBus(bus, varlenInfo, truncateAction);
            ros.slros2.internal.bus.Util.registerSLBus(busname, updatedBus);
            createVarlenInfoBus = true;
        else
            % no variable-length arrays
            ros.slros2.internal.bus.Util.registerSLBus(busname, bus);
        end
    end
    
    if createVarlenInfoBus
        ros.slros2.internal.bus.Util.createVarlenInfoBusIfNeeded(model);
    end
end


%%
function [bus, requiresVarlenInfoBus] = updateVarlenArrayLimitsInBus(bus, varlenInfo, truncateAction)

    varlenProps = varlenInfo.PropertyNames;
    busElementNames = {bus.Elements.Name};

    for j = 1:numel(varlenProps)
        % find corresponding bus element
        idx = find(strcmp(busElementNames, varlenProps{j}));
        assert(numel(idx)==1);

        % sanity check - is bus element a variable-length array?
        assert(strcmpi(bus.Elements(idx).DimensionsMode, 'Fixed'));
        elemInfo = ros.slros.internal.bus.BusItemInfo( bus.Elements(idx).Description );
        assert(elemInfo.isVarLenDataElement);

        % update the max length for the bus element
        bus.Elements(idx).Dimensions = varlenInfo.getMaxLength(varlenProps{j});

        if truncateAction == ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning
            elemInfo.TruncateAction = 'warn';
            bus.Elements(idx).Description = elemInfo.toDescription();
        end
    end

    requiresVarlenInfoBus = ~isempty(varlenProps);
end
