function busElements = setBusDimensions(elemInfo, elem, busElements, data, isVarsizeArray)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019-2020 The MathWorks, Inc.

% SETBUSDIMENSIONS Sets the dimension property of the bus element.
if isVarsizeArray
    elemInfo.IsVarLen = true;
    elem.DimensionsMode = 'Variable';
    % SL buses require non-zero value for dimensions, so set it to 1
    % for now. Conversion from Variable-length to Fixed-length array
    % will be done later (after getting user data on maximum lengths)
    elem.Dimensions = max(numel(data),1);
    elem.Description = elemInfo.toDescription();
    % Add the array and the corresponding SL_Info element to the bus
    [elem, arrayInfoElem] = addArrayInfoElement(elem);
    busElements(end+1:end+2) = [elem ; arrayInfoElem];
else
    elemInfo.IsVarLen = false;
    elem.DimensionsMode = 'Fixed';
    elem.Dimensions = max(numel(data),1);
    elem.Description = elemInfo.toDescription();
    busElements(end+1) = elem;
end
end


%%
function [arrayElem, arrayInfoElem] = addArrayInfoElement(arrayElem)
    assert(strcmp(arrayElem.DimensionsMode, 'Variable'));

    infoElemName = ros.slros.internal.bus.Util.getArrayInfoElementName(arrayElem.Name);

    busItemInfo = ros.slros.internal.bus.BusItemInfo(arrayElem.Description);
    busItemInfo.IsVarLen = true;
    busItemInfo.VarLenCategory = 'data';
    busItemInfo.VarLenElem = infoElemName;
    arrayElem.Dimensions = 1;  % temporary length
    arrayElem.DimensionsMode = 'Fixed';
    arrayElem.Description = busItemInfo.toDescription();

    busItemInfo2 = ros.slros.internal.bus.BusItemInfo;
    busItemInfo2.IsVarLen = true;
    busItemInfo2.VarLenCategory = 'length';
    busItemInfo2.VarLenElem = arrayElem.Name;

    arrayInfoElem = Simulink.BusElement;
    arrayInfoElem.Name = infoElemName;
    arrayInfoElem.DataType = ros.slros.internal.bus.Util.varlenInfoBusDataTypeStr();
    arrayInfoElem.DimensionsMode = 'Fixed';
    arrayInfoElem.Dimensions = 1;
    arrayInfoElem.SamplingMode = 'Sample based';
    arrayInfoElem.Description = busItemInfo2.toDescription();
end
