function slBusStruct = getBusStructForROSMsg(rosMsgIn, msgMetaData, rosver)
% This function is for internal use only. It may be removed in the future.

% GETBUSSTRUCTFORROSMSG Returns a MATLAB structure that can be used
% as Simulink.Bus element type to create Simulink bus definition from empty
% ROS message, ROSMSGIN and the meta-information of the message,
% MSGMETADATA. The method recursively populates the output, SLBUSSTRUCT,
% with additional sub-structure members of a nested message.

%   Copyright 2019-2020 The MathWorks, Inc.

    slBusStruct = struct; % initialize with empty struct
    if (ismember('MessageType', fieldnames(msgMetaData)))
        % add MessageType field
        slBusStruct.MessageType = msgMetaData.MessageType;
        if (numel(rosMsgIn) > 1) && ~isrow(rosMsgIn)
            % convert to row-vector (for all 1d nested arrays)
            rosMsgIn = rosMsgIn';
        end
    elseif (ismember('MLdataType', fieldnames(msgMetaData)))
        switch (msgMetaData.MLdataType)
          case {'string', 'char'}
            slBusStruct = rosMsgIn;
          otherwise
            slBusStruct = feval(msgMetaData.MLdataType, rosMsgIn);
        end
        return;
    end
    thisMsgStruct = rosMsgIn;
    thisInfoStruct = msgMetaData;
    thisFieldNames = fieldnames(thisMsgStruct);
    % Remove MessageType field from use
    thisFieldNames(strcmp(thisFieldNames, 'MessageType')) = [];
    arrSize = size(thisMsgStruct);
    for ii = 1:numel(thisFieldNames)
        for arrIter = 1:arrSize(2)
            if (isempty(thisMsgStruct))
                thisMsgStruct = ros.internal.getEmptyMessage(thisInfoStruct.MessageType,rosver);
            end
            slBusStruct(arrIter).(thisFieldNames{ii}) = ...
                ros.slros.internal.bus.getBusStructForROSMsg( ...
                    thisMsgStruct(arrIter).(thisFieldNames{ii}), ...
                    thisInfoStruct.(thisFieldNames{ii}), ...
                    rosver);
        end
    end
end
