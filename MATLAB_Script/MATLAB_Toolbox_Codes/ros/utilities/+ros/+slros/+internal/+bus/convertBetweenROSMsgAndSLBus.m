function ret = convertBetweenROSMsgAndSLBus(busstruct, msgMetaInfo, refRosMsg, direction, getCachedMsgFcn)
% This function is for internal use only. It may be removed in the future.

% convertBetweenROSMsgAndSLBus Function to convert from
% bus-struct to ROS message and vice-versa depending on the
% direction input ('BUS2MSG' or 'MSG2BUS')

%   Copyright 2019-2020 The MathWorks, Inc.

    ret = struct.empty(numel(busstruct),0);
    if (isfield(msgMetaInfo, 'MessageType'))
        % recurse and process the structure
        if isempty(refRosMsg)
            % for empty sub-structures
            refRosMsg = feval(getCachedMsgFcn, msgMetaInfo.MessageType);
        end
        allFieldNames = fieldnames(refRosMsg);
        % Remove MessageType field from use
        allFieldNames(strcmp(allFieldNames, 'MessageType')) = [];
        arrSize = numel(busstruct);
        for ii = 1:numel(allFieldNames)
            msgFieldName = allFieldNames{ii};
            thisInfo = msgMetaInfo.(msgFieldName);
            if isfield(thisInfo, 'constant') && thisInfo.constant
                if isequal(direction, 'BUS2MSG')
                    % for conversion from bus to message structure,
                    % add the constant fields again
                    for arrIter = 1:arrSize
                        [ret(arrIter,1).(msgFieldName)] = deal(refRosMsg(1).(msgFieldName));
                    end
                end
                % when converting from message to bus, remove
                % the constant fields
                continue;
            end
            for arrIter = 1:arrSize
                if isempty(busstruct(arrIter).(msgFieldName))
                    % if this is an empty message and is not a
                    % string, and it is a primitive then convert it
                    % to the correct data-type and assign the value
                    if isfield(thisInfo, 'MLdataType') && ~isequal(thisInfo.MLdataType, 'string')
                        if isequal(thisInfo.MLdataType,'struct')
                            ret(arrIter,1).(msgFieldName) = deal(feval(thisInfo.MLdataType, ...
                                    busstruct(arrIter).(msgFieldName) ...
                                    ));
                        else
                            ret(arrIter,1).(msgFieldName) = deal(cast(busstruct(arrIter).(msgFieldName),thisInfo.MLdataType));
                        end
                    else
                        % For empty strings and structures no need
                        % to cast the data-type
                        ret(arrIter,1).(msgFieldName) = deal(busstruct(arrIter).(msgFieldName));
                    end
                else
                    % recursion - for nested messages
                    ret(arrIter,1).(msgFieldName) = ros.slros.internal.bus.convertBetweenROSMsgAndSLBus(...
                            busstruct(arrIter).(msgFieldName), ...
                            thisInfo, ...
                            refRosMsg(1).(msgFieldName), ...
                            direction, getCachedMsgFcn);
                end
            end
        end
        if isempty(ret)
            ret = repmat(struct,numel(busstruct),1);
        end
    else
        switch (msgMetaInfo.MLdataType)
          case 'string'
            ret = busstruct;
          otherwise
            ret = cast(busstruct,msgMetaInfo.MLdataType);
        end
    end
end % function convertBetweenMsgAndBus
