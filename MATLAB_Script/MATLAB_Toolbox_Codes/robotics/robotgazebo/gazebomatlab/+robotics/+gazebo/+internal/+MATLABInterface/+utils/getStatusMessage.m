function [status, errorMessage, printStatus, printMessage] = ...
        getStatusMessage(jointName, returnMessage, p, setType, varargin)
    %This function is for internal use only. It may be removed in the future.
    %
    % This function returns status and error message in array as well as
    % printable form based on received message from Gazebo for set parameter
    % operation.

    %   Copyright 2020 The MathWorks, Inc.

    % get valid input parameter names
    inputParamNames = fieldnames(p.Results);
    validParamNames = inputParamNames(~structfun(@isempty,p.Results));

    % 'validParamNames' are in alphabetical order, while user input can be
    % in some other order. Thus 'validParamNames' need to rearrange in user
    % input order such that output will be in same order
    inputParamSeq = varargin{1}(1:2:end);
    for validSeqIdx = 1:numel(inputParamSeq)
        inputParamSeq{validSeqIdx} = validatestring(inputParamSeq{validSeqIdx},validParamNames);
    end
    validParamNames = inputParamSeq';

    % get joint type and error message
    % received message contains joint type first and error message in
    % second
    jointParamMsg = strsplit(returnMessage.error_message,':');
    if(strcmp(returnMessage.error_message,''))
        jointType = '';
        errorMsg = '';
    else
        jointType = jointParamMsg{1};
        errorMsg = jointParamMsg{2};
    end

    % get parameter name which are not set ( not supported by Gazebo )
    errorMessageParam =  strsplit(errorMsg,';');
    % get status true, if valid parameter is not in error message list
    paramStatus = ~ismember(validParamNames',errorMessageParam);

    status = [];
    errorMessage = [];
    errorParam = "";

    % create status array and error message string array based on valid
    % parameter names and error parameter
    for statusIdx = 1:numel(paramStatus)
        if(~strcmp(validParamNames(statusIdx),'Axis'))
            if(paramStatus(statusIdx))
                status = logical([status, paramStatus(statusIdx)]);
                msg = join([validParamNames(statusIdx)," parameter set successfully."],"");
                errorMessage = string([ errorMessage, msg]);
            else
                status = logical([status,paramStatus(statusIdx)]);
                if(strcmp(setType,'setJoint'))
                    % message for SetJoint operations
                    msg = join(["Failed to set ",validParamNames(statusIdx)," parameter.",...
                                " Setting ",validParamNames(statusIdx)," parameter for ",...
                                jointType," joint type of ",...
                                jointName," joint is not supported by Gazebo."],"");
                else
                    % message for SetLink and SetModel operations
                    msg = join(["Failed to set ",validParamNames(statusIdx)," parameter.",...
                                " Setting ",validParamNames(statusIdx)," parameter ",...
                                " is not supported by Gazebo."],"");
                end
                errorMessage = string([ errorMessage, msg]);
                errorParam = join([errorParam,validParamNames(statusIdx)]," ");
            end
        end
    end

    % create printable status and error message based received error message
    printStatus = all(status);
    if(strcmp(errorParam,""))
        printMessage = "Parameter set successfully.";
    else
        if(strcmp(setType,'setJoint'))
            printMessage = join(["Failed to set",errorParam," parameter.",...
                                " Setting",errorParam," parameter for ",...
                                jointType," joint type of ",...
                                jointName," joint is not supported by Gazebo."],"");
        else
            printMessage = join(["Failed to set",errorParam," parameter.",...
                                " Setting",errorParam," parameter ",...
                                " is not supported by Gazebo."],"");
        end
    end

end
