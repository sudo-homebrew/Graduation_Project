function h = addNodeToModel(node,model,name,defaultHeight,defaultWidth)
% Add a Simulink block equivalent to the node in the model and return the
% block handle.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

if startsWith(name,'F')
    NodeType  = 'track fuser';
    %For fuser blocks input is tracks.
    InputType = 'Tracks';
else
    NodeType  = 'tracker';
    %For tracker blocks input is Detections.
    InputType = 'Detections';
end

if isa(node,'fusion.trackingArchitecture.Tracker') ||...
        isa(node,'fusion.trackingArchitecture.TrackFuser')
    %Node is a custom node.
    if ismethod(node,'exportToSimulink')
        try
            h = node.exportToSimulink(model,name);
            blkName = get_param(h,'Name');
        catch ME
            switch ME.identifier
                case 'Simulink:Commands:InvSimulinkObjHandle'
                    % Invalid block handle.
                    causeException = MException(message('fusion:simulink:trackingArchitecture:InvalidBlockHandle',NodeType,name));
                    ME = addCause(ME,causeException);
                    throwAsCaller(ME);
                case {'MATLAB:TooManyInputs','MATLAB:minrhs'}
                    % Incorrect function arguments in exportToSimulink
                    % function.
                    causeException = MException(message('fusion:simulink:trackingArchitecture:IncorrectFuncArgs',NodeType,name));
                    ME = addCause(ME,causeException);
                    throwAsCaller(ME);
                otherwise
                    throwAsCaller(ME);
            end
        end
        %Verify that the block with correct name is added.
        assert(strcmpi(blkName,name),...
            message('fusion:simulink:trackingArchitecture:IncorrectBlockName',NodeType,name,name));

        portHndls = get_param(h,'PortHandles');
        numInputPorts = numel(portHndls.Inport);
        numOutputPorts = numel(portHndls.Outport);
        %The custom node must have at least 2 input ports and one output
        %port.
        assert(numInputPorts >= 2 && numOutputPorts >= 1,message('fusion:simulink:trackingArchitecture:InvalidPorts',NodeType,name,InputType));
    else
        %Node does not implement exportToSimulink function.
        error(message('fusion:simulink:trackingArchitecture:ExportFcnNotImplemented',NodeType,name));
    end
else
    %Node is a tracker or fuser shipping with SFTT.
    h = node.exportToSimulink('Model',model,'BlockName',name,'OpenModel',false);
end
%Make sure that trackers and fusers in the model have correct size.
fusion.simulink.internal.trackingArchitecture.resizeBlock(h,defaultHeight,defaultWidth);
end