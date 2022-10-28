classdef HeaderAssignmentBlockMask
%This class is for internal use only. It may be removed in the future.

%HeaderAssignmentBlockMask - Block mask callbacks for "Header Assignment" block

%   Copyright 2020-2021 The MathWorks, Inc.
    properties (Constant)
        %MaskType - Type of block mask
        %   Retrieve is with get_param(gcb, 'MaskType')
        MaskType = 'ROS Header Assignment'
        % The following strings (used for the SetHeaderFieldName dropdown)
        % is directly specified on the block mask & don't use the message
        % catalog.
        % Rationale: Even though these strings are user-visible, the user
        % can also programmatically set the parameter using
        %   set_param(gcb, 'SetHeaderFieldName', 'Specify your own')
        % If the strings are internationalized, then the set_param would
        % have to be locale-specific
        HeaderFieldNameByDefault = 'Use the default Header field name'
    end


    methods
        function maskInitialize(~, block)
        %maskInitialize Initialize the block mask
        % It is invoked when the user:
        % * Changes the value of a mask parameter by using the block dialog box or set_param.
        % * Changes any of the parameters that define the mask
        % * Causes the icon to be redrawn
        % * Copies the block
            blkH = get_param(block, 'handle');
            ros.internal.setBlockIcon(blkH, 'rosicons.robotlib_headerassignment');
        end

        function checkSetFrameId(~,block)
        %checkSetFrameId The users checks the SetFrameID option in
        %the mask

        % If the SetFrameID is not checked then grey out the FrameID
        % box
            paramCheck = get_param(block,'SetFrameID');
            blockMask = Simulink.Mask.get(block);
            frameIdEditBox = blockMask.getParameter('FrameID');

            if strcmp(paramCheck,'off')
                set_param(block,'FrameID','');
                frameIdEditBox.Visible = 'off';
            else
                frameIdEditBox.Visible = 'on';
            end
        end

        function setFrameId(~,block)
        %setFrameId The user adds FrameID to their ROS message

        % Execute frame id setting only when the 'FrameID' is visible
            blockMask = Simulink.Mask.get(block);
            frameIdEditBox = blockMask.getParameter('FrameID');
            if (strcmp(frameIdEditBox.Visible,'on'))
                % Check if the added FrameID length is shorter than the set
                % VariableSize Message
                blockFullName = [get_param(block, 'Parent'),'/',...
                                 get_param(block, 'Name')];
                strToASCIIblk = [blockFullName '/String To ASCII'];
                modelName = bdroot(blockFullName);
                varlenSizeStore = ros.slros.internal.bus.VarlenArraySizeStore(modelName);
                msgInfo = ros.slros.internal.bus.VarLenArrayInfo(...
                    'std_msgs/Header', modelName);

                % This message has variable-length arrays
                msgtypeInfo = varlenSizeStore.getUserSpecifiedArrayInfo(...
                    'std_msgs/Header');
                if isempty(msgtypeInfo)
                    % use default values
                    msgtypeInfo = msgInfo;
                end

                %Get the length of the frameId set by user
                maxLength = msgtypeInfo.getMaxLength('FrameId');

                % Check if the length of the frameID is shorter than
                % VariableSize Message
                ros.slros.internal.block.HeaderAssignmentBlockMask.dispatch(...
                    'checkVarLenArrayErr', blockFullName, maxLength);

                % If the VariableSize Messages is changed, the output vector
                % size of the block "String to ASCII" in the subsystem needs to
                % be changed.
                set_param(strToASCIIblk,'OutputVectorSize', num2str(maxLength));
            end
        end

        function checkSetHeaderFieldName(obj, block)
        %checkSetHeaderFieldName The users changes the dropdown for
        %header field name in the mask

        % Check if the dropdown is set to 'Use the default Header field
        % name', then grey out the header field name box
            paramCheck = get_param(block,'SetHeaderFieldName');
            blockMask = Simulink.Mask.get(block);
            fillHeader = blockMask.getParameter('HeaderFieldName');

            if strcmp(paramCheck, obj.HeaderFieldNameByDefault)
                set_param(block,'HeaderFieldName','Header');
                fillHeader.Visible = 'off';
            else
                fillHeader.Visible = 'on';
            end
        end

        function setHeaderFieldName(~,block)
        %setHeaderFieldName The user specifies their own Header field
        %name for their ROS message

        % Execute Header field name setting only when the
        % 'HeaderfieldName' is visible
            blockMask = Simulink.Mask.get(block);
            fillHeader = blockMask.getParameter('HeaderFieldName');
            if(strcmp(fillHeader.Visible, 'on'))
                % Check if the added Header fieldname exists in the ROS
                % message, if not throw out an error
                % set_param(block,'MaskHideContents','off')
                blockFullName = [get_param(block, 'Parent'),'/',...
                                 get_param(block, 'Name')];

                % Get the updated elements of the bus selector and bus
                % assignment block
                paramCheck = get_param(block,'HeaderFieldName');
                busFrameId = strcat(paramCheck,'.FrameId');
                busCurlen= strcat(paramCheck,'.FrameId_SL_Info.CurrentLength');
                busStamp = strcat(paramCheck,'.Stamp');

                %Update the bus selector block's elements to be assigned for
                %frame Id
                newSignalsIDSelector = {busFrameId};
                set_param(strcat(blockFullName,'/IdSelector'),'OutputSignals',strjoin(newSignalsIDSelector,','));

                %Update the bus selector block's elements to be assigned for
                %time stamp
                newSignalsStampSelector = {busStamp};
                set_param(strcat(blockFullName,'/StampSelector'),'OutputSignals',strjoin(newSignalsStampSelector,','));

                %Update the bus assignment block's elements to be assigned for
                %frame Id, time stamp and frame Id current length
                newSignalsBusAssign = {busFrameId,busCurlen,busStamp};
                set_param(strcat(blockFullName,'/HeaderAssign'),'AssignedSignals',strjoin(newSignalsBusAssign,','));

                ros.slros.internal.block.HeaderAssignmentBlockMask.dispatch('checkHeaderFieldNameErr', blockFullName);
            end
        end

        function checkHeaderFieldNameErr(~,block)
        %checkHeaderFieldNameErr Checks if the added Header fieldname
        %exists in the ROS message or not
            blockFullName = [get_param(block, 'Parent'),'/',...
                             get_param(block, 'Name')];
            userHeaderFieldName = get_param(block,'HeaderFieldName');

            % Get the input signal to the bus assignment block of the
            % subsystem
            busInputSig = get_param(strcat(blockFullName,'/HeaderAssign'),'InputSignals');
            % Compare if the input signal to the bus and check if the user
            % defined header field name matches or not.

            if(~isempty(busInputSig))
                % If both, the input signal and the user defined header
                % field name matches, then it assigns a value 1 to that cell in
                % the whole cell array (cell array size are different)
                checkCorrectFieldName = cellfun(@(x) strcmp(x,userHeaderFieldName),...
                                                busInputSig, 'UniformOutput', false);

                % If the sum of the cell value is more than 1, that means that
                % the input signal has the user defined header field name
                value = sum(cellfun(@(x) sum(x),checkCorrectFieldName));
                assert(value > 0,message('ros:slros:blockmask:InvalidHdrFldError',...
                                         userHeaderFieldName))
            end
        end

        function checkVarLenArrayErr(~,block,maxLength)
        % Check if the length of the frameID is shorter than
        % VariableSize Message
            frameId = get_param(block,'FrameID');
            frameLen = length(frameId);
            if frameLen > maxLength
                sldiagviewer.reportError(message('ros:slros:blockmask:ShortFrameIdError',...
                                                 num2str(frameLen),num2str(maxLength)).getString);
            end
        end

    end

    methods (Static)
        function dispatch(methodName, varargin)
            obj = ros.slros.internal.block.HeaderAssignmentBlockMask();
            obj.(methodName)(varargin{:});
        end
    end

end
