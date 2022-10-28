classdef (Abstract) CommonParameterMask < ros.slros.internal.block.CommonMask
%This class is for internal use only. It may be removed in the future.

%CommonParameterMask Common base class for ROS parameter blocks
%   In particular, this includes the Get and Set Parameter blocks.

%   Copyright 2015-2018 The MathWorks, Inc.

%% Model-wide Callbacks
    methods (Access = protected)
        function selectParameterDialogCloseCallback(~, isAcceptedSelection, selectedParam, selectedParamType, block)
        %selectParameterDialogCloseCallback Callback when user closes the parameter selector dialog

            if isAcceptedSelection
                % The data types are already enforced in the
                % ParameterSelector dialog, so there is no need to
                % double-check here

                % Set the parameter name and parameter type
                set_param(block, 'ParameterName', selectedParam);
                set_param(block, 'ParameterType', selectedParamType);
            end
        end
    end

    %% Block mask parameter Callbacks
    methods (Access = protected)
        % Callbacks for individual params are invoked when the user:
        % * Opens a mask dialog
        % * Modifies a param value and changes focus
        % * Modifies a param value and selects OK or Apply
        % * Updates the model (presses Ctrl+D or simulates the model)
        %
        % Note - these are **not** invoked when user does a SET_PARAM

        function onSelectFromNetwork(obj, block)
        %onSelectFromNetwork Executed when the user clicks the "Select" button

        % Open selection dialog
            try
                msgDlg = ros.slros.internal.dlg.ParameterSelector();
                msgDlg.openDialog(@(x,y,z)obj.selectParameterDialogCloseCallback(x,y,z,block));
            catch ME
                % Send error to Simulink diagnostic viewer rather than a
                % DDG dialog.
                % NOTE: This does NOT stop execution.
                reportAsError(MSLDiagnostic(ME));
            end
        end

        function setParameterSource(obj, block)
        % Have to implement this, since promoted popup is always "Top"
        % aligned and I want "Left" alignment

        % Adjust the mask here accordingly
            maskValues = get_param(block, 'MaskValues');
            maskEnables = get_param(block,'MaskEnables');

            % Get dialog controls to modify the button visibility
            mask = Simulink.Mask.get(block);
            dlg = mask.getDialogControls;

            d = obj.MaskDlgIndex.ParamSelect;

            if strcmpi(maskValues{obj.MaskParamIndex.ParamSourceDropdown}, obj.TopicSourceSpecifyOwn)
                % Custom parameter name specification
                % Enable editing of name and type
                maskEnables{obj.MaskParamIndex.ParamNameEdit} = 'on';
                maskEnables{obj.MaskParamIndex.ParamDataTypeDropdown} = 'on';
                % Disable the parameter selection button
                dlg(d(1)).DialogControls(d(2)).DialogControls(d(3)).Enabled = 'off';
            else
                % Select parameter from ROS network
                % Disable editing of parameter name
                maskEnables{obj.MaskParamIndex.ParamNameEdit} = 'off';
                maskEnables{obj.MaskParamIndex.ParamDataTypeDropdown} = 'off';
                % Enable parameter selection button
                dlg(d(1)).DialogControls(d(2)).DialogControls(d(3)).Enabled = 'on';
            end

            % Enable and disable selected block mask parameters
            set_param(gcb,'MaskEnables', maskEnables);

            obj.updateSubsystem(block);
        end
    end

end
