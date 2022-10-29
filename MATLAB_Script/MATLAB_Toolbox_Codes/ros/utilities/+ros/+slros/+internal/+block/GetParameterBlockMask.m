classdef GetParameterBlockMask < ros.slros.internal.block.CommonParameterMask
%This class is for internal use only. It may be removed in the future.

%GetParameterBlockMask Block mask callbacks for GetParameter block
%   Note that we are using a mask-on-mask on top of the GetParameter
%   system object. This has the advantage of easy customization, in
%   addition to the easy manipulation of the number of input and output
%   ports.

%   Copyright 2015-2020 The MathWorks, Inc.

    properties (Constant)
        %MaskType - Type of block mask
        %   Retrieve is with get_param(gcb, 'MaskType')
        MaskType = 'ROS Get Parameter'

        %MaskParamIndex - Struct specifying index of various parameters
        MaskParamIndex = struct( ...
            'ParamSourceDropdown', 1, ...
            'ParamNameEdit', 2, ...
            'ParamDataTypeDropdown', 3, ...
            'ParamMaxArrayLengthEdit', 4, ...
            'ParamInitialValueEdit', 5 ...
            );

        %MaskParamIndex - Struct specifying index of various widgets
        MaskDlgIndex = struct( ...
            'ParamSelect', [2 1 3] ...  % "Parameters" Container > "ROS Parameter" tab > "Select" Button
            )

        SysObjBlockName = '';
    end

    methods
        function updateSubsystem(~, block)
        %updateSubsystem Update the Block ID and Model Name
            blockId = ros.slros.internal.block.getCppIdentifierForBlock(block, 'ParamGet_');
            modelName = bdroot(block);

            set_param(block, 'ModelName', modelName);
            set_param(block, 'BlockId', blockId);
        end

        function maskInitialize(~, block)
            blkH = get_param(block, 'handle');
            ros.internal.setBlockIcon(blkH, 'rosicons.robotlib_getparam');
        end         
    end

    methods (Access = protected)
        function setParameterType(obj, block)
        %setParameterType Callback when the user selects a parameter data type

        % Adjust the mask here to show or hide the "Maximum length"
        % block mask parameter
            maskValues = get_param(block, 'MaskValues');
            maskVisibilities = get_param(block, 'MaskVisibilities');

            if ~ros.slros.internal.sim.DataTypes.isSimulinkDataTypeScalar(maskValues{obj.MaskParamIndex.ParamDataTypeDropdown})
                % Show maximum array length block mask parameter
                maskVisibilities{obj.MaskParamIndex.ParamMaxArrayLengthEdit} = 'on';
            else
                % Hide maximum array length block mask parameter
                maskVisibilities{obj.MaskParamIndex.ParamMaxArrayLengthEdit} = 'off';
            end

            % Set new block mask visibilities
            set_param(gcb, 'MaskVisibilities', maskVisibilities);

            obj.updateSubsystem(block);
        end

        function selectParameterDialogCloseCallback(obj, isAcceptedSelection, selectedParam, selectedParamType, block)
        %selectParameterDialogCloseCallback Callback when user closes the parameter selector dialog
        %   This function is overloaded from CommonParameterMask and
        %   executes some additional code.

        % Execute function on super class first
            selectParameterDialogCloseCallback@ros.slros.internal.block.CommonParameterMask( ...
                obj, isAcceptedSelection, selectedParam, selectedParamType, block);

            if isAcceptedSelection
                % Ensure that visibility of Maximum length field is handled
                % correctly.
                obj.setParameterType(block);
            end
        end
    end

    methods(Static)
        function dispatch(methodName, varargin)
        %dispatch Static dispatch method for callbacks

            obj = ros.slros.internal.block.GetParameterBlockMask();
            obj.(methodName)(varargin{:});
        end
    end
end
