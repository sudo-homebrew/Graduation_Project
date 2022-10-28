classdef ReadScan < ros.slros.internal.block.ReadXBase ...
        & ros.slros.internal.block.mixin.ConfigureUsingROS
% This class is for internal use only. It may be removed in the future.
%ReadScan Extract ranges, angles, and intensities from a ROS laser scan
% message.
%   See also readImage, rosReadScanAngles

%   Copyright 2021-2022 The MathWorks, Inc.

%#codegen

    properties(Nontunable)
        %ShowAnglesOutput ros:slros:blockmask:ShowAnglesOutputPrompt
        ShowAnglesOutput (1, 1) logical = true;
        %ShowIntensityOutput ros:slros:blockmask:ShowIntensityOutputPrompt
        ShowIntensityOutput (1, 1) logical = false;
    end

    % Public, non-tunable properties
    properties(Nontunable)
        %MaximumArrayLength ros:slros:blockmask:MaximumArrayLengthPrompt
        MaximumArrayLength = 100;
    end

    properties (Nontunable, Access = protected, Dependent)
        IconInfo
        IconImage
        OutputConfig
    end

    properties (Constant, Access = {?ros.slros.internal.block.ReadXBase, ...
                                    ?matlab.unittest.TestCase})
        %IconName - Name of block icon
        IconName = 'Read_Scan'
        %ValidMessageTypes - Message types that are valid inputs to block
        ValidMessageTypes = {'sensor_msgs/LaserScan'}
        %InvalidTypeID - Error ID if an invalid message is used as input
        InvalidTypeID = 'ros:slros:scan:InvalidType'
    end

    properties (Access = protected, Hidden)
        TableHeaders = ["Topic Name", "Message Type", "Scan Array Length", "Status"];
    end

    properties(Constant, Access = {?matlab.unittest.TestCase})
        ErrorCodeSuccess = uint8(0)
        ErrorCodeArrayTruncate = uint8(1)
        OutputAngles = uint8(1)
        OutputIntensity = uint8(2)
        OutputErrorCode = uint8(4)
    end

    methods
        % Constructor
        function obj = ReadScan(varargin)
        % Support name-value pair arguments when constructing object
            coder.allowpcode('plain');
            setProperties(obj,nargin,varargin{:});
        end

        function val = get.IconInfo(~)
            val = '';
        end

        function val = get.IconImage(~)
            val = 'robotlib_readscan.svg';
        end

        function val = get.OutputConfig(obj)
            val = uint8(0);
            if obj.ShowAnglesOutput
                val = val + obj.OutputAngles;
            end
            if obj.ShowIntensityOutput
                val = val + obj.OutputIntensity;
            end
            if obj.ShowErrorCodeOutput
                val = val + obj.OutputErrorCode;
            end
        end

        function set.MaximumArrayLength(obj,val)
            robotics.internal.validation.validateNumericMatrix(val, ...
                                                               'ReadScan','MaximumArrayLength','size',[1,1],'integer');
            obj.MaximumArrayLength = val;
        end
    end

    methods (Access = protected)
        %% Common functions
        function setupImpl(obj,~)
        % Perform one-time calculations, such as computing constants
            setupImpl@ros.slros.internal.block.ReadXBase(obj);
        end

        function varargout = stepImpl(obj,busstruct)
        %stepImpl Extract signals from laser scan message

        % Get current ranges and intensities length
            rangesLength = uint32(busstruct.Ranges_SL_Info.CurrentLength);
            intensitiesLength = uint32(busstruct.Intensities_SL_Info.CurrentLength);

            dataTruncated = (rangesLength < busstruct.Ranges_SL_Info.ReceivedLength) ...
                || (obj.ShowIntensityOutput && intensitiesLength < busstruct.Intensities_SL_Info.ReceivedLength);

            coder.varsize('rangesTmp','anglesTmp','intensitiesTmp',[obj.MaximumArrayLength,1],[1,0]);
            rangesTmp = zeros(obj.MaximumArrayLength,1,'single');
            anglesTmp = zeros(obj.MaximumArrayLength,1,'single');
            intensitiesTmp = zeros(obj.MaximumArrayLength,1,'single');

            if dataTruncated
                errorCode = obj.ErrorCodeArrayTruncate;
            else
                if ~obj.VariableSizeOutputs
                    rangesLength = uint32(obj.MaximumArrayLength);
                    intensitiesLength = uint32(obj.MaximumArrayLength);
                end
                errorCode = obj.ErrorCodeSuccess;

                rangesRange = min(rangesLength, busstruct.Ranges_SL_Info.CurrentLength);
                intensityRange = min(intensitiesLength, busstruct.Intensities_SL_Info.CurrentLength);

                rangesTmp(1:rangesRange) = busstruct.Ranges(1:rangesRange);
                anglesTmp(1:rangesRange) = single(ros.msg.sensor_msgs.internal.ScanAnglesReader.readScanAngles(...
                    busstruct.Ranges(1:rangesRange),busstruct.AngleMin, busstruct.AngleIncrement));
                intensitiesTmp(1:intensityRange) = busstruct.Intensities(1:intensityRange);
            end

            % Reshape/trim the outputs based on the "Output variable-size
            % signals" parameter
            [ranges, angles, intensities] = resizeOutputs(obj, rangesTmp, anglesTmp, intensitiesTmp, rangesLength, intensitiesLength);
            switch obj.OutputConfig
              case 0
                varargout = {ranges};
              case obj.OutputErrorCode
                varargout = {ranges, errorCode};
              case obj.OutputIntensity
                varargout = {ranges, intensities};
              case obj.OutputIntensity + obj.OutputErrorCode
                varargout = {ranges, intensities, errorCode};
              case obj.OutputAngles
                varargout = {ranges, angles};
              case obj.OutputAngles + obj.OutputErrorCode
                varargout = {ranges, angles, errorCode};
              case obj.OutputAngles + obj.OutputIntensity
                varargout = {ranges, angles, intensities};
              case obj.OutputAngles + obj.OutputIntensity + obj.OutputErrorCode
                varargout = {ranges, angles, intensities, errorCode};
            end
        end

        function [ranges, angles, intensities] = resizeOutputs(obj, rangesIn, anglesIn, intensityIn, rangesLength, intensitiesLength)
        %resizeOutputs Resize the output ranges, angles, and intensities
            if obj.VariableSizeOutputs
                minNumPoints = min(rangesLength, obj.MaximumArrayLength);
                minIntensity = min(intensitiesLength, obj.MaximumArrayLength);
                ranges = rangesIn(1:minNumPoints,1);
                angles = anglesIn(1:minNumPoints,1);
                intensities = intensityIn(1:minIntensity,1);
            else
                dataSize = coder.const(obj.MaximumArrayLength);
                ranges = reshape(rangesIn,[dataSize,1]);
                angles = reshape(anglesIn,[dataSize,1]);
                intensities = reshape(intensityIn,[dataSize,1]);
            end
        end

        function varargout = getOutputSizeImpl(obj)
        %getOutputSizeImpl Return size for each output port
            varargout = cell(1, nargout);
            [varargout{:}] = getOutputSizeImpl@ros.slros.internal.block.ReadXBase(obj);
            varargout{1} = [obj.MaximumArrayLength,1];
            k = 2;
            if obj.ShowAnglesOutput
                varargout{k} = [obj.MaximumArrayLength,1];
                k = k + 1;
            end
            if obj.ShowIntensityOutput
                varargout{k} = [obj.MaximumArrayLength,1];
            end
        end

        function varargout = getOutputDataTypeImpl(obj)
        %getOutputDataTypeImpl Return data type for each output port
            varargout = cell(1,nargout);
            [varargout{:}] = getOutputDataTypeImpl@ros.slros.internal.block.ReadXBase(obj);
            varargout{1} = 'single';
            k = 2;
            if obj.ShowAnglesOutput
                varargout{k} = 'single';
                k = k + 1;
            end
            if obj.ShowIntensityOutput
                varargout{k} = 'single';
            end
        end

        function varargout = isOutputComplexImpl(obj)
        %isOutputComplexImpl Return true for each output port with complex
        %data
            varargout = cell(1,nargout);
            [varargout{:}] = isOutputComplexImpl@ros.slros.internal.block.ReadXBase(obj);
            varargout{1} = false;
            k = 2;
            if obj.ShowAnglesOutput
                varargout{k} = false;
                k = k + 1;
            end
            if obj.ShowIntensityOutput
                varargout{k} = false;
            end
        end

        function varargout = isOutputFixedSizeImpl(obj)
        %isOutputFixedSizeImpl Return true for each output port with fixed size
            varargout = cell(1, nargout);
            [varargout{:}] = isOutputFixedSizeImpl@ros.slros.internal.block.ReadXBase(obj);
            varargout{1} = ~obj.VariableSizeOutputs;
            k = 2;
            if obj.ShowAnglesOutput
                varargout{k} = ~obj.VariableSizeOutputs;
                k = k + 1;
            end
            if obj.ShowIntensityOutput
                varargout{k} = ~obj.VariableSizeOutputs;
            end
        end

        %% Input Port and Output port Naming configuration
        function InName = getInputNamesImpl(~)
        %getInputNamesImpl Return input port names for System block
            InName = 'Msg';
        end

        function varargout = getOutputNamesImpl(obj)
        %getOutputNamesImpl Return output port names for System block
            varargout = cell(1, nargout);
            [varargout{:}] = getOutputNamesImpl@ros.slros.internal.block.ReadXBase(obj);
            varargout{1} = 'Ranges';
            k = 2;
            if obj.ShowAnglesOutput
                varargout{k} = 'Angles';
                k = k + 1;
            end
            if obj.ShowIntensityOutput
                varargout{k} = 'Intensity';
            end
        end

        function num = getNumOutputsImpl(obj)
        %getNumOutputsImpl Define number of outputs for system with optional outputs
            num = 1; % Ranges output
            if obj.ShowAnglesOutput
                num = num + 1;
            end
            if obj.ShowIntensityOutput
                num = num + 1;
            end
            num = num + getNumOutputsImpl@ros.slros.internal.block.ReadXBase(obj);
        end

        %% Other Methods
        function dialogCloseCallbackImpl(~, block, isAcceptedSelection, row)
        %dialogCloseCallback Callback for "Configure..." dialog
            if isAcceptedSelection
                set_param(block,'MaximumArrayLength',char(row.ScanArrayLength));
            end
        end
    end

    methods(Access = protected, Static)
        function header = getHeaderImpl
        % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename('class'), ...
                                                  'Title', slrosBlockmaskMessage('ReadScanTitle'), ...
                                                  'Text', slrosBlockmaskMessage('ReadScanDescription'),...
                                                  'ShowSourceLink', false);
        end

        function group = getPropertyGroupsImpl
        % Define property section(s) for System block dialog
            scanGroup = ros.slros.internal.block.mixin.ConfigureUsingROS.getConfigurableSettingsGroup( ...
                slrosBlockmaskMessage('ScanPropertiesPrompt'), ...
                {'MaximumArrayLength'}, ...
                message('ros:slros:topicselector:ScanDialogTitle').string);
            outputGroup = ros.slros.internal.block.ReadXBase.getOutputPropertyGroup(...
                {'ShowAnglesOutput','ShowIntensityOutput'});
            group = [scanGroup, outputGroup];
        end

        function topicSelector = generateTopicTable(node, varargin)
            topicSelector = ros.slroscpp.internal.ScanTopicTable(node, varargin{:});
        end
    end
end

function str = slrosBlockmaskMessage(key, varargin)
    str = getString(message(['ros:slros:blockmask:' key], varargin{:}));
end
