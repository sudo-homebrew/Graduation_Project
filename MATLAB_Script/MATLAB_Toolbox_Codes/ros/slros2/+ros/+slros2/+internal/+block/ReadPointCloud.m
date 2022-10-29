classdef ReadPointCloud < ros.slros2.internal.block.ReadXBase ...
        & ros.slros2.internal.block.mixin.ConfigureUsingROS2
%This class is for internal use only. It may be removed in the future.

%ReadPointCloud Extracts XYZ and RGB Data From ROS 2 PointCloud

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    properties(Nontunable)
        %PreserveStructure ros:slros:blockmask:PreserveStructurePrompt
        PreserveStructure (1, 1) logical = false

        %ShowRGBOutput ros:slros:blockmask:ShowRGBOutputPrompt
        ShowRGBOutput (1, 1) logical = false;

        %ShowIntensityOutput ros:slros:blockmask:ShowIntensityOutputPrompt
        ShowIntensityOutput (1, 1) logical = false;

    % Public, non-tunable properties
    
        %PointCloudDataType Point Cloud Datatype
        PointCloudDataType = 'single';

        %MaximumPointCloudSize ros:slros:blockmask:MaximumPointCloudSizePrompt
        MaximumPointCloudSize = [480,640];
    end

    properties (Constant, Access = {...
                                     ?ros.slros.internal.block.ReadXBase, ...
                                     ?matlab.unittest.TestCase})

        %IconName - Name of block icon
        IconName = 'Read Point Cloud'

        %ValidMessageTypes - Message types that are valid inputs to block
        ValidMessageTypes = {'sensor_msgs/PointCloud2'}

        %InvalidTypeID - Error ID if an invalid message is used as input
        InvalidTypeID = 'ros:slros2:pointcloud:InvalidType'
    end

    properties (Nontunable, Access = protected, Dependent)
        IconInfo
        IconImage
        OutputConfig
    end

    properties (Access=private, Transient)
        % Converter - Handle to object that encapsulates converting a
        % Simulink bus struct to a MATLAB ROS 2 message.
        Converter = []

        %InputIsPointCloud2 - Logical scalar
        InputIsPointCloud2 = true
    end

    properties(Constant, Hidden)
        PointCloudDataTypeSet = matlab.system.StringSet({'single'});
        Reader = ros.msg.sensor_msgs.internal.PointCloud2Reader;
    end

    properties (Access = protected, Hidden)
        TableHeaders = ["Topic Name", "Message Type", "Point Cloud Size", "Status"];
    end

    properties(Constant, Access = {?matlab.unittest.TestCase})
        ErrorCodeSuccess                 = uint8(0)
        ErrorCodeExceedsMaxSize          = uint8(1)
        ErrorCodeArrayTruncated          = uint8(2)
        ErrorCodeMissingXYZ              = uint8(3)
        ErrorCodeMissingRGB              = uint8(4)
        ErrorCodeMissingIntensity        = uint8(5)
        ErrorCodeWrongDataTypeXYZ        = uint8(6)
        ErrorCodeWrongDataTypeRGB        = uint8(7)
        ErrorCodeWrongDataTypeIntensity  = uint8(8)
        OutputRGB = uint8(1)
        OutputIntensity = uint8(2)
        OutputErrorCode = uint8(4)
    end

    methods
        % Constructor
        function obj = ReadPointCloud(varargin)
        % Support name-value pair arguments when constructing object
            coder.allowpcode('plain');
            setProperties(obj,nargin,varargin{:});
        end

        function val = get.IconInfo(~)
            val = '';
        end

        function val = get.IconImage(~)
            val = 'ros2lib_readpointcloud.svg';
        end

        function val = get.OutputConfig(obj)
            val = uint8(0);
            if obj.ShowRGBOutput
                val = val + obj.OutputRGB;
            end
            if obj.ShowIntensityOutput
                val = val + obj.OutputIntensity;
            end
            if obj.ShowErrorCodeOutput
                val = val + obj.OutputErrorCode;
            end
        end

        function set.MaximumPointCloudSize(obj, val)
            robotics.internal.validation.validateNumericMatrix(val, ...
                                                               'ReadPointCloud', 'MaximumPointCloudSize', ...
                                                               'size', [1,2], 'integer');
            obj.MaximumPointCloudSize = val;
        end
    end

    methods(Access = protected)
        %% Common functions
        function setupImpl(obj, ~)
        % Perform one-time calculations, such as computing constants
            setupImpl@ros.slros2.internal.block.ReadXBase(obj);

            if coder.target('MATLAB')
                rosMessageType = 'sensor_msgs/PointCloud2';
                obj.Converter = ros.slros2.internal.sim.BusStructToROSMsgConverter(...
                    rosMessageType, bdroot);
            end
        end

        function varargout = stepImpl(obj,busstruct)
        %stepImpl Extract signals from point cloud message
            dataTruncated = (busstruct.data_SL_Info.CurrentLength < ...
                             busstruct.data_SL_Info.ReceivedLength);

            pc = ros.slros2.internal.block.PointCloud2BusWrapper(busstruct);
            pc.PreserveStructureOnRead = obj.PreserveStructure;
            xIdx = pc.getFieldIndex('x');
            yIdx = pc.getFieldIndex('y');
            zIdx = pc.getFieldIndex('z');
            rgbIdx = pc.getFieldIndex('rgb');
            intensityIdx = pc.getFieldIndex('intensity');
            pcHeight = pc.Height;
            pcWidth = pc.Width;

            hasField = struct( ...
                'x', xIdx ~= 0, 'y', yIdx ~= 0, 'z', zIdx ~= 0, ...
                'rgb', rgbIdx ~= 0, 'intensity', intensityIdx ~= 0);
            blockHeight = coder.const(obj.MaximumPointCloudSize(1));
            blockWidth = coder.const(obj.MaximumPointCloudSize(2));
            if obj.PreserveStructure
                % The only data that we will return is that contained
                % in the intersection of the block size and the incoming
                % point cloud size.
                minHeight = min(blockHeight, pcHeight);
                minWidth = min(blockWidth, pcWidth);
                minNumPoints = minWidth*minHeight;

                % These assertions make it clear to the code generation
                % process that expressions like "1:minHeight" have
                % upper-bounds on their size (since blockHeight and
                % blockWidth are compile-time constants).
                coder.internal.assert(minHeight <= blockHeight, ...
                                      'ros:slros:cgen:InferenceAssertion')
                coder.internal.assert(minWidth <= blockWidth, ...
                                      'ros:slros:cgen:InferenceAssertion')
                coder.internal.assert( ...
                    minNumPoints <= blockWidth*blockHeight, ...
                    'ros:slros:cgen:InferenceAssertion')

                % We want to retrieve a blockHeight x blockWidth rectangle
                % for each signal. These lines produce a blockHeight x
                % blockWidth matrix of indices (pointIndices).
                pointIndices = zeros(blockHeight, blockWidth);

                % Each element of pointIndices is the row-major linear
                % index corresponding to that point in the incoming
                % message. If a point lies outside of the area defined by
                % pcWidth and pcHeight, the corresponding entry of
                % pointIndices is zero.
                rowIdx = 1:minHeight;
                colIdx = 1:minWidth;
                idx1 = repmat(colIdx(:), [1, blockHeight]);
                idx2 = repmat(rowIdx, [blockWidth, 1]);
                pointIndices(rowIdx, colIdx) = reshape( ...
                    sub2ind([pcWidth, pcHeight], ...
                            reshape(idx1(1:minWidth,1:minHeight), 1, []), ...
                            reshape(idx2(1:minWidth,1:minHeight), 1, [])), ...
                    minWidth, minHeight).';
            else
                % We want to retrieve as many points as permitted by the
                % "Maximum point cloud size" parameter.
                pointIndices = 1:blockWidth*blockHeight;
            end

            coder.varsize('xyzTmp', 'rgbTmp', [blockWidth*blockHeight,3], [1,0]);
            coder.varsize('intensityTmp', [blockWidth*blockHeight,1], [1,0]);

            % Extract the data from the point cloud.
            % Each if-statement below produces a minWidth*minHeight x C
            % array of values, where C is the number of values per-point of
            % the signal in question (e.g. 3 for xyz). These are the values
            % at the points specified by pointIndices. If an element of
            % pointIndices is greater than the number of points in the
            % incoming point cloud, the corresponding row in *Tmp will be
            % NaNs.
            if hasField.x && hasField.y && hasField.z
                xyzTmp = obj.Reader.readXYZ(pc, xIdx, yIdx, zIdx, ...
                                            obj.PointCloudDataType, pointIndices);
            else
                xyzTmp = NaN([blockWidth*blockHeight,3], ...
                             obj.PointCloudDataType);
            end
            % We always return the RGB values as doubles
            if hasField.rgb
                rgbTmp = obj.Reader.readRGB(pc, rgbIdx, pointIndices);
            else
                rgbTmp = NaN([blockWidth*blockHeight,3], 'double');
            end
            if hasField.intensity
                if isempty(coder.target)
                    % In simulation the intensity field data-type must be
                    % dynamically derived from the point cloud to ascertain
                    % correct output signal dimensions
                    intensityTmp = cast(obj.Reader.readField(pc, intensityIdx, ...
                                                             [], 1, pointIndices), obj.PointCloudDataType);
                else
                    % In code-generation the intensity field is a pre-defined
                    % MATLAB Type, single.
                    intensityTmp = obj.Reader.readField(pc, intensityIdx, ...
                                                        obj.PointCloudDataType, 1, pointIndices);
                end
            else
                intensityTmp = NaN([blockWidth*blockHeight,1], ...
                                   obj.PointCloudDataType);
            end

            errorCode = obj.computeErrorCode(hasField, dataTruncated);

            % Reshape/trim the outputs based on the "Preserve point cloud
            % structure" and "Output variable-size signals" parameters.
            [xyz, rgb, intensity] = obj.resizeOutputs(xyzTmp, rgbTmp, intensityTmp, pcHeight, pcWidth);
            switch obj.OutputConfig
              case 0
                varargout = {xyz};
              case obj.OutputErrorCode
                varargout = {xyz, errorCode};
              case obj.OutputIntensity
                varargout = {xyz, intensity(:,:,1)};
              case obj.OutputIntensity + obj.OutputErrorCode
                varargout = {xyz, intensity(:,:,1), errorCode};
              case obj.OutputRGB
                varargout = {xyz, rgb};
              case obj.OutputRGB + obj.OutputErrorCode
                varargout = {xyz, rgb, errorCode};
              case obj.OutputRGB + obj.OutputIntensity
                varargout = {xyz, rgb, intensity(:,:,1)};
              case obj.OutputRGB + obj.OutputIntensity + obj.OutputErrorCode
                varargout = {xyz, rgb, intensity(:,:,1), errorCode};
            end
        end

        function [xyz, rgb, intensity] = resizeOutputs(obj, xyzIn, rgbIn, intensityIn, pcHeight, pcWidth)
            if obj.PreserveStructure
                xyz = obj.resizeStructuredOutput(xyzIn, pcHeight, pcWidth);
                rgb = obj.resizeStructuredOutput(rgbIn, pcHeight, pcWidth);
                intensity = obj.resizeStructuredOutput(intensityIn, pcHeight, pcWidth);
            else
                xyz = obj.resizeUnstructuredOutput(xyzIn, pcHeight*pcWidth);
                rgb = obj.resizeUnstructuredOutput(rgbIn, pcHeight*pcWidth);
                intensity = obj.resizeUnstructuredOutput(intensityIn, pcHeight*pcWidth);
            end
        end

        function out = resizeUnstructuredOutput(obj, in, inNumPoints)
        %resizeUnstructuredOutput Create appropriately sized outputs
        % blockNumPoints is the number of points specified by the
        % "Maximum point cloud size" parameter, while inNumPoints is
        % the number of points specified by the input message.
            blockNumPoints = prod(obj.MaximumPointCloudSize);
            minNumPoints = min(blockNumPoints, inNumPoints);
            if obj.VariableSizeOutputs
                % If VariableSizeOutputs == true, the output should have
                % min(blockNumPoints,inNumPoints) rows, all of which
                % contain data.
                out = in(1:minNumPoints, :);
            else
                % If VariableSizeOutputs == false, the output should have
                % fixed size (rows and columns) that matches the size set
                % by getOutputSizeImpl method of the system object. The raw
                % data is structured to fixed sizes, to avoid size inference
                % conflict from MATLAB Coder.
                count = coder.const(size(in, 2));
                dataSize = coder.const(prod(obj.MaximumPointCloudSize));
                out = reshape(in, [dataSize, count]);
            end
        end

        function out = resizeStructuredOutput(obj, in, pcHeight, pcWidth)
        % resizeStructuredOutput Create appropriately sized outputs
        % Get the number of values per point
            count = size(in, 2);
            inStructured = reshape(in, [obj.MaximumPointCloudSize, count]);
            if obj.VariableSizeOutputs
                out = inStructured(1:min(pcHeight, obj.MaximumPointCloudSize(1)), ...
                                   1:min(pcWidth, obj.MaximumPointCloudSize(2)), 1:count);
            else
                out = inStructured;
            end
        end

        function errorCode = computeErrorCode(obj, hasField, truncated)
            if ~(hasField.x && hasField.y && hasField.z)
                errorCode = obj.ErrorCodeMissingXYZ;
                return;
            end
            if obj.ShowRGBOutput && ~hasField.rgb
                errorCode = obj.ErrorCodeMissingRGB;
                return;
            end
            if obj.ShowIntensityOutput && ~hasField.intensity
                errorCode = obj.ErrorCodeMissingIntensity;
                return;
            end
            if truncated
                errorCode = obj.ErrorCodeArrayTruncated;
                return;
            end
            errorCode = obj.ErrorCodeSuccess;
        end

        function out = outputSize(obj, numValues)
        %outputSize Returns size of output signal
            if obj.PreserveStructure
                out = [obj.MaximumPointCloudSize, numValues];
            else
                out = [prod(obj.MaximumPointCloudSize), numValues];
            end
        end

        function out = varsizeDims(obj)
        %varsizeDims Returns array indicating if each dim is varsize
            if obj.PreserveStructure
                out = [1, 1, 0];
            else
                out = [1, 0];
            end
        end

        function varargout = getOutputSizeImpl(obj)
        %getOutputSizeImpl Return Size for each output port
            varargout = cell(1, nargout);
            [varargout{:}] = getOutputSizeImpl@ros.slros.internal.block.ReadXBase(obj);
            varargout{1} = obj.outputSize(3);
            k = 2;
            if obj.ShowRGBOutput
                varargout{k} = obj.outputSize(3);
                k = k + 1;
            end
            if obj.ShowIntensityOutput
                varargout{k} = obj.outputSize(1);
            end
        end

        function varargout = getOutputDataTypeImpl(obj)
        %getOutputDataTypeImpl Return data type for each output port
            varargout = cell(1, nargout);
            [varargout{:}] = getOutputDataTypeImpl@ros.slros.internal.block.ReadXBase(obj);
            varargout{1} = obj.PointCloudDataType;
            k = 2;
            if obj.ShowRGBOutput
                varargout{k} = 'double';
                k = k + 1;
            end
            if obj.ShowIntensityOutput
                varargout{k} = obj.PointCloudDataType;
            end
        end

        function varargout = isOutputComplexImpl(obj)
        %isOutputComplexImpl Return true for each output port with complex data
            varargout = cell(1, nargout);
            [varargout{:}] = isOutputComplexImpl@ros.slros.internal.block.ReadXBase(obj);
            varargout{1} = false; % XYZ
            k = 2;
            if obj.ShowRGBOutput
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
            if obj.ShowRGBOutput
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
            varargout{1} = 'XYZ';
            k = 2;
            if obj.ShowRGBOutput
                varargout{k} = 'RGB';
                k = k + 1;
            end
            if obj.ShowIntensityOutput
                varargout{k} = 'Intensity';
            end
        end

        function num = getNumOutputsImpl(obj)
        %getNumOutputsImpl Define number of outputs for system with optional outputs
            num = 1; % XYZ output
            if obj.ShowRGBOutput
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
                set_param(block,'MaximumPointCloudSize',char(row.PointCloudSize));
            end
        end
    end

    methods(Access = protected, Static)
        function header = getHeaderImpl
        % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename('class'), ...
                                                  'Title', slrosBlockmaskMessage('ReadPointCloudTitle'), ...
                                                  'Text', slrosBlockmaskMessage('ReadPointCloudDescription'),...
                                                  'ShowSourceLink', false);
        end

        function group = getPropertyGroupsImpl
        % Define property section(s) for System block dialog

            pointCloudGroup = ros.slros2.internal.block.mixin.ConfigureUsingROS2.getConfigurableSettingsGroup( ...
                slrosBlockmaskMessage('PointCloudPropertiesPrompt'), ...
                {'MaximumPointCloudSize'}, ...
                message('ros:slros2:topicselector:PointCloudDialogTitle').string);
            outputGroup = ros.slros.internal.block.ReadXBase.getOutputPropertyGroup( ...
                {'PreserveStructure', 'ShowRGBOutput','ShowIntensityOutput'});
            group = [pointCloudGroup, outputGroup];
        end

        function topicSelector = generateTopicTable(node, varargin)
            topicSelector = ros.slros2.internal.PointCloudTopicTable(node, varargin{:});
        end
    end
end

function str = slrosBlockmaskMessage(key, varargin)
    str = getString(message(['ros:slros2:blockmask:' key], varargin{:}));
end
