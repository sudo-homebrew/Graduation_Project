classdef ReadImage <  ros.slros.internal.block.ReadXBase ...
        & ros.slros.internal.block.mixin.ConfigureUsingROS
    %This class is for internal use only. It may be removed in the future.

    %ReadImage Extract image signal from ROS image message.
    %   Supported Formats: ['rgb8','rgb16','rgba8','rgba16',     ...     %rgbX
    %                       'bgr8','bgr16','bgra8','bgra16',     ...     %bgrX
    %                       'mono8','mono16',                    ...     %monoX
    %                       '8uc1', '8uc2', '8uc3', '8uc4',      ...     %8ucX
    %                       '8sc1', '8sc2', '8sc3', '8sc4',      ...     %8scX
    %                       '16uc1', '16uc2', '16uc3', '16uc4',  ...     %16ucX
    %                       '16sc1', '16sc2', '16sc3', '16sc4',  ...     %16scX
    %                       '32sc1', '32sc2', '32sc3', '32sc4',  ...     %32scX
    %                       '32fc1', '32fc2', '32fc3', '32fc4',  ...     %32fcX
    %                       '64fc1', '64fc2', '64fc3', '64fc4']  ...     %64fcX
    %
    %   See also readImage, writeImage

    %   Copyright 2017-2019 The MathWorks, Inc.

    %#codegen


    properties(Nontunable)
        %ShowAlphaOutput ros:slros:blockmask:ShowAlphaOutputPortPrompt
        ShowAlphaOutput (1, 1) logical = false;

    % Public, non-tunable properties

        %MaximumImageSize ros:slros:blockmask:MaximumImageSizePrompt
        MaximumImageSize = [2000 2000];

    end

    properties (Nontunable, Dependent)
        %ImageEncoding ros:slros:blockmask:ImageEncodingPrompt
        ImageEncoding
    end

    properties (Dependent, Access = private, Nontunable)
        %DataType Data type of image signal
        DataType

        %NumChannels Number of channels in image signal
        NumChannels

        %HasAlpha True if the specified encoding has an alpha channel
        HasAlpha
    end

    properties(Nontunable, Transient, Access=protected)
        %OutputConfig
        OutputConfig
    end

    properties(DiscreteState)
        %Image
        Image

        %ImageSize
        ImageSize
    end

    properties (Constant, Access = {...
        ?ros.slros.internal.block.ReadXBase, ...
        ?matlab.unittest.TestCase})

        %IconName - Name of block icon
        IconName = 'Read Image'

        %ValidMessageTypes - Message types that are valid inputs to block
        ValidMessageTypes = {'sensor_msgs/Image', 'sensor_msgs/CompressedImage'}

        %InvalidTypeID - Error ID if an invalid message is used as input
        InvalidTypeID = 'ros:slros:image:InvalidType'
    end

    properties (Nontunable, Access = private)
        %ImageEncodingInternal
        ImageEncodingInternal = ros.msg.sensor_msgs.internal.ImageEncoding.info('rgb8');
    end

    properties (Access = protected, Dependent)
        IconInfo
        IconImage
    end

    % Declare the error codes as constants here, so that they can be easily
    % shared between simulation and the generated code.
    % The values are passed to the "initialize" function of the C++ object.
    properties (Constant, Access = {?matlab.unittest.TestCase, ?ros.slros.internal.block.ReadImage})
        %ErrorCodeSuccess - Successfully read the image message
        ErrorCodeSuccess = uint8(0)

        %ErrorCodeEncodingMismatch - The image message has a different encoding
        %   The encoding of the image message is compared to the encoding
        %   specified by the user in the block mask.
        ErrorCodeEncodingMismatch = uint8(1)

        %ErrorCodeExceedsMaxSize - The image message exceeds the size limit
        %   The width and height fields of the message are compared to
        %   obj.MaximumImageSize
        ErrorCodeExceedsMaxSize = uint8(2)

        %ErrorCodeArrayTruncate - Error code if the "data" field was truncated
        ErrorCodeArrayTruncate = uint8(3)

        %ErrorCodeDecompression - Error code if something goes wrong while
        %decompressing the image
        ErrorCodeDecompression = uint8(4)

        %OutputImageOnly
        OutputImageOnly = uint8(0)

        %OutputImageAndErrorCode
        OutputImageAndErrorCode = uint8(1)

        %OutputImageAndAlpha
        OutputImageAndAlpha = uint8(2)

        %OutputAll
        OutputAll = uint8(3)
    end

    properties(Constant, Hidden)
        ImageEncodingSet = matlab.system.StringSet( ...
            ros.msg.sensor_msgs.internal.ImageEncoding.NonBayerEncodings);
    end

    properties (Access = protected, Hidden)
        TableHeaders = ["Topic Name", "Message Type", "Image Encoding", "Maximum Image Size", "Status"];
    end


    methods
        %% Constructor of the System Object for transform Utility
        function obj = ReadImage(varargin)
        %Extract image signal from ROS image message.
            setProperties(obj,nargin,varargin{:});
        end

        function set.ImageEncoding(obj, val)
            obj.ImageEncodingInternal = ...
                ros.msg.sensor_msgs.internal.ImageEncoding.info(coder.const(val));
        end

        function val = get.ImageEncoding(obj)
            val = obj.ImageEncodingInternal.Name;
        end

        function val = get.DataType(obj)
            val = coder.const(obj.ImageEncodingInternal.DataType);
        end

        function val = get.NumChannels(obj)
            val = obj.ImageEncodingInternal.NumChannels;
        end

        function val = get.HasAlpha(obj)
            val = obj.ImageEncodingInternal.HasAlpha;
        end

        function val = get.OutputConfig(obj)
            if obj.ShowAlphaOutput && obj.HasAlpha
                if obj.ShowErrorCodeOutput
                    val = obj.OutputAll;
                else
                    val = obj.OutputImageAndAlpha;
                end
            else
                if obj.ShowErrorCodeOutput
                    val = obj.OutputImageAndErrorCode;
                else
                    val = obj.OutputImageOnly;
                end
            end
        end

        function val = get.IconInfo(obj)
            val = sprintf('[%d, %d] : %s', ...
                          obj.MaximumImageSize, ...
                          obj.ImageEncoding);
        end
        
        function val = get.IconImage(~)
            val = 'robotlib_readimage.svg';
        end
        
    end

    methods (Access = protected)
        %% Common functions
        function varargout = stepImpl(obj,busstruct)
        %stepImpl Convert the busstruct to an array signal

        % Convert the busstruct and update state if successful
            errorCode = obj.updateBusAndState(busstruct);

            % Set outputs based on current state
            [image, alpha] = obj.outputs();

            % Marshall the requested outputs
            switch obj.OutputConfig
              case obj.OutputImageOnly
                varargout = {image};
              case obj.OutputImageAndAlpha
                varargout = {image, alpha};
              case obj.OutputImageAndErrorCode
                varargout = {image, errorCode};
              case obj.OutputAll
                varargout = {image, alpha, errorCode};
            end
        end

        function errorCode = updateBusAndState(obj, busstruct)
        %update Convert the busstruct and update state if successful

        % Do not attempt conversion if the Data field is truncated
            if busstruct.Data_SL_Info.CurrentLength < busstruct.Data_SL_Info.ReceivedLength
                errorCode = obj.ErrorCodeArrayTruncate;
                return;
            end

            % Call appropriate conversion method for message type
            if isfield(busstruct,'Format') % sensor_msgs/CompressedImage
                                           % Do not attempt conversion if incoming encoding differs
                                           % from stored encoding
                format = char(busstruct.Format(1:busstruct.Format_SL_Info.CurrentLength)');
                if ~strncmpi(format, [obj.ImageEncoding, ';'], numel(obj.ImageEncoding)+1)
                    errorCode = obj.ErrorCodeEncodingMismatch;
                    return;
                end
                [rawImage, errorCode] = obj.decompressImage(busstruct);
            else % sensor_msgs/Image
                 % Do not attempt conversion if incoming encoding differs
                 % from stored encoding
                enc = char(busstruct.Encoding(1:busstruct.Encoding_SL_Info.CurrentLength)');
                if ~strcmpi(enc,obj.ImageEncoding)
                    errorCode = obj.ErrorCodeEncodingMismatch;
                    return;
                end
                data = busstruct.Data(1:busstruct.Data_SL_Info.CurrentLength,1);
                [rawImage, errorCode] = obj.decodeImage(data,busstruct.Width,busstruct.Height);
            end

            % Return without updating if the conversion was not successful
            if errorCode ~= 0
                return;
            end

            % Update state
            [sz, dt] = obj.getImageSpecification;
            obj.Image = zeros(sz, dt);
            [height, width, ~] = size(rawImage);
            obj.Image(1:height, 1:width, :) = rawImage;
            % The "0*obj.ImageSize" below keeps Simulink from
            % suppressing ImageSize when the block is using
            % fixed-size outputs
            obj.ImageSize = 0*obj.ImageSize + cast([height, width], 'like', obj.ImageSize);
            errorCode = obj.ErrorCodeSuccess;
        end

        function [image, alpha] = outputs(obj)
        %output Return image and alpha values based on current state
            if obj.HasAlpha
                fullImage = obj.Image(:,:,1:obj.NumChannels-1);
                fullAlpha = obj.Image(:, :, obj.NumChannels);
            else
                fullImage = obj.Image(:,:,1:obj.NumChannels);
                fullAlpha = zeros([obj.MaximumImageSize,0], 'like', obj.Image);
            end
            if obj.VariableSizeOutputs
                [image, alpha] = obj.createVarsizeOutputs(fullImage, fullAlpha);
            else
                image = fullImage;
                alpha = fullAlpha;
            end
        end

        function [image, alpha] = createVarsizeOutputs(obj, fullImage, fullAlpha)
        %createVarsizeOutputs Trim outputs to size of incoming data
            if obj.HasAlpha
                coder.varsize('image', [obj.MaximumImageSize, obj.NumChannels-1], [1, 1, 0]);
                coder.varsize('alpha', [obj.MaximumImageSize, 1], [1, 1, 0]);
                image = fullImage(1:obj.ImageSize(1), 1:obj.ImageSize(2), :);
                alpha = fullAlpha(1:obj.ImageSize(1), 1:obj.ImageSize(2));
            else
                coder.varsize('image', [obj.MaximumImageSize, obj.NumChannels], [1, 1, 0]);
                image = fullImage(1:obj.ImageSize(1), 1:obj.ImageSize(2), :);
                alpha = fullAlpha;
            end
        end

        function [image, errorCode] = decodeImage(obj, data, width, height)
        %decodeImage Convert raw uint8 data to array
            if (height > obj.MaximumImageSize(1)) || (width > obj.MaximumImageSize(2))
                errorCode = obj.ErrorCodeExceedsMaxSize;
                image = obj.Image;
                return;
            end
            if obj.HasAlpha
                [tempImage, alpha] = ...
                    ros.msg.sensor_msgs.internal.ImageReader.readImage( ...
                        data, width, height, obj.ImageEncodingInternal);
                image = cat(3,tempImage,alpha);
            else
                image = ...
                    ros.msg.sensor_msgs.internal.ImageReader.readImage( ...
                        data, width, height, obj.ImageEncodingInternal);
            end
            errorCode = obj.ErrorCodeSuccess;
        end

        function [image, errorCode] = decompressImage(obj, busstruct)
        %decompressImage Convert compressed image to array
            height = int32(obj.MaximumImageSize(1));
            width = int32(obj.MaximumImageSize(2));
            if coder.target('MATLAB')
                try
                    data = busstruct.Data(1:busstruct.Data_SL_Info.CurrentLength,1);
                    format = char(busstruct.Format(1:busstruct.Format_SL_Info.CurrentLength)');
                    msg = rosmessage('sensor_msgs/CompressedImage');
                    msg.Data = data;
                    msg.Format = format;
                    imageRaw = msg.readImage;
                    [height, width, ~] = size(imageRaw);
                    if obj.HasAlpha
                        % Padding with zeros for the alpha channel, as readImage
                        % does not return alpha data for CompressedImage
                        % messages
                        image = cat(3,imageRaw,zeros(height, width, obj.DataType));
                    else
                        image = imageRaw;
                    end
                    errorCode = obj.ErrorCodeSuccess;
                catch
                    image = zeros([obj.MaximumImageSize, obj.NumChannels],obj.DataType);
                    height = obj.MaximumImageSize(1);
                    width = obj.MaximumImageSize(2);
                    errorCode = obj.ErrorCodeDecompression;
                end
            elseif coder.target('Rtw')
                % Codegeneration requires the value to be specified for all
                % conditions
                fullImage = zeros([obj.MaximumImageSize, obj.NumChannels],obj.DataType);
                width = int32(obj.MaximumImageSize(2));
                height = int32(obj.MaximumImageSize(1));

                % Add code for compressed images on ROS
                decompressImageMethod = coder.const(sprintf('decompressImage<%d>', ...
                                                            obj.NumChannels));
                coder.ceval(decompressImageMethod, ...
                            coder.rref(busstruct.Data), ...
                            busstruct.Data_SL_Info.CurrentLength, ...
                            coder.ref(fullImage), ...
                            coder.ref(width), coder.ref(height));

                % The C++ call above updates WIDTH and HEIGHT to contain
                % the dimensions of the incoming image.
                image = fullImage(1:height, 1:width, :);
                errorCode = obj.ErrorCodeSuccess;
            end
            if (errorCode == 0) && any([height, width] > obj.MaximumImageSize)
                errorCode = obj.ErrorCodeExceedsMaxSize;
            end
        end

        function validateInputsImpl(obj,~)
            validateInputsImpl@ros.slros.internal.block.ReadXBase(obj);
            validateMaximumSize(obj.MaximumImageSize,'Maximum Size');
        end

        %% Output configuration
        function varargout = getOutputSizeImpl(obj)
        %getOutputSizeImpl Return Size for each output port
            varargout = cell(1, nargout);
            [varargout{:}] = getOutputSizeImpl@ros.slros.internal.block.ReadXBase(obj);
            if obj.HasAlpha
                varargout{1} = [obj.MaximumImageSize, obj.NumChannels-1]; % Image
                if obj.ShowAlphaOutput && obj.HasAlpha
                    varargout{2} = obj.MaximumImageSize;
                end
            else
                varargout{1} = [obj.MaximumImageSize, obj.NumChannels]; % Image
            end
        end

        function varargout = getOutputDataTypeImpl(obj)
        %getOutputDataTypeImpl Return data type for each output port
            varargout = cell(1, nargout);
            [varargout{:}] = getOutputDataTypeImpl@ros.slros.internal.block.ReadXBase(obj);
            imageDataType = obj.DataType;
            varargout{1} = imageDataType; % Image
            if obj.ShowAlphaOutput && obj.HasAlpha
                varargout{2} = imageDataType;
            end
        end

        function varargout = isOutputComplexImpl(obj)
        %isOutputComplexImpl Return true for each output port with complex data
            varargout = cell(1, nargout);
            [varargout{:}] = isOutputComplexImpl@ros.slros.internal.block.ReadXBase(obj);
            varargout{1} = false; % Image
            if obj.ShowAlphaOutput && obj.HasAlpha
                varargout{2} = false;
            end
        end

        function varargout = isOutputFixedSizeImpl(obj)
        %isOutputFixedSizeImpl Return true for each output port with fixed size
            varargout = cell(1, nargout);
            [varargout{:}] = isOutputFixedSizeImpl@ros.slros.internal.block.ReadXBase(obj);
            varargout{1} = ~obj.VariableSizeOutputs;
            if obj.ShowAlphaOutput && obj.HasAlpha
                varargout{2} = ~obj.VariableSizeOutputs;
            end
        end

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
        % Return size, data type, and complexity of discrete-state
        % specified in name
            switch name
              case 'Image'
                [sz, dt, cp] = getImageSpecification(obj);
              case 'ImageSize'
                sz = [1,2];
                dt = 'uint32';
                cp = false;
            end
        end

        function [sz, dt, cp] = getImageSpecification(obj)
            sz = [obj.MaximumImageSize,obj.NumChannels];
            dt = obj.DataType;
            cp = false;
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
            varargout{1} = 'Image';
            if obj.ShowAlphaOutput && obj.HasAlpha
                varargout{2} = 'Alpha';
            end
        end

        function num = getNumOutputsImpl(obj)
        %getNumOutputsImpl Define number of outputs for system with optional outputs
            num = 1; % Image output
            if obj.ShowAlphaOutput && obj.HasAlpha
                num = num + 1;
            end
            num = num + getNumOutputsImpl@ros.slros.internal.block.ReadXBase(obj);
        end

        function loadObjectImpl(obj,s,wasLocked)
        % Set properties in object obj to values in structure s

        % Set private and protected properties
            obj.ImageEncodingInternal = s.ImageEncodingInternal;

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        function s = saveObjectImpl(obj)
        % Set properties in structure s to values in object obj

        % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            s.ImageEncodingInternal = obj.ImageEncodingInternal;
        end

        function resetImpl(obj)
        %resetImpl Reset internal states
            [sz, dt] = obj.getImageSpecification;
            obj.Image = zeros(sz, dt);
            obj.ImageSize = uint32(obj.MaximumImageSize);
        end

        function flag = isInactivePropertyImpl(obj,prop)
        % Return false if property is visible based on object
        % configuration, for the command line and System block dialog
            flag = false;
            if strcmp(prop, 'ShowAlphaOutput') && ~obj.HasAlpha
                flag = true;
            end
        end

        %% Other Methods
        function dialogCloseCallbackImpl(~, block, isAcceptedSelection, row)
        %dialogCloseCallback Callback when user closes the Image selector dialog
            if isAcceptedSelection
                set_param(block,'ImageEncoding',char(row.ImageEncoding));
                set_param(block,'MaximumImageSize',char(row.MaximumImageSize));
            end
        end
    end

    methods(Access = protected, Static)
        function header = getHeaderImpl
        % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename('class'), ...
                                                  'Title', slrosBlockmaskMessage('ReadImageTitle'), ...
                                                  'Text', slrosBlockmaskMessage('ReadImageDescription'),...
                                                  'ShowSourceLink', false);
        end

        function group = getPropertyGroupsImpl
        % Define property section(s) for System block dialog
            imageGroup = ros.slros.internal.block.mixin.ConfigureUsingROS.getConfigurableSettingsGroup( ...
                slrosBlockmaskMessage('ImagePropertiesPrompt'), ...
                {'MaximumImageSize', 'ImageEncoding'}, ...
                message('ros:slros:topicselector:ImageDialogTitle').string, ...
                'DependOnPrivatePropertyList', {'ImageEncoding'});
            outputGroup = ros.slros.internal.block.ReadXBase.getOutputPropertyGroup( ...
                'ShowAlphaOutput');
            group = [imageGroup, outputGroup];
        end

        function topicSelector = generateTopicTable(node, varargin)
            topicSelector = ros.slroscpp.internal.ImageTopicTable(node, varargin{:});
        end
    end
end
function validateMaximumSize(val,name)
    validateattributes(val, {'single','double'}, {'nonempty','real','2d','ncols', 2,'nrows',1},'Read Image',name);
end

function str = slrosBlockmaskMessage(key, varargin)
    str = getString(message(['ros:slros:blockmask:' key], varargin{:}));
end
