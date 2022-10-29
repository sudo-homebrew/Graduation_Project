classdef GetParameter < ...
        ros.slros.internal.block.ParameterBase
    %This class is for internal use only. It may be removed in the future.

    %Get the value for a named ROS parameter
    %
    %   H = ros.slros.internal.block.GetParameter creates a system
    %   object, H, that gets the value for a named ROS parameter. The list of
    %   all ROS parameters is maintained by the ROS parameter server.
    %
    %   This system object is intended for use with the MATLAB System
    %   block. In order to access the ROS functionality from MATLAB, see
    %   ROSPARAM.
    %
    %   See also ros.slros.internal.block.SetParameter

    %   Copyright 2015-2020 The MathWorks, Inc.

    %#codegen

    properties (Nontunable)
        %ParameterMaxArrayLength - Maximum length of received array
        %   This parameter only applies if the user specifies an array data type.
        %   The Value output of the Get Parameter block will always be a
        %   fixed-size array with this size. Any array that has more
        %   elements than this parameter will be truncated.
        %   Default: 16
        ParameterMaxArrayLength = uint32(16)

        %ParameterInitialValue - Initial value
        %   The initial value is used as output if the parameter with the
        %   given name does not exist on model launch, or if the parameter
        %   data type does not match.
        %   If error conditions occur during runtime, the last received
        %   value is used instead of this initial value.
        %   Default: 0.0
        ParameterInitialValue = 0.0

        %SampleTime - Sample time for source block
        %   Default: -1 (inherited)
        SampleTime = -1

        %ShowErrorCodeOutput - Show ErrorCode output
        %   If this property is true, the ErrorCode output port is shown on
        %   the block. If the property is false, then the output port is
        %   hidden.
        %   Default: true
        ShowErrorCodeOutput (1, 1) logical = true
    end

    properties (Constant, Access=?ros.slros.internal.block.mixin.NodeDependent)
        %MessageCatalogName - Name of this block used in message catalog
        %   This property is used by the NodeDependent base class to
        %   customize error messages with the block name.
        
        %   Due a limitation in Embedded MATLAB code-generation with UTF-8 characters,
        %   use English text instead of message("ros:slros:blockmask:GetParameterTitle").getString
        MessageCatalogName = 'ROS Get Parameter'
    end

    properties (Access = protected)
        %StateHandler - Object handling the state of the parameter value
        %   The handler uses GetParameterScalarState for scalar parameters
        %   and GetParameterArrayState for array parameters.
        StateHandler

        %SampleTimeHandler - Object for validating sample time settings
        SampleTimeHandler
    end

    % Declare the error codes as constants here, so that they can be easily
    % shared between simulation and the generated code.
    % The values are passed to the "initialize" function of the C++ object.
    properties (Constant, Access = {?matlab.unittest.TestCase, ?ros.slros.internal.block.GetParameterStateInterface, ?ros.slros.internal.block.GetParameter})
        %ErrorCodeSuccess - Error code emitted when the parameter was successfully retrieved
        ErrorCodeSuccess = uint8(0)

        %ErrorCodeNoParam - Error code if the parameter does not exist
        ErrorCodeNoParam = uint8(1)

        %ErrorCodeTypeMismatch - Error code if the parameter exists, but has a different data type
        %   The received data type is compared to the data type specified
        %   by the user in the block mask.
        ErrorCodeTypeMismatch = uint8(2)

        %ErrorCodeArrayTruncate - Error code if a received array was truncated
        %   This can happen if the array length is greater than the maximum
        %   array length specified in ParameterMaxArrayLength.
        ErrorCodeArrayTruncate = uint8(3)
    end

    properties (Access = private, Transient)
        %LastValidValue - Last valid value of the ROS parameter that was received
        %   The data type of this value is determined by the ParameterType
        %   property.
        LastValidValue = []
    end

    methods
        function obj = GetParameter(varargin)
        %GetParameter Standard constructor

        % Enable code to be generated even if this file is p-coded
            coder.allowpcode('plain');

            % Support name-value pair arguments when constructing the object.
            setProperties(obj, nargin, varargin{:});

            % Initialize sample time validation object
            obj.SampleTimeHandler = robotics.slcore.internal.block.SampleTimeImpl;
        end

        function set.ParameterMaxArrayLength(obj, val)
        %set.ParameterMaxArrayLength Set the maximum permissible array size
        %   The input can be any numeric data type, as long as it's an
        %   integer and within the limits of the uint32 data type. The
        %   maximum array length has to be positive.

            validateattributes(val, {'numeric'}, {'real', 'positive', 'nonempty', 'scalar', 'nonnan', 'integer',...
                                '<=', intmax('uint32')}, ...
                               '', 'ParameterMaxArrayLength');
            obj.ParameterMaxArrayLength = uint32(val);
        end

        function set.ParameterInitialValue(obj, val)
        %set.ParameterInitialValue Set the initial value
        %   Validate the user input based on the data type of the ROS
        %   parameter.

            obj.ParameterInitialValue = obj.StateHandler.validateInitialValue(val, obj.ParameterTypeML); %#ok<MCSUP>

        end

        function set.SampleTime(obj, sampleTime)
        %set.SampleTime Validate sample time specified by user
            obj.SampleTime = obj.SampleTimeHandler.validate(sampleTime); %#ok<MCSUP>
        end
    end

    methods (Access = protected)
        function num = getNumInputsImpl(~)
        %getNumInputsImpl Get number of inputs
            num = 0;
        end

        function num = getNumOutputsImpl(obj)
        %getNumOutputsImpl Get number of outputs
        %   The number of outputs is configurable based on the checkbox
        %   value of "Show ErrorCode output".

        % The value output is always present
        % Potentially add Length output if we have an array
            num = obj.StateHandler.getNumOutputsImpl(obj);
        end

        function varargout = getOutputSizeImpl(obj)
        %getOutputSizeImpl Get output size
        %   The output value could be an array, if it is a string.

            valueSize = obj.StateHandler.getValueOutputSizeImpl(obj);

            % The other output parameters (array length, error code) will
            % always be scalar
            varargout = {valueSize, [1 1], [1 1]};
        end

        function varargout = isOutputFixedSizeImpl(~)
        %isOutputFixedSizeImpl Are outputs fixed-size
        %   This is true for all three outputs (Value, ArrayLength,
        %   and ErrorCode)
            varargout =  {true, true, true};
        end

        function varargout = getOutputDataTypeImpl(obj)
        %getOutputDataTypeImpl Get data type of outputs
        %   The data type of the value is determined by user. The error
        %   code is always a uint8. The length of the receive

            varargout = obj.StateHandler.getOutputDataTypeImpl(obj, obj.ParameterTypeML);
        end

        function varargout = isOutputComplexImpl(~)
        %isOutputComplexImpl Are outputs complex-valued
            varargout = {false, false, false};
        end
    end


    methods (Access = protected)
        %% Common System Object functions
        %%
        function setupImpl(obj)
        %setupImpl Model initialization call
        %   setupImpl is called when model is being initialized at the
        %   start of a simulation.

            setupImpl@ros.slros.internal.block.ParameterBase(obj);

            if coder.target('Rtw')
                % 'Rtw' is executed during model build

                % Include the header file with the C++ declarations
                coder.cinclude(obj.HeaderFile);
                % Append \0 to obj.ParameterName, since MATLAB doesn't
                % automatically zero-terminate strings in generated code
                zeroDelimName = [obj.ParameterName 0];
                coder.ceval([obj.BlockId '.initialize'], zeroDelimName);
                coder.ceval([obj.BlockId '.initialize_error_codes'], obj.ErrorCodeSuccess, ...
                            obj.ErrorCodeNoParam, obj.ErrorCodeTypeMismatch, ...
                            obj.ErrorCodeArrayTruncate);

                obj.StateHandler.setupSetInitialValue(obj);
            end
        end

        function varargout = getOutputNamesImpl(obj)
        %getOutputNamesImpl Get names of output ports

            varargout = obj.StateHandler.getOutputNamesImpl(obj);
        end


        %%
        function varargout = stepImpl(obj)
        %stepImpl System Object step call
        %   VALUE is the returned parameter value and ERRORCODE
        %   indicates if the retrieval of the parameter was successful.
        %   ERRORCODE has the following possible values:
        %      0 = successfully retrieved parameter value and returned
        %          in VALUE
        %      1 = The parameter with ParameterName does not exist on
        %          the parameter server. If this error occurs on model
        %          launch, VALUE will contain ParameterInitialValue. If this
        %          happens during runtime, VALUE will contain the last
        %          received valid parameter value.
        %      2 = The parameter with ParameterName exists, but has a
        %          data type that is different from the one specified
        %          in Simulink. If this error occurs on model
        %          launch, VALUE will contain ParameterInitialValue. If this
        %          happens during runtime, VALUE will contain the last
        %          received valid parameter value.
        %      3 = The array parameter was retrieved, but the array
        %          length was longer than the user-specified maximum
        %          length. The output array was truncated to this
        %          maximum length.

        % By default, assume that everything is fine
            errorCode = obj.ErrorCodeSuccess;

            if coder.target('MATLAB')
                % Interpreted execution

                % Assume that the value will exist with the right data
                % type. Treat error conditions only if exception occurs.
                % try/catch is not supported for code generation, but this
                % block will always execute in interpreted mode.
                try
                    value = obj.ParameterStream.getValue;

                    % Handle incoming strings correctly
                    if ischar(value) && strcmp(obj.ParameterTypeML, 'uint8')
                        value = cast(value, 'uint8');
                    end

                    % Check for correct datatype
                    if ~isa(value, obj.ParameterTypeML)
                        errorCode = obj.ErrorCodeTypeMismatch;
                    end

                catch
                    % The parameter does not exist on the server (or some
                    % other undefined error occurred).
                    errorCode = obj.ErrorCodeNoParam;
                end

                % Depending on error code, assign output value and store
                % valid values.
                if errorCode == obj.ErrorCodeSuccess
                    % Remember valid value if get was successful
                    obj.LastValidValue = value;
                else
                    % An error occurred. If a valid value was previously
                    % received, return it. Otherwise, return the
                    % ParameterInitialValue.
                    if ~isempty(obj.LastValidValue)
                        value = obj.LastValidValue;
                    else
                        % Truncate initial value if necessary (to avoid
                        % wrong error codes)
                        value = obj.StateHandler.getInitialValue(obj);
                    end
                end

                % Execute simulation behavior for scalar or array data
                % types (the StateHandler encapsulates the specific
                % behavior)
                [value, receivedLength, errorCode] = obj.StateHandler.simulationStepImpl( ...
                    obj, value, errorCode, obj.ParameterTypeML);

                if errorCode == obj.ErrorCodeArrayTruncate
                    % Save last valid value in truncated form
                    obj.LastValidValue = value;
                end

            else % coder.target('Rtw')
                 % Executed in the code-generated model

                [value, receivedLength, errorCode] = ...
                    obj.StateHandler.codegenStepImpl(obj, obj.ParameterTypeML);

            end

            % Construct output
            varargout = obj.StateHandler.getStepImplOutput(obj, value, receivedLength, errorCode);
        end

        %% Custom mask visualization functions
        function maskDisplay = getMaskDisplayImpl(obj)
        %getMaskDisplayImpl Customize the mask icon display
        %   This method allows customization of the mask display code. Note
        %   that this works both for the base mask and for the mask-on-mask
        %   that we are using.

        % Construct the output labels based no the number of outputs
            numOutputs = obj.getNumOutputsImpl;
            [outputNames{1:numOutputs}] = obj.getOutputNamesImpl;

            portLabelText = {};
            for i = 1:length(outputNames)
                portLabelText = [portLabelText ['port_label(''output'', ' num2str(i) ', ''' outputNames{i} ''');']]; %#ok<AGROW>
            end
            if length(obj.ParameterName) > 16
                paramNameText = ['text(83, 12, ''' obj.ParameterName ''', ''horizontalAlignment'', ''right'');'];
            else
                paramNameText = ['text(45, 12, ''' obj.ParameterName ''', ''horizontalAlignment'', ''center'');'];
            end            
            maskDisplay = { ...
                ['plot([110,110,110,110],[110,110,110,110]);', newline], ... % Fix min and max x,y co-ordinates for autoscale mask units
                ['plot([0,0,0,0],[0,0,0,0]);', newline],...
                'color(''black'')', ...
                paramNameText, ...
                portLabelText{:}};
        end

    end

    methods (Access = protected)
        function sts = getSampleTimeImpl(obj)
        %getSampleTimeImpl Return sample time specification

            sts_base = obj.SampleTimeHandler.createSampleTimeSpec();
            
            % Add allow constant sample time to inherited
            if sts_base.Type == "Inherited"
              sts = createSampleTime(obj, "Type", "Inherited", "Allow", "Constant");
            else
              sts = sts_base;
            end
        end
    end

    %% Concrete implementations of abstract ParameterBase methods
    methods(Access = protected)
        function setStateHandler(obj, isScalar)
        %setStateHandler Set the correct state handler object (scalar or array)

            if isScalar
                obj.StateHandler = ros.slros.internal.block.GetParameterScalarState;
            else
                obj.StateHandler = ros.slros.internal.block.GetParameterArrayState;
            end
        end
    end

    methods(Static, Access = protected)
        % Note that this is ignored for the mask-on-mask
        function header = getHeaderImpl
        %getHeaderImpl Create mask header
        %   This only has an effect on the base mask.
            header = matlab.system.display.Header(mfilename('class'), ...
                                                  'Title', message('ros:slros:blockmask:GetParameterTitle').getString, ...
                                                  'Text', message('ros:slros:blockmask:GetParameterDescription').getString, ...
                                                  'ShowSourceLink', false);
        end

        % Note that this is ignored for the mask-on-mask
        % This function is important for the promotion of parameters to
        % work correctly.
        function groups = getPropertyGroupsImpl(~)
        %getPropertyGroupsImpl Create property display groups.
        %   This only has an effect on the base mask.

            paramGroup = matlab.system.display.Section(...
                'Title', message('ros:slros:blockmask:ROSParameterPrompt').getString,...
                'PropertyList', {'ParameterSource','ParameterName','ParameterType','ParameterMaxArrayLength','ParameterInitialValue'});

            otherGroup = matlab.system.display.Section(...
                'Title', message('ros:slros:blockmask:ParametersHeadingPrompt').getString,...
                'PropertyList', {'ShowErrorCodeOutput', 'SampleTime', 'ModelName', 'BlockId'});

            groups = [paramGroup,otherGroup];
        end
    end

end
