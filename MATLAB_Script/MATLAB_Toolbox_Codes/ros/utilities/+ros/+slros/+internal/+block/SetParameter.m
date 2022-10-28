classdef SetParameter < ...
    ros.slros.internal.block.ParameterBase
%This class is for internal use only. It may be removed in the future.

%SetParameter Set the value for a named ROS parameter
%
%   H = ros.slros.internal.block.SetParameter creates a system
%   object, H, that sets the value for a named ROS parameter. The list of
%   all ROS parameters is maintained by the ROS parameter server.
%
%   This system object is intended for use with the MATLAB System
%   block. In order to access the ROS functionality from MATLAB, see
%   ROSPARAM.
%
%   See also ros.slros.internal.block.GetParameter

%   Copyright 2015-2020 The MathWorks, Inc.

%#codegen

    properties (Constant, Access=?ros.slros.internal.block.mixin.NodeDependent)
        %MessageCatalogName - Name of this block used in message catalog
        %   This property is used by the NodeDependent base class to
        %   customize error messages with the block name.
                
        %   Due a limitation in Embedded MATLAB code-generation with UTF-8 characters,
        %   use English text instead of message("ros:slros:blockmask:SetParameterTitle").getString
        MessageCatalogName = 'ROS Set Parameter'
    end

    properties (Access = protected)
        %StateHandler - Object handling the state of the parameter value
        %   The handler uses SetParameterScalarState for scalar parameters
        %   and SetParameterArrayState for array parameters.
        StateHandler
    end

    methods
        function obj = SetParameter(varargin)
        %SetParameter Standard constructor

        % Enable code to be generated even this file is p-coded
            coder.allowpcode('plain');

            % Support name-value pair arguments when constructing the object.
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)

        function num = getNumInputsImpl(obj)
        %getNumInputsImpl Get number of inputs
            num = obj.StateHandler.getNumInputsImpl;
        end

        function num = getNumOutputsImpl(~)
        %getNumOutputsImpl Get number of outputs
            num = 0;
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
                % Append 0 to obj.ParameterName, since MATLAB doesn't
                % automatically zero-terminate strings in generated code
                zeroDelimName = [obj.ParameterName 0];
                coder.ceval([obj.BlockId '.initialize'], zeroDelimName);
            end
        end

        function varargout = getInputNamesImpl(obj)
        %getInputNamesImpl Get the labels for the input ports

            varargout = obj.StateHandler.getInputNamesImpl;
        end


        %%
        function stepImpl(obj, varargin)
        %stepImpl System Object step call
        %   STEPIMPL(OBJ, VALUE) is the main processing
        %   loop for this system object if a scalar parameter is set.
        %
        %   STEPIMPL(OBJ, VALUE, ARRAYLENGTH) is the main processing
        %   loop for this system object if a string or numeric array
        %   parameter is set.
        %
        %   VALUE is the parameter value that should be set on the ROS
        %   parameter server.
        %   ARRAYLENGTH is the length of the array that should be sent.
        %   ARRAYLENGTH must be smaller than the length of the array in
        %   VALUE.
        %
        %   Note that setting a parameter value on the server will be
        %   executed synchronously both in MATLAB and in C++, meaning
        %   that stepImpl will block until the value is set on the
        %   parameter server.

            if coder.target('MATLAB')
                % Interpreted execution

                value = obj.StateHandler.simulationStepImpl(obj.ParameterTypeML, varargin{:});
                obj.ParameterStream.setValue(value);

            else % coder.target('Rtw')
                 % Executed in the code-generated model

                obj.StateHandler.codegenStepImpl(obj, varargin{:});

            end

        end

        function validateInputsImpl(obj, varargin)
        %validateInputsImpl Validate and check all inputs to this block
        %   This function is only called once when the model starts up,
        %   not at runtime.

            obj.StateHandler.validateInputsImpl(obj.ParameterTypeML, varargin{:});
        end



        %% Custom mask visualization functions
        function maskDisplay = getMaskDisplayImpl(obj)
        %getMaskDisplayImpl Customize the mask icon display
        %   This method allows customization of the mask display code. Note
        %   that this works both for the base mask and for the mask-on-mask
        %   that we are using.

        % Construct the input labels based no the number of inputs
            numInputs = obj.getNumInputsImpl;
            [inputNames{1:numInputs}] = obj.getInputNamesImpl;

            portLabelText = {};
            for i = 1:length(inputNames)
                portLabelText = [portLabelText ['port_label(''input'', ' num2str(i) ', ''' inputNames{i} ''');']]; %#ok<AGROW>
            end
            if length(obj.ParameterName) > 16
                paramNameText = ['text(83, 12, ''' obj.ParameterName ''', ''horizontalAlignment'', ''right'');'];
            else
                paramNameText = ['text(50, 12, ''' obj.ParameterName ''', ''horizontalAlignment'', ''center'');'];
            end              
            maskDisplay = { ...
                ['plot([110,110,110,110],[110,110,110,110]);', newline], ... % Fix min and max x,y co-ordinates for autoscale mask units
                ['plot([0,0,0,0],[0,0,0,0]);', newline],...
                'color(''black'')', ...
                paramNameText, ...
                portLabelText{:}};
        end
        
        function sts = getSampleTimeImpl(obj)
          sts = createSampleTime(obj, "Type", "Inherited", "Allow", "Constant");
        end

        %% Ensure that input cannot change size once system object is locked
        function flag = isInputSizeMutableImpl(~,~)
        %isInputSizeMutableImpl Mutable input size status
        %   This function will be called once for each input of the
        %   system block.
            flag = false;
        end
    end

    %% Concrete implementations of abstract ParameterBase methods
    methods(Access = protected)
        function setStateHandler(obj, isScalar)
        %setStateHandler Set the correct state handler object (scalar or array)

            if isScalar
                obj.StateHandler = ros.slros.internal.block.SetParameterScalarState;
            else
                obj.StateHandler = ros.slros.internal.block.SetParameterArrayState;
            end
        end
    end

    methods(Static, Access = protected)
        % Note that this is ignored for the mask-on-mask
        function header = getHeaderImpl
        %getHeaderImpl Create mask header
        %   This only has an effect on the base mask.
            header = matlab.system.display.Header(mfilename('class'), ...
                                                  'Title', message('ros:slros:blockmask:SetParameterTitle').getString, ...
                                                  'Text', message('ros:slros:blockmask:SetParameterDescription').getString, ...
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
                'PropertyList', {'ParameterSource','ParameterName','ParameterType'});

            otherGroup = matlab.system.display.Section(...
                'Title', message('ros:slros:blockmask:ParametersHeadingPrompt').getString',...
                'PropertyList', {'ModelName', 'BlockId'});

            groups = [paramGroup, otherGroup];
        end

    end

end
