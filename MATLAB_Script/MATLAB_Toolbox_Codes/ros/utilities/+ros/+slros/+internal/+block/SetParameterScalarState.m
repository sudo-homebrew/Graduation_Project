classdef SetParameterScalarState < ros.slros.internal.block.SetParameterStateInterface
%SetParameterScalarState Class containing logic for setting scalar parameters

%   Copyright 2015-2020 The MathWorks, Inc.

%#codegen

    methods
        function numInputs = getNumInputsImpl(~)
        %getNumInputsImpl Get the number of input arguments
        %   For scalars, we only have the Value input.

            numInputs = 1;
        end

        function inputNames = getInputNamesImpl(~)
        %getInputNamesImpl Get the labels for the input ports

            inputNames = {'Value'};
        end

        function validateInputsImpl(obj, mlDataType, varargin)
        %validateInputsImpl Validate and check all inputs to this block

            narginchk(3,3);

            % Scalar value input
            value = varargin{1};
            obj.validateValueInput(value, mlDataType);
        end

        function value = simulationStepImpl(obj, mlDataType, varargin)
        %simulationStepImpl Execute during step in simulation mode

            narginchk(3,3);
            value = varargin{1};

            % Input validation
            obj.validateValueInput(value, mlDataType);
        end

        function codegenStepImpl(~, paramObj, varargin)
        %codegenStepImpl Execute during step in code generation mode

            narginchk(3,3);
            value = varargin{1};
            coder.ceval([paramObj.BlockId '.set_parameter'], value);
        end
    end

    methods (Static, Access = private)
        function validateValueInput(value, mlDataType)
        %validateValueInput Validate the Value input
        %   MATLAB ROS allows infinity and NaN values, so we will allow
        %   them as well.

            validateattributes(value, {mlDataType}, {'nonempty', 'real', 'scalar'}, '', '');
        end
    end

end
