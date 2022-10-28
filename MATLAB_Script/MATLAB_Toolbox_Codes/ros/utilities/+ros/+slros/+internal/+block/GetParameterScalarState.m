classdef GetParameterScalarState < ros.slros.internal.block.GetParameterStateInterface
%GetParameterScalarState Class containing logic for getting scalar parameters

%   Copyright 2015-2020 The MathWorks, Inc.

%#codegen

    methods
        function validValue = validateInitialValue(obj, value, mlDataType)
        %validateInitialValue Validate the user-specified initial value

        % Validation attributes that are common to all data types
            argName = 'ParameterInitialValue';

            % Validate input based on the MATLAB data type of the parameter
            switch mlDataType
              case 'double'
                % Any other numeric data type can be represented as a
                % double.
                validateattributes(value, obj.ValidSimulinkNumericTypes, {'nonempty', 'real', 'scalar'}, ...
                                   '', argName);
              case 'int32'
                % Make sure that value is within int32 data type limits.
                validateattributes(value, obj.ValidSimulinkNumericTypes, ...
                                   {'nonempty', 'real', 'scalar', 'integer', '<=', intmax('int32'), ...
                                    '>=', intmin('int32')}, '', argName);
              case 'logical'
                % Allow numeric inputs as long as they can be
                % interpreted as logical.

                % Excluding nan inputs, since they cannot be cast to a
                % logical
                validateattributes(value, {'uint8', 'int8', 'uint16', 'int16', ...
                                    'uint32', 'int32', 'single', 'double', 'logical'}, ...
                                   {'nonempty', 'real', 'scalar', 'nonnan'}, '', ...
                                   argName);
              otherwise
                error(message('ros:slros:getparam:InitialDataTypeNotValid', mlDataType));
            end

            % Now cast the value to the intended data type
            validValue = cast(value, mlDataType);
        end

        function numOutputs = getNumOutputsImpl(~, paramObj)
        %getNumOutputsImpl Get the number of output arguments

            if paramObj.ShowErrorCodeOutput
                % 2 outputs: Value, ErrorCode
                numOutputs = 2;
            else
                % 1 output: Value
                numOutputs = 1;
            end
        end

        function valueSize = getValueOutputSizeImpl(~, ~)
        %getValueOutputSizeImpl Get the size of the Value output

        % The output is scalar
            valueSize = [1 1];
        end

        function outputTypes = getOutputDataTypeImpl(~, paramObj, mlDataType)
        %getOutputDataTypeImpl Get data type of outputs

            if paramObj.ShowErrorCodeOutput
                % 2 outputs: Value, ErrorCode
                outputTypes = {mlDataType, 'uint8'};
            else
                % 1 output: Value
                outputTypes = {mlDataType};
            end
        end

        function outputNames = getOutputNamesImpl(~, paramObj)
        %getOutputNamesImpl Get names of output ports

            if paramObj.ShowErrorCodeOutput
                outputNames = {'Value', 'ErrorCode'};
            else
                outputNames = {'Value'};
            end
        end

        function setupSetInitialValue(~, paramObj)
        %setupSetInitialValue Set initial value in generated code

            coder.ceval([paramObj.BlockId '.set_initial_value'], ...
                        paramObj.ParameterInitialValue);
        end

        function [value, recvLength, errorCode] = simulationStepImpl(~, ~, value, errorCode, ~)
        %simulationStepImpl Execute during step in simulation mode

            recvLength = uint32(1);
        end

        function [value, recvLength, errorCode] = codegenStepImpl(~, paramObj, mlDataType)
        %codegenStepImpl Execute during step in code generation mode

            paramValue = cast(0, mlDataType);
            recvLength = uint32(1);

            % Pre-allocate error code to allow coders to infer data type
            % for ceval call.
            errorCode = uint8(0); 

            value = coder.nullcopy(paramValue);
            errorCode = coder.ceval([paramObj.BlockId '.get_parameter'], coder.wref(value));
        end

        function outputArgs = getStepImplOutput(~, paramObj, value, ~, errorCode)
        %getStepImplOutput Get output for step function

            if paramObj.ShowErrorCodeOutput
                outputArgs = cell(1,2);
                outputArgs{1} = value;
                outputArgs{2} = errorCode;
            else
                outputArgs = cell(1,1);
                outputArgs{1} = value;
            end
        end

        function initialValue = getInitialValue(~, paramObj)
        %getInitialValue Get initial value (potentially processed)
            initialValue = paramObj.ParameterInitialValue;
        end
    end

end
