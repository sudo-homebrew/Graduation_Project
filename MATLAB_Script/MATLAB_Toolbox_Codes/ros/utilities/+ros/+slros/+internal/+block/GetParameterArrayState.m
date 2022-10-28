classdef GetParameterArrayState < ros.slros.internal.block.GetParameterStateInterface
%GetParameterArrayState Class containing logic for getting array parameters

%   Copyright 2015-2020 The MathWorks, Inc.

%#codegen

    methods
        function validValue = validateInitialValue(obj, value, mlDataType)
        %validateInitialValue Validate the user-specified initial value

        % Validation attributes that are common to all data types
            argName = 'ParameterInitialValue';

            % Validate input based on the MATLAB data type of the parameter
            switch mlDataType
              case 'uint8'
                % Initial string values are defined as uint8 arrays
                validateattributes(value, obj.ValidSimulinkNumericTypes, ...
                                   {'real', 'integer', '<=', intmax('uint8'), ...
                                    '>=', intmin('uint8')}, '', argName);

                % If non-empty, ensure that input is a scalar or a vector
                if ~isempty(value)
                    validateattributes(value, obj.ValidSimulinkNumericTypes, {'vector'}, '', argName);
                end

              otherwise
                error(message('ros:slros:getparam:InitialDataTypeNotValid', mlDataType));
            end

            % Now cast the value to the intended data type
            validValue = cast(value, mlDataType);
        end

        function numOutputs = getNumOutputsImpl(~, paramObj)
        %getNumOutputsImpl Get the number of output arguments

            if paramObj.ShowErrorCodeOutput
                % 3 outputs: Value, Length, ErrorCode
                numOutputs = 3;
            else
                % 2 outputs: Value, Length
                numOutputs = 2;
            end
        end

        function valueSize = getValueOutputSizeImpl(~, paramObj)
        %getValueOutputSizeImpl Get the size of the Value output

            valueSize = double([1 paramObj.ParameterMaxArrayLength]);
        end

        function outputTypes = getOutputDataTypeImpl(~, paramObj, mlDataType)
        %getOutputDataTypeImpl Get data type of outputs

            if paramObj.ShowErrorCodeOutput
                % 3 outputs: Value, Length, ErrorCode
                outputTypes = {mlDataType, 'uint32', 'uint8'};
            else
                % 2 outputs: Value, Length
                outputTypes = {mlDataType, 'uint32'};
            end
        end

        function outputNames = getOutputNamesImpl(~, paramObj)
        %getOutputNamesImpl Get names of output ports

            if paramObj.ShowErrorCodeOutput
                outputNames = {'Value','Length','ErrorCode'};
            else
                outputNames = {'Value','Length'};
            end
        end

        function setupSetInitialValue(~, paramObj)
        %setupSetInitialValue Set initial value in generated code

            initialValue = char(paramObj.ParameterInitialValue);
            initialLength = uint32(length(initialValue));
            maxLength = paramObj.ParameterMaxArrayLength;

            % Truncate the initial value in generated code if it is
            % longer than the maximum length
            if initialLength > maxLength
                initialLength = maxLength;
                truncatedValue = initialValue(1:maxLength);
                coder.ceval([paramObj.BlockId '.set_initial_value'], coder.rref(truncatedValue), ...
                            initialLength);
            else
                coder.ceval([paramObj.BlockId '.set_initial_value'], coder.rref(initialValue), ...
                            initialLength);
            end
        end

        function [value, recvLength, errorCode] = simulationStepImpl(~, paramObj, value, errorCode, mlDataType)
        %simulationStepImpl Execute during step in simulation mode

        % Always make it a row vector
            if iscolumn(value)
                value = value.';
            end

            % Pad or truncate array
            % Don't do anything if array has maximum length already
            recvLength = uint32(length(value));
            maxLength = paramObj.ParameterMaxArrayLength;
            if recvLength > maxLength
                % Truncate
                recvLength = maxLength;
                value = value(1:maxLength);

                % Set error code to indicate that truncation occurred.
                errorCode = paramObj.ErrorCodeArrayTruncate;

            elseif recvLength < maxLength
                % Pad with zeros
                value(recvLength+1:maxLength) = cast(0, mlDataType);
            end
        end

        function [value, recvLength, errorCode] = codegenStepImpl(~, paramObj, mlDataType)
        %codegenStepImpl Execute during step in code generation mode

        % Initializing paramValue with ParameterMaxArrayLength zeros ensures
        % that we don't have to take care of zero padding in the C++ code.
            paramValue = cast(zeros(1, paramObj.ParameterMaxArrayLength), mlDataType);
            recvLength = uint32(1);

            % Pre-allocate error code to allow coders to infer data type
            % for ceval call.
            errorCode = uint8(0); 

            if isa(paramValue, 'uint8')
                % Special handling for strings to avoid compilation
                % warnings
                charValue = char(zeros(1, paramObj.ParameterMaxArrayLength));
                errorCode = coder.ceval([paramObj.BlockId '.get_parameter'], ...
                                        paramObj.ParameterMaxArrayLength, coder.wref(charValue), coder.wref(recvLength));
                value = cast(charValue, 'uint8');
            else
                value = coder.nullcopy(paramValue);
                errorCode = coder.ceval([paramObj.BlockId '.get_parameter'], ...
                                        paramObj.ParameterMaxArrayLength, coder.wref(value), coder.wref(recvLength));
            end
        end

        function outputArgs = getStepImplOutput(~, paramObj, value, recvLength, errorCode)
        %getStepImplOutput Get output for step function

            if paramObj.ShowErrorCodeOutput
                outputArgs = cell(1,3);
                outputArgs{1} = value;
                outputArgs{2} = recvLength;
                outputArgs{3} = errorCode;
            else
                outputArgs = cell(1,2);
                outputArgs{1} = value;
                outputArgs{2} = recvLength;
            end
        end

        function initialValue = getInitialValue(~, paramObj)
        %getInitialValue Get initial value (potentially processed)
        %   In the case of an array parameter, truncate the initial
        %   value, if it is longer than the maximum array length.

            initValue = paramObj.ParameterInitialValue;
            if length(initValue) > paramObj.ParameterMaxArrayLength
                initialValue = initValue(1:paramObj.ParameterMaxArrayLength);
            else
                initialValue = initValue;
            end
        end
    end

end
