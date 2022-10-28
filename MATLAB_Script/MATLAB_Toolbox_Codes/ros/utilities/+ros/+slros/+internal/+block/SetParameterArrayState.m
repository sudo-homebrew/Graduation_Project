classdef SetParameterArrayState < ros.slros.internal.block.SetParameterStateInterface
%SetParameterArrayState Class containing logic for setting array parameters

%   Copyright 2015-2020 The MathWorks, Inc.

%#codegen

    methods
        function numInputs = getNumInputsImpl(~)
        %getNumInputsImpl Get the number of input arguments

        % In the case of an array, we have both a Value and a Length
        % input
            numInputs = 2;
        end

        function inputNames = getInputNamesImpl(~)
        %getInputNamesImpl Get the labels for the input ports

        % In addition to the array, also output its length
            inputNames = {'Value', 'Length'};
        end

        function validateInputsImpl(obj, mlDataType, varargin)
        %validateInputsImpl Validate and check all inputs to this block

            narginchk(4,4);
            [value, arrayLength] = varargin{:};

            % This is an array input
            obj.validateValueInput(value, mlDataType);
            obj.validateArrayLengthInput(arrayLength);
        end

        function value = simulationStepImpl(obj, mlDataType, varargin)
        %simulationStepImpl Execute during step in simulation mode

            narginchk(4,4);
            [value, arrayLength] = varargin{:};

            % Input validation
            obj.validateValueInput(value, mlDataType);
            arrayLength = obj.validateArrayLengthInput(arrayLength);

            if arrayLength > length(value)
                % Error out if specified length is bigger than length of
                % actual array
                ex = MException(message('ros:slros:setparam:LengthLargerThanArray', ...
                                        num2str(length(value)), num2str(arrayLength)));
                throwAsCaller(ex);
            end

            if arrayLength < length(value)
                % Only send the part of the input array up to arrayLength.
                value = value(1:arrayLength);
            end

            % Special case if input is a string
            if isa(value, 'uint8')
                value = cast(value, 'char');
            end
        end

        function codegenStepImpl(~, paramObj, varargin)
        %codegenStepImpl Execute during step in code generation mode

            value = varargin{1};
            arrayLength = uint32(varargin{2});
            maxLength = length(value);

            % In simulation mode, we display an error if the number of
            % elements to write is bigger than the length of the actual
            % array. We want to avoid runtime errors in generated code, so
            % use rosout to log the error and return without setting the
            % parameter.
            if arrayLength > maxLength
                modelName = [paramObj.ModelName 0];
                coder.ceval([paramObj.BlockId '.length_error'], modelName, ...
                            arrayLength, uint32(maxLength));
                return;
            end

            if isa(value, 'uint8')
                % Special handling for strings necessary. Cast it to
                % character before passing to C++ case. This avoid compiler
                % warnings on build.
                charValue = cast(value, 'char');
                coder.ceval([paramObj.BlockId '.set_parameter_array'], coder.rref(charValue), maxLength, arrayLength);
            else
                coder.ceval([paramObj.BlockId '.set_parameter_array'], coder.rref(value), maxLength, arrayLength);
            end
        end
    end

    methods (Static, Access = private)
        function validateValueInput(value, mlDataType)
        %validateValueInput Validate the Value input
        %   MATLAB ROS allows infinity and NaN values, so we will allow
        %   them as well.
        %   Since we are using fixed-size signals, system object
        %   infrastructure prevents us from allowing empty inputs. If
        %   users want to send empty arrays, they can give an arbitrary
        %   input to Value and set the Length input to 0.

            validateattributes(value, {mlDataType}, {'nonempty', 'real', 'vector'}, '', '');
        end

        function validArrayLength = validateArrayLengthInput(arrayLength)
        %validateArrayLengthInput Validate the Length input

        % Since we will cast it to a uint32, ensure that the input is
        % within its numeric limits
            validateattributes(arrayLength, {'numeric'}, {'scalar', 'real', '>=', 0, '<=', intmax('uint32')}, '', '');
            validArrayLength = uint32(arrayLength);
        end
    end
end
