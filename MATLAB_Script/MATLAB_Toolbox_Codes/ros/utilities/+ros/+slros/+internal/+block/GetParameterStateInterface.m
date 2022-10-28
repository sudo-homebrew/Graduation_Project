classdef GetParameterStateInterface
%GetParameterStateInterface Interface for parameter state
%   This interface is based on the strategy / state design pattern and
%   allows the GetParameter block to select at runtime if the behavior
%   for a scalar parameter value or an array value should be executed.
%   Derived classes have to implement this interface.

%   Copyright 2015-2020 The MathWorks, Inc.

%#codegen

    properties (Constant)
        %ValidSimulinkNumericTypes - All valid numeric types in Simulink
        %   Note that this does not include 64-bit data types, since they
        %   are not currently supported.
        ValidSimulinkNumericTypes = {'uint8', 'int8', 'uint16', 'int16', ...
                            'uint32', 'int32', 'single', 'double'};
    end

    methods (Abstract)

        validateInitialValue(obj, value, mlDataType);
        %validateInitialValue Validate the user-specified initial value

        getNumOutputsImpl(obj, paramObj)
        %getNumOutputsImpl Get the number of additional output arguments
        %   This function only returns the number of arguments that are not
        %   shared between the states.

        getValueOutputSizeImpl(obj, paramObj)
        %getValueOutputSizeImpl Get the size of the Value output

        getOutputDataTypeImpl(obj, paramObj, mlDataType)
        %getOutputDataTypeImpl Get data type of outputs

        getOutputNamesImpl(obj, paramObj)
        %getOutputNamesImpl Get names of output ports

        setupSetInitialValue(obj, paramObj)
        %setupSetInitialValue Set initial value in generated code

        simulationStepImpl(obj, paramObj, value, errorCode, mlDataType)
        %simulationStepImpl Execute during step in simulation mode

        codegenStepImpl(obj, paramObj, mlDataType);
        %codegenStepImpl Execute during step in code generation mode

        getStepImplOutput(obj, paramObj, value, recvLength, errorCode)
        %getStepImplOutput Get output of step function

        getInitialValue(obj)
        %getInitialValue Get initial value (potentially processed)
    end

end
