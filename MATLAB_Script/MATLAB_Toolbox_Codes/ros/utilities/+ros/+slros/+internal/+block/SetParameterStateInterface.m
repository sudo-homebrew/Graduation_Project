classdef SetParameterStateInterface
%SetParameterStateInterface Interface for parameter state
%   This interface is based on the strategy / state design pattern and
%   allows the SetParameter block to select at runtime if the behavior
%   for a scalar parameter value or an array value should be executed.
%   Derived classes have to implement this interface.

%   Copyright 2015-2020 The MathWorks, Inc.

%#codegen

    methods (Abstract)

        getNumInputsImpl(obj)
        %getNumInputsImpl Get the number of additional input arguments
        %   This function only returns the number of arguments that are not
        %   shared between the states.

        getInputNamesImpl(obj)
        %getInputNamesImpl Get the labels for the input ports

        validateInputsImpl(obj, mlDataType, varargin)
        %validateInputsImpl Validate and check all inputs to this block

        simulationStepImpl(obj, mlDataType, varargin)
        %simulationStepImpl Execute during step in simulation mode

        codegenStepImpl(obj, paramObj, varargin)
        %codegenStepImpl Execute during step in code generation mode
    end

end
