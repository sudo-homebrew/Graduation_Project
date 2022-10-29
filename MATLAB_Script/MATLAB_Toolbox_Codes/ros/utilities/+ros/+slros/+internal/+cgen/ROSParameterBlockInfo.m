classdef ROSParameterBlockInfo < ros.slros.internal.cgen.ROSBlockInfo
%This class is for internal use only. It may be removed in the future.

%ROSParameterBlockInfo is a utility class that encapsulates information about
%  a single ROS block dealing with ROS parameters in a Simulink model.
%
%  See also: cgen.ROSModelInfo

%   Copyright 2015-2018 The MathWorks, Inc.

    properties
        %ParamType - Data type of parameter value in Simulink
        ParamType

        %IsArray - Indicate if this represents an array parameter
        %   This property will be false if it is a scalar parameter.
        IsArray

        %CppParamType - Data type of parameter value in code generated from Simulink
        CppParamType

        %ROSCppParamType - Data type of parameter value in ROS C++ code
        %   Note that this C++ date type might be different from the
        %   CppParamType that Simulink uses in C/C++ code.
        ROSCppParamType
    end

end
