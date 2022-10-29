classdef ParameterStream < handle
%This class is for internal use only. It may be removed in the future.

%ParameterStream Encapsulates an interface for getting and setting ROS parameters
%   This interface is purely abstract and has to be implemented by
%   concrete subclasses.

%   Copyright 2015-2020 The MathWorks, Inc.

    properties (Abstract)
        %ParameterName - Name of the ROS parameter
        ParameterName
    end


    methods (Abstract)
        paramValue = getValue(obj)
        %getValue Get the value for a parameter
        %   This abstract method has to be implemented by all subclasses of
        %   ParameterStream.

        setValue(obj, paramValue)
        %setValue Set the value for a parameter
        %   This abstract method has to be implemented by all subclasses of
        %   ParameterStream.
    end

end
