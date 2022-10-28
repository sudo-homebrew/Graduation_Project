classdef ParameterListStream < ros.slros.internal.sim.ParameterStream
%This class is for internal use only. It may be removed in the future.

%ParameterListStream Getting and setting ROS parameters from a list
%   There is an underlying FIFO list of parameters this is used both as
%   a source and a target for get/set operations.

%   Copyright 2015-2020 The MathWorks, Inc.

    properties
        %ParameterName - Name of the ROS parameter
        ParameterName
    end

    properties (Transient, SetAccess = ?matlab.unittest.TestCase)
        %ParameterList - Cell array of parameters
        ParameterList = {}
    end

    properties (Constant, Access = ?matlab.unittest.TestCase)
        %DefaultParameterName - The default parameter name
        DefaultParameterName = 'arbitrary'
    end


    methods
        function obj = ParameterListStream
        %ParameterListStream Constructor for parameter list stream

            obj.ParameterName = obj.DefaultParameterName;
        end

        function paramValue = getValue(obj)
        %getValue Get the value for a parameter

        % Throw an error if there is no parameter to get
            if isempty(obj.ParameterList)
                error(message('ros:mlros:param:ParameterDoesNotExist', obj.ParameterName));
            end

            % Otherwise, remove parameter from the front of the FIFO and
            % return it
            paramValue = obj.ParameterList{1};
            obj.ParameterList(1) = [];
        end

        function setValue(obj, paramValue)
        %setValue Set the value for a parameter
            obj.ParameterList{end+1} = paramValue;
        end

        function clearParameterList(obj)
        %clearParameterList Empty list of current parameters
            obj.ParameterList = {};
        end
    end

    methods
        function set.ParameterList(obj, paramList)
        %set.ParameterList Setter for ParameterList property

            validateattributes(paramList, {'cell'}, {}, 'ParameterListStream', 'ParameterList');
            obj.ParameterList = paramList;
        end
    end

end
