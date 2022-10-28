classdef ParameterNetStream < ros.slros.internal.sim.ParameterStream
%This class is for internal use only. It may be removed in the future.

%ParameterNetStream Getting and setting ROS parameters from ROS network

%   Copyright 2015-2020 The MathWorks, Inc.

    properties
        %ParameterName - Name of the ROS parameter
        ParameterName
    end

    properties (Access = ?matlab.unittest.TestCase, Transient)
        %ParameterTree - Parameter tree object
        %   This is a MATLAB ROS object that is used to communicate with
        %   the parameter server during model simulation.
        ParameterTree
    end


    methods
        function obj = ParameterNetStream(name, nodeObj)
        %ParameterNetStream Constructor for parameter network stream

            obj.ParameterName = name;
            if exist('nodeObj', 'var')
                % Use the node object to create a parameter tree
                obj.ParameterTree = ros.ParameterTree(nodeObj);
            else
                % Use the global node to create the parameter tree
                obj.ParameterTree = rosparam;
            end
        end

        function paramValue = getValue(obj)
        %getValue Get the value for a parameter
            paramValue = obj.ParameterTree.get(obj.ParameterName);
        end

        function setValue(obj, paramValue)
        %setValue Set the value for a parameter
            obj.ParameterTree.set(obj.ParameterName, paramValue);
        end
    end

end
