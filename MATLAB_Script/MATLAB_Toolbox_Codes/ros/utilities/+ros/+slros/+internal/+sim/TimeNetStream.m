classdef TimeNetStream < ros.slros.internal.sim.TimeStream
%This class is for internal use only. It may be removed in the future.

%TimeNetStream Getting ROS time from ROS network

%   Copyright 2018-2020 The MathWorks, Inc.

    properties (Access = ?matlab.unittest.TestCase, Transient)
        %Node - Node object
        %   This is a MATLAB ROS node object that is used to extract the
        %   time during model simulation.
        Node
    end


    methods
        function obj = TimeNetStream(nodeObj)
        %TimeNetStream Constructor for time network stream

            if nargin == 1
                % Use the pass node object
                obj.Node = nodeObj;
            else
                % Use the global node
                obj.Node = ros.internal.Global.getNodeHandle(false);
            end
        end

        function currentTime = getTime(obj)
        %currentTime Get the current ROS time from the node
            currentTime = obj.Node.CurrentTime;
        end
    end

end
