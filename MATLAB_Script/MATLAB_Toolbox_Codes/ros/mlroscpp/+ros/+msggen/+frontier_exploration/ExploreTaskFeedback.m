
classdef ExploreTaskFeedback < ros.Message
    %ExploreTaskFeedback MATLAB implementation of frontier_exploration/ExploreTaskFeedback
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'frontier_exploration/ExploreTaskFeedback' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '00baaad5b6065fdad0f528586c3caaf2' % The MD5 Checksum of the message definition
        PropertyList = { 'NextFrontier' 'BasePosition' } % List of non-constant message properties
        ROSPropertyList = { 'next_frontier' 'base_position' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.PoseStamped' ...
            'ros.msggen.geometry_msgs.PoseStamped' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        NextFrontier
        BasePosition
    end
    methods
        function set.NextFrontier(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.PoseStamped'};
            validateattributes(val, validClasses, validAttributes, 'ExploreTaskFeedback', 'NextFrontier')
            obj.NextFrontier = val;
        end
        function set.BasePosition(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.PoseStamped'};
            validateattributes(val, validClasses, validAttributes, 'ExploreTaskFeedback', 'BasePosition')
            obj.BasePosition = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.frontier_exploration.ExploreTaskFeedback.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.frontier_exploration.ExploreTaskFeedback(strObj);
        end
    end
end