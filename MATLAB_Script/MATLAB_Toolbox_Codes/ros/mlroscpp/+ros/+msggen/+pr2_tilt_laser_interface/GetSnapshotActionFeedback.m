
classdef GetSnapshotActionFeedback < ros.Message
    %GetSnapshotActionFeedback MATLAB implementation of pr2_tilt_laser_interface/GetSnapshotActionFeedback
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'pr2_tilt_laser_interface/GetSnapshotActionFeedback' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '7e4d330d40a18bf66084a10cc1648970' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Status' 'Feedback' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'status' 'feedback' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.actionlib_msgs.GoalStatus' ...
            'ros.msggen.pr2_tilt_laser_interface.GetSnapshotFeedback' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Status
        Feedback
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'GetSnapshotActionFeedback', 'Header')
            obj.Header = val;
        end
        function set.Status(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.actionlib_msgs.GoalStatus'};
            validateattributes(val, validClasses, validAttributes, 'GetSnapshotActionFeedback', 'Status')
            obj.Status = val;
        end
        function set.Feedback(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.pr2_tilt_laser_interface.GetSnapshotFeedback'};
            validateattributes(val, validClasses, validAttributes, 'GetSnapshotActionFeedback', 'Feedback')
            obj.Feedback = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.pr2_tilt_laser_interface.GetSnapshotActionFeedback.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.pr2_tilt_laser_interface.GetSnapshotActionFeedback(strObj);
        end
    end
end