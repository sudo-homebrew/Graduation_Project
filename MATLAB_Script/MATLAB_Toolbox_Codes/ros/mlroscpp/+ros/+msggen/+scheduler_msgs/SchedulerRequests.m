
classdef SchedulerRequests < ros.Message
    %SchedulerRequests MATLAB implementation of scheduler_msgs/SchedulerRequests
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'scheduler_msgs/SchedulerRequests' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd0adc2f83296939c4b208a3e0619e86f' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Requester' 'Requests' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'requester' 'requests' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.uuid_msgs.UniqueID' ...
            'ros.msggen.scheduler_msgs.Request' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Requester
        Requests
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'SchedulerRequests', 'Header')
            obj.Header = val;
        end
        function set.Requester(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.uuid_msgs.UniqueID'};
            validateattributes(val, validClasses, validAttributes, 'SchedulerRequests', 'Requester')
            obj.Requester = val;
        end
        function set.Requests(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.scheduler_msgs.Request.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.scheduler_msgs.Request'};
            validateattributes(val, validClasses, validAttributes, 'SchedulerRequests', 'Requests')
            obj.Requests = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.scheduler_msgs.SchedulerRequests.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.scheduler_msgs.SchedulerRequests(strObj);
        end
    end
end
