
classdef UserTimeStatus < ros.Message
    %UserTimeStatus MATLAB implementation of applanix_msgs/UserTimeStatus
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'applanix_msgs/UserTimeStatus' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '3d0128542d920da1a88ece6cf34151c4' % The MD5 Checksum of the message definition
        PropertyList = { 'Td' 'TimeSynchRejections' 'UserTimeResynchs' 'UserTimeValid' 'TimeSynchMessageReceived' } % List of non-constant message properties
        ROSPropertyList = { 'td' 'time_synch_rejections' 'user_time_resynchs' 'user_time_valid' 'time_synch_message_received' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.applanix_msgs.TimeDistance' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Td
        TimeSynchRejections
        UserTimeResynchs
        UserTimeValid
        TimeSynchMessageReceived
    end
    methods
        function set.Td(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.applanix_msgs.TimeDistance'};
            validateattributes(val, validClasses, validAttributes, 'UserTimeStatus', 'Td')
            obj.Td = val;
        end
        function set.TimeSynchRejections(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'UserTimeStatus', 'TimeSynchRejections');
            obj.TimeSynchRejections = uint32(val);
        end
        function set.UserTimeResynchs(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'UserTimeStatus', 'UserTimeResynchs');
            obj.UserTimeResynchs = uint32(val);
        end
        function set.UserTimeValid(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'UserTimeStatus', 'UserTimeValid');
            obj.UserTimeValid = uint8(val);
        end
        function set.TimeSynchMessageReceived(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'UserTimeStatus', 'TimeSynchMessageReceived');
            obj.TimeSynchMessageReceived = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.applanix_msgs.UserTimeStatus.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.applanix_msgs.UserTimeStatus(strObj);
        end
    end
end