
classdef TimeSyncControl < ros.Message
    %TimeSyncControl MATLAB implementation of applanix_msgs/TimeSyncControl
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'applanix_msgs/TimeSyncControl' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '1e6b12cc55fda44e001851a5e5433aad' % The MD5 Checksum of the message definition
        PropertyList = { 'Transaction' 'UserPpsTime' 'UserTimeConversionFactor' } % List of non-constant message properties
        ROSPropertyList = { 'transaction' 'user_pps_time' 'user_time_conversion_factor' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Transaction
        UserPpsTime
        UserTimeConversionFactor
    end
    methods
        function set.Transaction(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TimeSyncControl', 'Transaction');
            obj.Transaction = uint16(val);
        end
        function set.UserPpsTime(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TimeSyncControl', 'UserPpsTime');
            obj.UserPpsTime = double(val);
        end
        function set.UserTimeConversionFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'TimeSyncControl', 'UserTimeConversionFactor');
            obj.UserTimeConversionFactor = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.applanix_msgs.TimeSyncControl.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.applanix_msgs.TimeSyncControl(strObj);
        end
    end
end