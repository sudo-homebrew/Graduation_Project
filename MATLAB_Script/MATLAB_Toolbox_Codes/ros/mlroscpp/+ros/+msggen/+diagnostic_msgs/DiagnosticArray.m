
classdef DiagnosticArray < ros.Message
    %DiagnosticArray MATLAB implementation of diagnostic_msgs/DiagnosticArray
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'diagnostic_msgs/DiagnosticArray' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '60810da900de1dd6ddd437c3503511da' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Status' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'status' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.diagnostic_msgs.DiagnosticStatus' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Status
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'DiagnosticArray', 'Header')
            obj.Header = val;
        end
        function set.Status(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.diagnostic_msgs.DiagnosticStatus.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.diagnostic_msgs.DiagnosticStatus'};
            validateattributes(val, validClasses, validAttributes, 'DiagnosticArray', 'Status')
            obj.Status = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.diagnostic_msgs.DiagnosticArray.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.diagnostic_msgs.DiagnosticArray(strObj);
        end
    end
end