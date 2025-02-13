
classdef TestName < ros.Message
    %TestName MATLAB implementation of roseus/TestName
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'roseus/TestName' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '70bc7fd92cd8428f6a02d7d0df4d9b80' % The MD5 Checksum of the message definition
        PropertyList = { 'Name' } % List of non-constant message properties
        ROSPropertyList = { 'name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.roseus.StringStamped' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Name
    end
    methods
        function set.Name(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.roseus.StringStamped'};
            validateattributes(val, validClasses, validAttributes, 'TestName', 'Name')
            obj.Name = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.roseus.TestName.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.roseus.TestName(strObj);
        end
    end
end
