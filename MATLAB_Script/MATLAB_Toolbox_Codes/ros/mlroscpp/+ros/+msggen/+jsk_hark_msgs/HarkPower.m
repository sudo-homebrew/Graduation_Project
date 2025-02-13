
classdef HarkPower < ros.Message
    %HarkPower MATLAB implementation of jsk_hark_msgs/HarkPower
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'jsk_hark_msgs/HarkPower' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '251c13d7a8be27144a2b24c6f53df705' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Count' 'Directions' 'DataBytes' 'Powers' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'count' 'directions' 'data_bytes' 'powers' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Count
        Directions
        DataBytes
        Powers
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'HarkPower', 'Header')
            obj.Header = val;
        end
        function set.Count(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'HarkPower', 'Count');
            obj.Count = int32(val);
        end
        function set.Directions(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'HarkPower', 'Directions');
            obj.Directions = int32(val);
        end
        function set.DataBytes(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'HarkPower', 'DataBytes');
            obj.DataBytes = int32(val);
        end
        function set.Powers(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = single.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HarkPower', 'Powers');
            obj.Powers = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.jsk_hark_msgs.HarkPower.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.jsk_hark_msgs.HarkPower(strObj);
        end
    end
end
