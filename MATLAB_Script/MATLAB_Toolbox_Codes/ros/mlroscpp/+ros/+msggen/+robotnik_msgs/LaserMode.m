
classdef LaserMode < ros.Message
    %LaserMode MATLAB implementation of robotnik_msgs/LaserMode
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'robotnik_msgs/LaserMode' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '6d9ec22e8c4a4279a8a61cc162391fee' % The MD5 Checksum of the message definition
        PropertyList = { 'Name' } % List of non-constant message properties
        ROSPropertyList = { 'name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        STANDARD = 'standard';
        DOCKINGSTATION = 'docking_station';
        CART = 'cart';
        CARTDOCKINGCART = 'cart_docking_cart';
        DOCKINGCART = 'docking_cart';
        INVALID = 'invalid';
    end
    properties
        Name
    end
    methods
        function set.Name(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'LaserMode', 'Name');
            obj.Name = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.robotnik_msgs.LaserMode.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.robotnik_msgs.LaserMode(strObj);
        end
    end
end