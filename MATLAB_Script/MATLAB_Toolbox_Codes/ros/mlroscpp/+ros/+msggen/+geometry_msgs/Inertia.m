
classdef Inertia < ros.Message
    %Inertia MATLAB implementation of geometry_msgs/Inertia
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'geometry_msgs/Inertia' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '1d26e4bb6c83ff141c5cf0d883c2b0fe' % The MD5 Checksum of the message definition
        PropertyList = { 'Com' 'M' 'Ixx' 'Ixy' 'Ixz' 'Iyy' 'Iyz' 'Izz' } % List of non-constant message properties
        ROSPropertyList = { 'com' 'm' 'ixx' 'ixy' 'ixz' 'iyy' 'iyz' 'izz' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.Vector3' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Com
        M
        Ixx
        Ixy
        Ixz
        Iyy
        Iyz
        Izz
    end
    methods
        function set.Com(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Vector3'};
            validateattributes(val, validClasses, validAttributes, 'Inertia', 'Com')
            obj.Com = val;
        end
        function set.M(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Inertia', 'M');
            obj.M = double(val);
        end
        function set.Ixx(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Inertia', 'Ixx');
            obj.Ixx = double(val);
        end
        function set.Ixy(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Inertia', 'Ixy');
            obj.Ixy = double(val);
        end
        function set.Ixz(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Inertia', 'Ixz');
            obj.Ixz = double(val);
        end
        function set.Iyy(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Inertia', 'Iyy');
            obj.Iyy = double(val);
        end
        function set.Iyz(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Inertia', 'Iyz');
            obj.Iyz = double(val);
        end
        function set.Izz(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Inertia', 'Izz');
            obj.Izz = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.geometry_msgs.Inertia.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.geometry_msgs.Inertia(strObj);
        end
    end
end