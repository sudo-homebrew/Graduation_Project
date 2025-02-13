
classdef cartesian_position < ros.Message
    %cartesian_position MATLAB implementation of sr_robot_msgs/cartesian_position
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'sr_robot_msgs/cartesian_position' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'cfc3f1750a36a668eb93288ed1128f42' % The MD5 Checksum of the message definition
        PropertyList = { 'TipName' 'TipPosX' 'TipPosY' 'TipPosZ' 'TipOrientationRho' 'TipOrientationTheta' 'TipOrientationSigma' } % List of non-constant message properties
        ROSPropertyList = { 'tip_name' 'tip_pos_x' 'tip_pos_y' 'tip_pos_z' 'tip_orientation_rho' 'tip_orientation_theta' 'tip_orientation_sigma' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
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
        TipName
        TipPosX
        TipPosY
        TipPosZ
        TipOrientationRho
        TipOrientationTheta
        TipOrientationSigma
    end
    methods
        function set.TipName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'cartesian_position', 'TipName');
            obj.TipName = char(val);
        end
        function set.TipPosX(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'cartesian_position', 'TipPosX');
            obj.TipPosX = single(val);
        end
        function set.TipPosY(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'cartesian_position', 'TipPosY');
            obj.TipPosY = single(val);
        end
        function set.TipPosZ(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'cartesian_position', 'TipPosZ');
            obj.TipPosZ = single(val);
        end
        function set.TipOrientationRho(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'cartesian_position', 'TipOrientationRho');
            obj.TipOrientationRho = single(val);
        end
        function set.TipOrientationTheta(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'cartesian_position', 'TipOrientationTheta');
            obj.TipOrientationTheta = single(val);
        end
        function set.TipOrientationSigma(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'cartesian_position', 'TipOrientationSigma');
            obj.TipOrientationSigma = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.sr_robot_msgs.cartesian_position.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.sr_robot_msgs.cartesian_position(strObj);
        end
    end
end
