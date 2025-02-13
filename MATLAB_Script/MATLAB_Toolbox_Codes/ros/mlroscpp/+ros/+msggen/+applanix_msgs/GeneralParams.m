
classdef GeneralParams < ros.Message
    %GeneralParams MATLAB implementation of applanix_msgs/GeneralParams
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'applanix_msgs/GeneralParams' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'dd32351725c9c39d1131c0be17b24a7b' % The MD5 Checksum of the message definition
        PropertyList = { 'ImuLeverArm' 'PrimaryGnssLeverArm' 'Aux1GnssLeverArm' 'Aux2GnssLeverArm' 'ImuMountingAngle' 'RefMountingAngle' 'Transaction' 'TimeTypes' 'DistanceType' 'Autostart' 'Multipath' } % List of non-constant message properties
        ROSPropertyList = { 'imu_lever_arm' 'primary_gnss_lever_arm' 'aux_1_gnss_lever_arm' 'aux_2_gnss_lever_arm' 'imu_mounting_angle' 'ref_mounting_angle' 'transaction' 'time_types' 'distance_type' 'autostart' 'multipath' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.Point32' ...
            'ros.msggen.geometry_msgs.Point32' ...
            'ros.msggen.geometry_msgs.Point32' ...
            'ros.msggen.geometry_msgs.Point32' ...
            'ros.msggen.geometry_msgs.Point32' ...
            'ros.msggen.geometry_msgs.Point32' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        MULTIPATHLOW = uint8(0)
        MULTIPATHMEDIUM = uint8(1)
        MULTIPATHHIGH = uint8(2)
    end
    properties
        ImuLeverArm
        PrimaryGnssLeverArm
        Aux1GnssLeverArm
        Aux2GnssLeverArm
        ImuMountingAngle
        RefMountingAngle
        Transaction
        TimeTypes
        DistanceType
        Autostart
        Multipath
    end
    methods
        function set.ImuLeverArm(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point32'};
            validateattributes(val, validClasses, validAttributes, 'GeneralParams', 'ImuLeverArm')
            obj.ImuLeverArm = val;
        end
        function set.PrimaryGnssLeverArm(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point32'};
            validateattributes(val, validClasses, validAttributes, 'GeneralParams', 'PrimaryGnssLeverArm')
            obj.PrimaryGnssLeverArm = val;
        end
        function set.Aux1GnssLeverArm(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point32'};
            validateattributes(val, validClasses, validAttributes, 'GeneralParams', 'Aux1GnssLeverArm')
            obj.Aux1GnssLeverArm = val;
        end
        function set.Aux2GnssLeverArm(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point32'};
            validateattributes(val, validClasses, validAttributes, 'GeneralParams', 'Aux2GnssLeverArm')
            obj.Aux2GnssLeverArm = val;
        end
        function set.ImuMountingAngle(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point32'};
            validateattributes(val, validClasses, validAttributes, 'GeneralParams', 'ImuMountingAngle')
            obj.ImuMountingAngle = val;
        end
        function set.RefMountingAngle(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point32'};
            validateattributes(val, validClasses, validAttributes, 'GeneralParams', 'RefMountingAngle')
            obj.RefMountingAngle = val;
        end
        function set.Transaction(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GeneralParams', 'Transaction');
            obj.Transaction = uint16(val);
        end
        function set.TimeTypes(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GeneralParams', 'TimeTypes');
            obj.TimeTypes = uint8(val);
        end
        function set.DistanceType(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GeneralParams', 'DistanceType');
            obj.DistanceType = uint8(val);
        end
        function set.Autostart(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GeneralParams', 'Autostart');
            obj.Autostart = uint8(val);
        end
        function set.Multipath(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GeneralParams', 'Multipath');
            obj.Multipath = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.applanix_msgs.GeneralParams.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.applanix_msgs.GeneralParams(strObj);
        end
    end
end
