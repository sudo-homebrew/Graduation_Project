
classdef CalibratedInstallationParameters < ros.Message
    %CalibratedInstallationParameters MATLAB implementation of applanix_msgs/CalibratedInstallationParameters
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'applanix_msgs/CalibratedInstallationParameters' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '1477542d348ed571610f2395143ec8d8' % The MD5 Checksum of the message definition
        PropertyList = { 'Td' 'PrimaryGnssLeverArm' 'Aux1GnssLeverArm' 'Aux2GnssLeverArm' 'DmiLeverArm' 'Status' 'PrimaryGnssLeverFom' 'Aux1GnssLeverFom' 'Aux2GnssLeverFom' 'DmiLeverFom' 'DmiScaleFactor' 'DmiScaleFactorFom' } % List of non-constant message properties
        ROSPropertyList = { 'td' 'primary_gnss_lever_arm' 'aux_1_gnss_lever_arm' 'aux_2_gnss_lever_arm' 'dmi_lever_arm' 'status' 'primary_gnss_lever_fom' 'aux_1_gnss_lever_fom' 'aux_2_gnss_lever_fom' 'dmi_lever_fom' 'dmi_scale_factor' 'dmi_scale_factor_fom' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.applanix_msgs.TimeDistance' ...
            'ros.msggen.geometry_msgs.Point32' ...
            'ros.msggen.geometry_msgs.Point32' ...
            'ros.msggen.geometry_msgs.Point32' ...
            'ros.msggen.geometry_msgs.Point32' ...
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
        STATUSPRIMARYGNSSLEVERARMCALIBRATING = uint16(1)
        STATUSAUX1GNSSLEVERARMCALIBRATING = uint16(2)
        STATUSAUX2GNSSLEVERARMCALIBRATING = uint16(4)
        STATUSDMILEVERARMCALIBRATING = uint16(8)
        STATUSDMISCALEFACTORCALIBRATING = uint16(16)
        STATUSPOSITIONFIXLEVERARMCALIBRATING = uint16(64)
        STATUSPRIMARYGNSSLEVERARMCALIBRATED = uint16(256)
        STATUSAUX1GNSSLEVERARMCALIBRATED = uint16(512)
        STATUSAUX2GNSSLEVERARMCALIBRATED = uint16(1024)
        STATUSDMILEVERARMCALIBRATED = uint16(2048)
        STATUSDMISCALEFACTORCALIBRATED = uint16(4096)
        STATUSPOSITIONFIXLEVERARMCALIBRATED = uint16(16384)
    end
    properties
        Td
        PrimaryGnssLeverArm
        Aux1GnssLeverArm
        Aux2GnssLeverArm
        DmiLeverArm
        Status
        PrimaryGnssLeverFom
        Aux1GnssLeverFom
        Aux2GnssLeverFom
        DmiLeverFom
        DmiScaleFactor
        DmiScaleFactorFom
    end
    methods
        function set.Td(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.applanix_msgs.TimeDistance'};
            validateattributes(val, validClasses, validAttributes, 'CalibratedInstallationParameters', 'Td')
            obj.Td = val;
        end
        function set.PrimaryGnssLeverArm(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point32'};
            validateattributes(val, validClasses, validAttributes, 'CalibratedInstallationParameters', 'PrimaryGnssLeverArm')
            obj.PrimaryGnssLeverArm = val;
        end
        function set.Aux1GnssLeverArm(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point32'};
            validateattributes(val, validClasses, validAttributes, 'CalibratedInstallationParameters', 'Aux1GnssLeverArm')
            obj.Aux1GnssLeverArm = val;
        end
        function set.Aux2GnssLeverArm(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point32'};
            validateattributes(val, validClasses, validAttributes, 'CalibratedInstallationParameters', 'Aux2GnssLeverArm')
            obj.Aux2GnssLeverArm = val;
        end
        function set.DmiLeverArm(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Point32'};
            validateattributes(val, validClasses, validAttributes, 'CalibratedInstallationParameters', 'DmiLeverArm')
            obj.DmiLeverArm = val;
        end
        function set.Status(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'CalibratedInstallationParameters', 'Status');
            obj.Status = uint16(val);
        end
        function set.PrimaryGnssLeverFom(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'CalibratedInstallationParameters', 'PrimaryGnssLeverFom');
            obj.PrimaryGnssLeverFom = uint16(val);
        end
        function set.Aux1GnssLeverFom(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'CalibratedInstallationParameters', 'Aux1GnssLeverFom');
            obj.Aux1GnssLeverFom = uint16(val);
        end
        function set.Aux2GnssLeverFom(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'CalibratedInstallationParameters', 'Aux2GnssLeverFom');
            obj.Aux2GnssLeverFom = uint16(val);
        end
        function set.DmiLeverFom(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'CalibratedInstallationParameters', 'DmiLeverFom');
            obj.DmiLeverFom = uint16(val);
        end
        function set.DmiScaleFactor(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'CalibratedInstallationParameters', 'DmiScaleFactor');
            obj.DmiScaleFactor = single(val);
        end
        function set.DmiScaleFactorFom(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'CalibratedInstallationParameters', 'DmiScaleFactorFom');
            obj.DmiScaleFactorFom = uint16(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.applanix_msgs.CalibratedInstallationParameters.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.applanix_msgs.CalibratedInstallationParameters(strObj);
        end
    end
end