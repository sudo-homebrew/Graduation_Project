
classdef navdata_raw_measures < ros.Message
    %navdata_raw_measures MATLAB implementation of ardrone_autonomy/navdata_raw_measures
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'ardrone_autonomy/navdata_raw_measures' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '4da7145c7478d1eb84be4d5fa4acd9ca' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'DroneTime' 'Tag' 'Size' 'RawGyros' 'RawGyros110' 'VbatRaw' 'UsDebutEcho' 'UsFinEcho' 'UsAssociationEcho' 'UsDistanceEcho' 'UsCourbeTemps' 'UsCourbeValeur' 'UsCourbeRef' 'FlagEchoIni' 'NbEcho' 'SumEcho' 'AltTempRaw' 'Gradient' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'drone_time' 'tag' 'size' 'raw_gyros' 'raw_gyros_110' 'vbat_raw' 'us_debut_echo' 'us_fin_echo' 'us_association_echo' 'us_distance_echo' 'us_courbe_temps' 'us_courbe_valeur' 'us_courbe_ref' 'flag_echo_ini' 'nb_echo' 'sum_echo' 'alt_temp_raw' 'gradient' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
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
        Header
        DroneTime
        Tag
        Size
        RawGyros
        RawGyros110
        VbatRaw
        UsDebutEcho
        UsFinEcho
        UsAssociationEcho
        UsDistanceEcho
        UsCourbeTemps
        UsCourbeValeur
        UsCourbeRef
        FlagEchoIni
        NbEcho
        SumEcho
        AltTempRaw
        Gradient
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'Header')
            obj.Header = val;
        end
        function set.DroneTime(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'DroneTime');
            obj.DroneTime = double(val);
        end
        function set.Tag(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'Tag');
            obj.Tag = uint16(val);
        end
        function set.Size(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'Size');
            obj.Size = uint16(val);
        end
        function set.RawGyros(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int16.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'RawGyros');
            obj.RawGyros = int16(val);
        end
        function set.RawGyros110(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int16.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'RawGyros110');
            obj.RawGyros110 = int16(val);
        end
        function set.VbatRaw(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'VbatRaw');
            obj.VbatRaw = uint32(val);
        end
        function set.UsDebutEcho(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'UsDebutEcho');
            obj.UsDebutEcho = uint16(val);
        end
        function set.UsFinEcho(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'UsFinEcho');
            obj.UsFinEcho = uint16(val);
        end
        function set.UsAssociationEcho(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'UsAssociationEcho');
            obj.UsAssociationEcho = uint16(val);
        end
        function set.UsDistanceEcho(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'UsDistanceEcho');
            obj.UsDistanceEcho = uint16(val);
        end
        function set.UsCourbeTemps(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'UsCourbeTemps');
            obj.UsCourbeTemps = uint16(val);
        end
        function set.UsCourbeValeur(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'UsCourbeValeur');
            obj.UsCourbeValeur = uint16(val);
        end
        function set.UsCourbeRef(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'UsCourbeRef');
            obj.UsCourbeRef = uint16(val);
        end
        function set.FlagEchoIni(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'FlagEchoIni');
            obj.FlagEchoIni = uint16(val);
        end
        function set.NbEcho(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'NbEcho');
            obj.NbEcho = uint16(val);
        end
        function set.SumEcho(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'SumEcho');
            obj.SumEcho = uint32(val);
        end
        function set.AltTempRaw(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'AltTempRaw');
            obj.AltTempRaw = int32(val);
        end
        function set.Gradient(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'navdata_raw_measures', 'Gradient');
            obj.Gradient = int16(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.ardrone_autonomy.navdata_raw_measures.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.ardrone_autonomy.navdata_raw_measures(strObj);
        end
    end
end
