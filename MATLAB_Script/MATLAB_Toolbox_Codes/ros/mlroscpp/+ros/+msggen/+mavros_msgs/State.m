
classdef State < ros.Message
    %State MATLAB implementation of mavros_msgs/State
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'mavros_msgs/State' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '65cd0a9fff993b062b91e354554ec7e9' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Connected' 'Armed' 'Guided' 'ManualInput' 'Mode' 'SystemStatus' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'connected' 'armed' 'guided' 'manual_input' 'mode' 'system_status' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        MODEAPMPLANEMANUAL = 'MANUAL';
        MODEAPMPLANECIRCLE = 'CIRCLE';
        MODEAPMPLANESTABILIZE = 'STABILIZE';
        MODEAPMPLANETRAINING = 'TRAINING';
        MODEAPMPLANEACRO = 'ACRO';
        MODEAPMPLANEFBWA = 'FBWA';
        MODEAPMPLANEFBWB = 'FBWB';
        MODEAPMPLANECRUISE = 'CRUISE';
        MODEAPMPLANEAUTOTUNE = 'AUTOTUNE';
        MODEAPMPLANEAUTO = 'AUTO';
        MODEAPMPLANERTL = 'RTL';
        MODEAPMPLANELOITER = 'LOITER';
        MODEAPMPLANELAND = 'LAND';
        MODEAPMPLANEGUIDED = 'GUIDED';
        MODEAPMPLANEINITIALISING = 'INITIALISING';
        MODEAPMPLANEQSTABILIZE = 'QSTABILIZE';
        MODEAPMPLANEQHOVER = 'QHOVER';
        MODEAPMPLANEQLOITER = 'QLOITER';
        MODEAPMPLANEQLAND = 'QLAND';
        MODEAPMPLANEQRTL = 'QRTL';
        MODEAPMCOPTERSTABILIZE = 'STABILIZE';
        MODEAPMCOPTERACRO = 'ACRO';
        MODEAPMCOPTERALTHOLD = 'ALT_HOLD';
        MODEAPMCOPTERAUTO = 'AUTO';
        MODEAPMCOPTERGUIDED = 'GUIDED';
        MODEAPMCOPTERLOITER = 'LOITER';
        MODEAPMCOPTERRTL = 'RTL';
        MODEAPMCOPTERCIRCLE = 'CIRCLE';
        MODEAPMCOPTERPOSITION = 'POSITION';
        MODEAPMCOPTERLAND = 'LAND';
        MODEAPMCOPTEROFLOITER = 'OF_LOITER';
        MODEAPMCOPTERDRIFT = 'DRIFT';
        MODEAPMCOPTERSPORT = 'SPORT';
        MODEAPMCOPTERFLIP = 'FLIP';
        MODEAPMCOPTERAUTOTUNE = 'AUTOTUNE';
        MODEAPMCOPTERPOSHOLD = 'POSHOLD';
        MODEAPMCOPTERBRAKE = 'BRAKE';
        MODEAPMCOPTERTHROW = 'THROW';
        MODEAPMCOPTERAVOIDADSB = 'AVOID_ADSB';
        MODEAPMCOPTERGUIDEDNOGPS = 'GUIDED_NOGPS';
        MODEAPMROVERMANUAL = 'MANUAL';
        MODEAPMROVERLEARNING = 'LEARNING';
        MODEAPMROVERSTEERING = 'STEERING';
        MODEAPMROVERHOLD = 'HOLD';
        MODEAPMROVERAUTO = 'AUTO';
        MODEAPMROVERRTL = 'RTL';
        MODEAPMROVERGUIDED = 'GUIDED';
        MODEAPMROVERINITIALISING = 'INITIALISING';
        MODEPX4MANUAL = 'MANUAL';
        MODEPX4ACRO = 'ACRO';
        MODEPX4ALTITUDE = 'ALTCTL';
        MODEPX4POSITION = 'POSCTL';
        MODEPX4OFFBOARD = 'OFFBOARD';
        MODEPX4STABILIZED = 'STABILIZED';
        MODEPX4RATTITUDE = 'RATTITUDE';
        MODEPX4MISSION = 'AUTO.MISSION';
        MODEPX4LOITER = 'AUTO.LOITER';
        MODEPX4RTL = 'AUTO.RTL';
        MODEPX4LAND = 'AUTO.LAND';
        MODEPX4RTGS = 'AUTO.RTGS';
        MODEPX4READY = 'AUTO.READY';
        MODEPX4TAKEOFF = 'AUTO.TAKEOFF';
    end
    properties
        Header
        Connected
        Armed
        Guided
        ManualInput
        Mode
        SystemStatus
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'State', 'Header')
            obj.Header = val;
        end
        function set.Connected(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'State', 'Connected');
            obj.Connected = logical(val);
        end
        function set.Armed(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'State', 'Armed');
            obj.Armed = logical(val);
        end
        function set.Guided(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'State', 'Guided');
            obj.Guided = logical(val);
        end
        function set.ManualInput(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'State', 'ManualInput');
            obj.ManualInput = logical(val);
        end
        function set.Mode(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'State', 'Mode');
            obj.Mode = char(val);
        end
        function set.SystemStatus(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'State', 'SystemStatus');
            obj.SystemStatus = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.mavros_msgs.State.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.mavros_msgs.State(strObj);
        end
    end
end