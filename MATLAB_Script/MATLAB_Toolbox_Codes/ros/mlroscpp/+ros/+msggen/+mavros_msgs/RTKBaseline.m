
classdef RTKBaseline < ros.Message
    %RTKBaseline MATLAB implementation of mavros_msgs/RTKBaseline
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'mavros_msgs/RTKBaseline' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'bd5852b76aa13136cec34a65089dfdb2' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'TimeLastBaselineMs' 'RtkReceiverId' 'Wn' 'Tow' 'RtkHealth' 'RtkRate' 'Nsats' 'BaselineCoordsType' 'BaselineAMm' 'BaselineBMm' 'BaselineCMm' 'Accuracy' 'IarNumHypotheses' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'time_last_baseline_ms' 'rtk_receiver_id' 'wn' 'tow' 'rtk_health' 'rtk_rate' 'nsats' 'baseline_coords_type' 'baseline_a_mm' 'baseline_b_mm' 'baseline_c_mm' 'accuracy' 'iar_num_hypotheses' } % List of non-constant ROS message properties
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
            } % Types of contained nested messages
    end
    properties (Constant)
        RTKBASELINECOORDINATESYSTEMECEF = uint8(0)
        RTKBASELINECOORDINATESYSTEMNED = uint8(1)
    end
    properties
        Header
        TimeLastBaselineMs
        RtkReceiverId
        Wn
        Tow
        RtkHealth
        RtkRate
        Nsats
        BaselineCoordsType
        BaselineAMm
        BaselineBMm
        BaselineCMm
        Accuracy
        IarNumHypotheses
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'Header')
            obj.Header = val;
        end
        function set.TimeLastBaselineMs(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'TimeLastBaselineMs');
            obj.TimeLastBaselineMs = uint32(val);
        end
        function set.RtkReceiverId(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'RtkReceiverId');
            obj.RtkReceiverId = uint8(val);
        end
        function set.Wn(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'Wn');
            obj.Wn = uint16(val);
        end
        function set.Tow(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'Tow');
            obj.Tow = uint32(val);
        end
        function set.RtkHealth(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'RtkHealth');
            obj.RtkHealth = uint8(val);
        end
        function set.RtkRate(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'RtkRate');
            obj.RtkRate = uint8(val);
        end
        function set.Nsats(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'Nsats');
            obj.Nsats = uint8(val);
        end
        function set.BaselineCoordsType(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'BaselineCoordsType');
            obj.BaselineCoordsType = uint8(val);
        end
        function set.BaselineAMm(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'BaselineAMm');
            obj.BaselineAMm = int32(val);
        end
        function set.BaselineBMm(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'BaselineBMm');
            obj.BaselineBMm = int32(val);
        end
        function set.BaselineCMm(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'BaselineCMm');
            obj.BaselineCMm = int32(val);
        end
        function set.Accuracy(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'Accuracy');
            obj.Accuracy = uint32(val);
        end
        function set.IarNumHypotheses(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'RTKBaseline', 'IarNumHypotheses');
            obj.IarNumHypotheses = int32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.mavros_msgs.RTKBaseline.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.mavros_msgs.RTKBaseline(strObj);
        end
    end
end