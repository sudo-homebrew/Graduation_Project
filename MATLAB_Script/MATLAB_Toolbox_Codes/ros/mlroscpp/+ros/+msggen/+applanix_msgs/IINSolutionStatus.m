
classdef IINSolutionStatus < ros.Message
    %IINSolutionStatus MATLAB implementation of applanix_msgs/IINSolutionStatus
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'applanix_msgs/IINSolutionStatus' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '421e809231149ab046729933cacbd490' % The MD5 Checksum of the message definition
        PropertyList = { 'Td' 'NumSatellites' 'APrioriPdop' 'BaselineLength' 'Status' 'PrnAssignment' 'L1CycleSlipFlag' 'L2CycleSlipFlag' } % List of non-constant message properties
        ROSPropertyList = { 'td' 'num_satellites' 'a_priori_pdop' 'baseline_length' 'status' 'prn_assignment' 'l1_cycle_slip_flag' 'l2_cycle_slip_flag' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.applanix_msgs.TimeDistance' ...
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
        STATUSFIXEDNARROWLANERTK = uint16(1)
        STATUSFIXEDWIDELANERTK = uint16(2)
        STATUSFLOATRTK = uint16(3)
        STATUSCODEDGPS = uint16(4)
        STATUSRTCMDGPS = uint16(5)
        STATUSAUTONOMOUSCA = uint16(6)
        STATUSGNSSNAVSOLUTION = uint16(7)
        STATUSNOSOLUTION = uint16(8)
    end
    properties
        Td
        NumSatellites
        APrioriPdop
        BaselineLength
        Status
        PrnAssignment
        L1CycleSlipFlag
        L2CycleSlipFlag
    end
    methods
        function set.Td(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.applanix_msgs.TimeDistance'};
            validateattributes(val, validClasses, validAttributes, 'IINSolutionStatus', 'Td')
            obj.Td = val;
        end
        function set.NumSatellites(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'IINSolutionStatus', 'NumSatellites');
            obj.NumSatellites = uint16(val);
        end
        function set.APrioriPdop(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'IINSolutionStatus', 'APrioriPdop');
            obj.APrioriPdop = single(val);
        end
        function set.BaselineLength(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'IINSolutionStatus', 'BaselineLength');
            obj.BaselineLength = single(val);
        end
        function set.Status(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'IINSolutionStatus', 'Status');
            obj.Status = uint16(val);
        end
        function set.PrnAssignment(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 12};
            validateattributes(val, validClasses, validAttributes, 'IINSolutionStatus', 'PrnAssignment');
            obj.PrnAssignment = uint8(val);
        end
        function set.L1CycleSlipFlag(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'IINSolutionStatus', 'L1CycleSlipFlag');
            obj.L1CycleSlipFlag = uint16(val);
        end
        function set.L2CycleSlipFlag(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'IINSolutionStatus', 'L2CycleSlipFlag');
            obj.L2CycleSlipFlag = uint16(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.applanix_msgs.IINSolutionStatus.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.applanix_msgs.IINSolutionStatus(strObj);
        end
    end
end