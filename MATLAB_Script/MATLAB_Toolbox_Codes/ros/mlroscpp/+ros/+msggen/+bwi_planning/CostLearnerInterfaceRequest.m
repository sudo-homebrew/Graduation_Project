
classdef CostLearnerInterfaceRequest < ros.Message
    %CostLearnerInterfaceRequest MATLAB implementation of bwi_planning/CostLearnerInterfaceRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'bwi_planning/CostLearnerInterfaceRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '627a1a3de8dd9f91e561a53c602eba1d' % The MD5 Checksum of the message definition
        PropertyList = { 'Location' 'DoorFrom' 'DoorTo' 'Cost' } % List of non-constant message properties
        ROSPropertyList = { 'location' 'door_from' 'door_to' 'cost' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Location
        DoorFrom
        DoorTo
        Cost
    end
    methods
        function set.Location(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'CostLearnerInterfaceRequest', 'Location');
            obj.Location = char(val);
        end
        function set.DoorFrom(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'CostLearnerInterfaceRequest', 'DoorFrom');
            obj.DoorFrom = char(val);
        end
        function set.DoorTo(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'CostLearnerInterfaceRequest', 'DoorTo');
            obj.DoorTo = char(val);
        end
        function set.Cost(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'CostLearnerInterfaceRequest', 'Cost');
            obj.Cost = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.bwi_planning.CostLearnerInterfaceRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.bwi_planning.CostLearnerInterfaceRequest(strObj);
        end
    end
end