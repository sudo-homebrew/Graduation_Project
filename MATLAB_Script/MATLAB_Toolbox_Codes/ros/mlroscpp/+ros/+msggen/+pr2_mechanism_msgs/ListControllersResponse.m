
classdef ListControllersResponse < ros.Message
    %ListControllersResponse MATLAB implementation of pr2_mechanism_msgs/ListControllersResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'pr2_mechanism_msgs/ListControllersResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '39c8d39516aed5c7d76284ac06c220e5' % The MD5 Checksum of the message definition
        PropertyList = { 'Controllers' 'State' } % List of non-constant message properties
        ROSPropertyList = { 'controllers' 'state' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Controllers
        State
    end
    methods
        function set.Controllers(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'ListControllersResponse', 'Controllers');
            obj.Controllers = cell(val);
        end
        function set.State(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'ListControllersResponse', 'State');
            obj.State = cell(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.pr2_mechanism_msgs.ListControllersResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.pr2_mechanism_msgs.ListControllersResponse(strObj);
        end
    end
end