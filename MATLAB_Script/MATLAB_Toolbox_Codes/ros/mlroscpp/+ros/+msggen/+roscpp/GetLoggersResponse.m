
classdef GetLoggersResponse < ros.Message
    %GetLoggersResponse MATLAB implementation of roscpp/GetLoggersResponse
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'roscpp/GetLoggersResponse' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '32e97e85527d4678a8f9279894bb64b0' % The MD5 Checksum of the message definition
        PropertyList = { 'Loggers' } % List of non-constant message properties
        ROSPropertyList = { 'loggers' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.roscpp.Logger' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Loggers
    end
    methods
        function set.Loggers(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.roscpp.Logger.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.roscpp.Logger'};
            validateattributes(val, validClasses, validAttributes, 'GetLoggersResponse', 'Loggers')
            obj.Loggers = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.roscpp.GetLoggersResponse.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.roscpp.GetLoggersResponse(strObj);
        end
    end
end