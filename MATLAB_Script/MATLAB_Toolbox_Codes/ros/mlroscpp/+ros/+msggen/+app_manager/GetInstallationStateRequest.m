
classdef GetInstallationStateRequest < ros.Message
    %GetInstallationStateRequest MATLAB implementation of app_manager/GetInstallationStateRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'app_manager/GetInstallationStateRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'f7e64723808960ca985ba81f45f1b8a7' % The MD5 Checksum of the message definition
        PropertyList = { 'RemoteUpdate' } % List of non-constant message properties
        ROSPropertyList = { 'remote_update' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        RemoteUpdate
    end
    methods
        function set.RemoteUpdate(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GetInstallationStateRequest', 'RemoteUpdate');
            obj.RemoteUpdate = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.app_manager.GetInstallationStateRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.app_manager.GetInstallationStateRequest(strObj);
        end
    end
end