
classdef SyncServiceInfo < ros.Message
    %SyncServiceInfo MATLAB implementation of fkie_multimaster_msgs/SyncServiceInfo
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'fkie_multimaster_msgs/SyncServiceInfo' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '8c21bb9ea24403924441840b8c167c40' % The MD5 Checksum of the message definition
        PropertyList = { 'Service' 'Serviceuri' 'Node' 'Nodeuri' } % List of non-constant message properties
        ROSPropertyList = { 'service' 'serviceuri' 'node' 'nodeuri' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Service
        Serviceuri
        Node
        Nodeuri
    end
    methods
        function set.Service(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SyncServiceInfo', 'Service');
            obj.Service = char(val);
        end
        function set.Serviceuri(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SyncServiceInfo', 'Serviceuri');
            obj.Serviceuri = char(val);
        end
        function set.Node(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SyncServiceInfo', 'Node');
            obj.Node = char(val);
        end
        function set.Nodeuri(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SyncServiceInfo', 'Nodeuri');
            obj.Nodeuri = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.fkie_multimaster_msgs.SyncServiceInfo.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.fkie_multimaster_msgs.SyncServiceInfo(strObj);
        end
    end
end