
classdef Implementation < ros.Message
    %Implementation MATLAB implementation of concert_msgs/Implementation
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'concert_msgs/Implementation' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'aeb0655c516d030025a8fe13f0998166' % The MD5 Checksum of the message definition
        PropertyList = { 'LinkGraph' 'Name' 'DotGraph' } % List of non-constant message properties
        ROSPropertyList = { 'link_graph' 'name' 'dot_graph' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.concert_msgs.LinkGraph' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        LinkGraph
        Name
        DotGraph
    end
    methods
        function set.LinkGraph(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.concert_msgs.LinkGraph'};
            validateattributes(val, validClasses, validAttributes, 'Implementation', 'LinkGraph')
            obj.LinkGraph = val;
        end
        function set.Name(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Implementation', 'Name');
            obj.Name = char(val);
        end
        function set.DotGraph(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Implementation', 'DotGraph');
            obj.DotGraph = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.concert_msgs.Implementation.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.concert_msgs.Implementation(strObj);
        end
    end
end
