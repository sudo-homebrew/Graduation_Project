
classdef RecognizedObjectArray < ros.Message
    %RecognizedObjectArray MATLAB implementation of object_recognition_msgs/RecognizedObjectArray
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'object_recognition_msgs/RecognizedObjectArray' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'bad6b1546b9ebcabb49fb3b858d78964' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Objects' 'Cooccurrence' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'objects' 'cooccurrence' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.object_recognition_msgs.RecognizedObject' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Objects
        Cooccurrence
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'RecognizedObjectArray', 'Header')
            obj.Header = val;
        end
        function set.Objects(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.object_recognition_msgs.RecognizedObject.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.object_recognition_msgs.RecognizedObject'};
            validateattributes(val, validClasses, validAttributes, 'RecognizedObjectArray', 'Objects')
            obj.Objects = val;
        end
        function set.Cooccurrence(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = single.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'RecognizedObjectArray', 'Cooccurrence');
            obj.Cooccurrence = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.object_recognition_msgs.RecognizedObjectArray.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.object_recognition_msgs.RecognizedObjectArray(strObj);
        end
    end
end