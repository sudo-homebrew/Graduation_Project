
classdef SpeechRecognitionRequest < ros.Message
    %SpeechRecognitionRequest MATLAB implementation of speech_recognition_msgs/SpeechRecognitionRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'speech_recognition_msgs/SpeechRecognitionRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'af5602408bd36e4d9a80cde6f4453023' % The MD5 Checksum of the message definition
        PropertyList = { 'Vocabulary' 'Grammar' 'GrammarName' 'Duration' 'Quiet' 'Threshold' } % List of non-constant message properties
        ROSPropertyList = { 'vocabulary' 'grammar' 'grammar_name' 'duration' 'quiet' 'threshold' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.speech_recognition_msgs.Vocabulary' ...
            'ros.msggen.speech_recognition_msgs.Grammar' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Vocabulary
        Grammar
        GrammarName
        Duration
        Quiet
        Threshold
    end
    methods
        function set.Vocabulary(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.speech_recognition_msgs.Vocabulary'};
            validateattributes(val, validClasses, validAttributes, 'SpeechRecognitionRequest', 'Vocabulary')
            obj.Vocabulary = val;
        end
        function set.Grammar(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.speech_recognition_msgs.Grammar'};
            validateattributes(val, validClasses, validAttributes, 'SpeechRecognitionRequest', 'Grammar')
            obj.Grammar = val;
        end
        function set.GrammarName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SpeechRecognitionRequest', 'GrammarName');
            obj.GrammarName = char(val);
        end
        function set.Duration(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SpeechRecognitionRequest', 'Duration');
            obj.Duration = single(val);
        end
        function set.Quiet(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SpeechRecognitionRequest', 'Quiet');
            obj.Quiet = logical(val);
        end
        function set.Threshold(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'SpeechRecognitionRequest', 'Threshold');
            obj.Threshold = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.speech_recognition_msgs.SpeechRecognitionRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.speech_recognition_msgs.SpeechRecognitionRequest(strObj);
        end
    end
end