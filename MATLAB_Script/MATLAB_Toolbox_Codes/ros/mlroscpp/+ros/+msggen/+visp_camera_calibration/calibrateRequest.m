
classdef calibrateRequest < ros.Message
    %calibrateRequest MATLAB implementation of visp_camera_calibration/calibrateRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'visp_camera_calibration/calibrateRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '7b57c459896a8f1f8df45a385acfc123' % The MD5 Checksum of the message definition
        PropertyList = { 'Method' 'SampleWidth' 'SampleHeight' } % List of non-constant message properties
        ROSPropertyList = { 'method' 'sample_width' 'sample_height' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Method
        SampleWidth
        SampleHeight
    end
    methods
        function set.Method(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'calibrateRequest', 'Method');
            obj.Method = int32(val);
        end
        function set.SampleWidth(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'calibrateRequest', 'SampleWidth');
            obj.SampleWidth = int32(val);
        end
        function set.SampleHeight(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'calibrateRequest', 'SampleHeight');
            obj.SampleHeight = int32(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.visp_camera_calibration.calibrateRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.visp_camera_calibration.calibrateRequest(strObj);
        end
    end
end