
classdef OpticalFlowRad < ros.Message
    %OpticalFlowRad MATLAB implementation of mavros_msgs/OpticalFlowRad
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'mavros_msgs/OpticalFlowRad' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '65d93e03c6188c7ee30415b2a39ad40d' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'IntegrationTimeUs' 'IntegratedX' 'IntegratedY' 'IntegratedXgyro' 'IntegratedYgyro' 'IntegratedZgyro' 'Temperature' 'Quality' 'TimeDeltaDistanceUs' 'Distance' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'integration_time_us' 'integrated_x' 'integrated_y' 'integrated_xgyro' 'integrated_ygyro' 'integrated_zgyro' 'temperature' 'quality' 'time_delta_distance_us' 'distance' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            '' ...
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
    end
    properties
        Header
        IntegrationTimeUs
        IntegratedX
        IntegratedY
        IntegratedXgyro
        IntegratedYgyro
        IntegratedZgyro
        Temperature
        Quality
        TimeDeltaDistanceUs
        Distance
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'OpticalFlowRad', 'Header')
            obj.Header = val;
        end
        function set.IntegrationTimeUs(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'OpticalFlowRad', 'IntegrationTimeUs');
            obj.IntegrationTimeUs = uint32(val);
        end
        function set.IntegratedX(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'OpticalFlowRad', 'IntegratedX');
            obj.IntegratedX = single(val);
        end
        function set.IntegratedY(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'OpticalFlowRad', 'IntegratedY');
            obj.IntegratedY = single(val);
        end
        function set.IntegratedXgyro(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'OpticalFlowRad', 'IntegratedXgyro');
            obj.IntegratedXgyro = single(val);
        end
        function set.IntegratedYgyro(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'OpticalFlowRad', 'IntegratedYgyro');
            obj.IntegratedYgyro = single(val);
        end
        function set.IntegratedZgyro(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'OpticalFlowRad', 'IntegratedZgyro');
            obj.IntegratedZgyro = single(val);
        end
        function set.Temperature(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'OpticalFlowRad', 'Temperature');
            obj.Temperature = int16(val);
        end
        function set.Quality(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'OpticalFlowRad', 'Quality');
            obj.Quality = uint8(val);
        end
        function set.TimeDeltaDistanceUs(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'OpticalFlowRad', 'TimeDeltaDistanceUs');
            obj.TimeDeltaDistanceUs = uint32(val);
        end
        function set.Distance(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'OpticalFlowRad', 'Distance');
            obj.Distance = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.mavros_msgs.OpticalFlowRad.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.mavros_msgs.OpticalFlowRad(strObj);
        end
    end
end