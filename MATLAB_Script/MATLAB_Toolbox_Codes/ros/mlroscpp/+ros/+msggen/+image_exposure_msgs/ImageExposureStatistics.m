
classdef ImageExposureStatistics < ros.Message
    %ImageExposureStatistics MATLAB implementation of image_exposure_msgs/ImageExposureStatistics
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'image_exposure_msgs/ImageExposureStatistics' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '334dc170ce6287d1de470f25be78dd9e' % The MD5 Checksum of the message definition
        PropertyList = { 'Stamp' 'PixelVal' 'PixelAge' 'TimeReference' 'Shutterms' 'Gaindb' 'UnderExposed' 'OverExposed' 'MeanIrradiance' 'MinMeasurableIrradiance' 'MaxMeasurableIrradiance' 'MinObservedIrradiance' 'MaxObservedIrradiance' } % List of non-constant message properties
        ROSPropertyList = { 'stamp' 'pixelVal' 'pixelAge' 'time_reference' 'shutterms' 'gaindb' 'underExposed' 'overExposed' 'meanIrradiance' 'minMeasurableIrradiance' 'maxMeasurableIrradiance' 'minObservedIrradiance' 'maxObservedIrradiance' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msg.Time' ...
            'ros.msggen.statistics_msgs.Stats1D' ...
            'ros.msggen.statistics_msgs.Stats1D' ...
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
        Stamp
        PixelVal
        PixelAge
        TimeReference
        Shutterms
        Gaindb
        UnderExposed
        OverExposed
        MeanIrradiance
        MinMeasurableIrradiance
        MaxMeasurableIrradiance
        MinObservedIrradiance
        MaxObservedIrradiance
    end
    methods
        function set.Stamp(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Time'};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'Stamp')
            obj.Stamp = val;
        end
        function set.PixelVal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.statistics_msgs.Stats1D'};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'PixelVal')
            obj.PixelVal = val;
        end
        function set.PixelAge(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.statistics_msgs.Stats1D'};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'PixelAge')
            obj.PixelAge = val;
        end
        function set.TimeReference(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'TimeReference');
            obj.TimeReference = char(val);
        end
        function set.Shutterms(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'Shutterms');
            obj.Shutterms = single(val);
        end
        function set.Gaindb(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'Gaindb');
            obj.Gaindb = single(val);
        end
        function set.UnderExposed(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'UnderExposed');
            obj.UnderExposed = uint32(val);
        end
        function set.OverExposed(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'OverExposed');
            obj.OverExposed = uint32(val);
        end
        function set.MeanIrradiance(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'MeanIrradiance');
            obj.MeanIrradiance = double(val);
        end
        function set.MinMeasurableIrradiance(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'MinMeasurableIrradiance');
            obj.MinMeasurableIrradiance = double(val);
        end
        function set.MaxMeasurableIrradiance(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'MaxMeasurableIrradiance');
            obj.MaxMeasurableIrradiance = double(val);
        end
        function set.MinObservedIrradiance(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'MinObservedIrradiance');
            obj.MinObservedIrradiance = double(val);
        end
        function set.MaxObservedIrradiance(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'ImageExposureStatistics', 'MaxObservedIrradiance');
            obj.MaxObservedIrradiance = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.image_exposure_msgs.ImageExposureStatistics.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.image_exposure_msgs.ImageExposureStatistics(strObj);
        end
    end
end