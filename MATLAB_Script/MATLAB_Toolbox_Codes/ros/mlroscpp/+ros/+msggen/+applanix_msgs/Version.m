
classdef Version < ros.Message
    %Version MATLAB implementation of applanix_msgs/Version
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'applanix_msgs/Version' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '436db5a21d31ed873f1f24270392aeca' % The MD5 Checksum of the message definition
        PropertyList = { 'Td' 'SystemVersion' 'PrimaryGnssVersion' 'SecondaryGnssVersion' 'TotalHours' 'NumRuns' 'AvgRunLength' 'LongestRun' 'CurrentRun' } % List of non-constant message properties
        ROSPropertyList = { 'td' 'system_version' 'primary_gnss_version' 'secondary_gnss_version' 'total_hours' 'num_runs' 'avg_run_length' 'longest_run' 'current_run' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.applanix_msgs.TimeDistance' ...
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
        Td
        SystemVersion
        PrimaryGnssVersion
        SecondaryGnssVersion
        TotalHours
        NumRuns
        AvgRunLength
        LongestRun
        CurrentRun
    end
    methods
        function set.Td(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.applanix_msgs.TimeDistance'};
            validateattributes(val, validClasses, validAttributes, 'Version', 'Td')
            obj.Td = val;
        end
        function set.SystemVersion(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 120};
            validateattributes(val, validClasses, validAttributes, 'Version', 'SystemVersion');
            obj.SystemVersion = uint8(val);
        end
        function set.PrimaryGnssVersion(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 80};
            validateattributes(val, validClasses, validAttributes, 'Version', 'PrimaryGnssVersion');
            obj.PrimaryGnssVersion = uint8(val);
        end
        function set.SecondaryGnssVersion(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 80};
            validateattributes(val, validClasses, validAttributes, 'Version', 'SecondaryGnssVersion');
            obj.SecondaryGnssVersion = uint8(val);
        end
        function set.TotalHours(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Version', 'TotalHours');
            obj.TotalHours = single(val);
        end
        function set.NumRuns(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Version', 'NumRuns');
            obj.NumRuns = uint32(val);
        end
        function set.AvgRunLength(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Version', 'AvgRunLength');
            obj.AvgRunLength = single(val);
        end
        function set.LongestRun(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Version', 'LongestRun');
            obj.LongestRun = single(val);
        end
        function set.CurrentRun(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Version', 'CurrentRun');
            obj.CurrentRun = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.applanix_msgs.Version.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.applanix_msgs.Version(strObj);
        end
    end
end
