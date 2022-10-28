classdef ScanAnglesReader
%This class is for internal use only. It may be removed in the future.

%   Copyright 2021 The MathWorks, Inc.

%ScanAnglesReader Class for converting ROS LaserScan data to a scan angle
% array
%   See also: ros.msg.sensor_msgs.LaserScan


%#codegen

    methods (Static)

        function angles = readScanAngles(ranges, angleMin, angleIncrement)
        % readScan Convert the ROS scan data into a array of scan angles
        %   See ros.msg.sensor_msgs.LaserScan.readScanAngles for
        %   details

            numReadings = numel(ranges);
            rawAngles = angleMin + (0:numReadings-1)' * angleIncrement;
            % Wrap the angles to the (-pi,pi] interval.
            angles = robotics.internal.wrapToPi(double(rawAngles));
        end
    end
end
