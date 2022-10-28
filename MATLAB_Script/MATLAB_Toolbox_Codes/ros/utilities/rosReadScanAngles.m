function angles = rosReadScanAngles(msg)
%rosReadScanAngles Return the scan angles from ROS/ROS 2 message struct
%   ANGLES = rosReadScanAngles(MSG) calculates the scan angles,
%   ANGLES, corresponding to the range readings in the message MSG.
%
%   Angles are measured counter-clockwise around the positive z
%   axis, with the zero angle forward along the x axis. The
%   angles will be returned in radians and are wrapped to the
%   (-pi,pi] interval.
%
%   ANGLES will be returned as an N-by-1 vector, where N is
%   equal to the number of range readings.

%   The message specifies the scanning angles through the
%   AngleMin, AngleMax and AngleIncrement properties. For some
%   sensor messages, these could be in conflict. For example,
%   commonly only the AngleMin and AngleIncrement properties
%   are used to generate the angle vector. See
%   https://docs.ros.org/en/melodic/api/laser_geometry/html/laser__geometry_8cpp_source.html
%   for the ROS implementation. Alternatively, scan angles could be calculated as a
%   linearly spaced vector between AngleMin and AngleMax. The
%   results of the two different ways of calculating the scan angles
%   might differ, but based on existing precedents we adhere to
%   the former convention.
%
%   Example:
%       % Create a LaserScan message
%       msg = rosmessage("sensor_msgs/LaserScan","DataFormat","struct");
%       msg.AngleMin = -0.5467;
%       msg.AngleMax = 0.5467;
%       msg.AngleIncrement = 0.0017;
%       msg.RangeMin = 0.45;
%       msg.RangeMax = 10;
%       msg.Ranges = single(linspace(0.45,10,640)');
%
%       % Retrive scan angles from LaserScan message
%       angles = rosReadScanAngles(msg);

%   Copyright 2020 The MathWorks, Inc.

% Extract field names from input message
%#codegen

% Ensure the generated code is not inlined
    coder.inline('never');

    % Validate Input argument
    validateattributes(msg, {'struct'},{'scalar'},'rosReadScanAngles');

    % Code generation only supports sensor_msgs/Image as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'sensor_msgs/LaserScan'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosReadScanAngles','sensor_msgs/LaserScan');

        % Code generation does not support empty message struct
        if isfield(msg,'Ranges')
            coder.internal.assert(~isempty(msg.Ranges),...
                                  'ros:mlroscpp:codegen:EmptyInputMsg','rosReadScanAngles');
        else
            coder.internal.assert(~isequal(msg.ranges,0),...
                                  'ros:mlroscpp:codegen:EmptyInputMsg','rosReadScanAngles');
        end
    end

    % Get scan angles from input message
    if isfield(msg, 'Ranges')
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    angles = specialMsgUtil.readScanAngles(msg);
end
