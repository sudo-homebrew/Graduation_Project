function lidarScanObj = rosReadLidarScan(msg)
%rosReadLidarScan Return an lidar scan object from ROS/ROS 2 message struct
%   LIDARSCANOBJ = rosReadLidarScan(MSG) Reads the Cartesian output of the
%   laser scan msg and creates a lidarScan object.
%   The ranges and angles in the lidarScan object will be
%   stored in double-precision.
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
%       % Get a lidarScan object out of ROS LaserScan msg
%       scan = rosReadLidarScan(msg);
%
%       % Perform few operations on the lidarScan object
%       minRange = 0.1;
%       maxRange = 8.0;
%       validScan = removeInvalidData(scan,"RangeLimits",[minRange, maxRange])
%
%       % Visualize scan
%       figure
%       plot(validScan);

%   Copyright 2020 The MathWorks, Inc.
%#codegen

% Ensure the generated code is not inlined
    coder.inline('never');

    % Validate Input argument
    validateattributes(msg, {'struct'},{'scalar'},'rosReadLidarScan');

    % Code generation only supports sensor_msgs/Image as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'sensor_msgs/LaserScan'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosReadLidarScan','sensor_msgs/LaserScan');

        % Code generation does not support empty message struct
        if isfield(msg,'Ranges')
            coder.internal.assert(~isempty(msg.Ranges),...
                                  'ros:mlroscpp:codegen:EmptyInputMsg','rosReadLidarScan');
        else
            coder.internal.assert(~isequal(msg.ranges,0),...
                                  'ros:mlroscpp:codegen:EmptyInputMsg','rosReadLidarScan');
        end
    end

    % Get lidarScan from input message
    if isfield(msg, 'Ranges')
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    lidarScanObj = specialMsgUtil.readLidarScan(msg);
end
