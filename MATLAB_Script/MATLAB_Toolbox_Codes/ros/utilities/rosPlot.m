function plotHandles = rosPlot(msg, varargin)
%rosPlot Plot LiDAR or point cloud from ROS/ROS 2 message struct
%   PLOTHANDLE = rosPlot(MSG) creates a point plot of the scan in the
%   current axes (or creates a new one). The scan is
%   shown in Cartesian (XY) coordinates and the axes are
%   automatically scaled to the maximum range that the laser
%   scanner supports.
%
%   The plot is oriented to follow the standard ROS convention of: x
%   axis forward and y axis left.
%
%   PLOTHANDLE = rosPlot(___,Name,Value) allows the specification of
%   optional name/value pairs to control the plotting.
%   Potential name/value pairs are:
%      "Parent"       -  specifies the parent axes in which the laser scan
%                        should be drawn. By default, the laser scan will be
%                        plotted in the currently active axes.
%      "MaximumRange" -  sets the maximum plot range to a fixed value (in meters).
%                        This will set the minimum and maximum X axis limits and
%                        the maximum Y axis limit to the specified value. The
%                        minimum Y axis limit is automatically determined by the
%                        opening angle of the laser scanner. You can use this
%                        name-value pair if the automatic scaling does not give the
%                        desired result.
%                        Default: Value of RangeMax message property
%
%   PLOTHANDLES = rosPlot(___) returns a column vector of lineseries
%   handles, PLOTHANDLES, for the laser scan. Use PLOTHANDLES to
%   modify properties of the lineseries after it is created.
%
%   This function will plot the Cartesian (XY) coordinates of the
%   laser scan readings and scale the plot to fit the data.
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
%       % Plot the LaserScan with rosPlot
%       rosPlot(msg);

%   Copyright 2020 The MathWorks, Inc.

% Validate input argument
    validateattributes(msg,{'struct'},{'scalar'},'rosPlot');

    % plot scatter if the input message is a PointCloud2 message
    if(strcmp(msg.MessageType,'sensor_msgs/PointCloud2'))
        if isfield(msg, 'Data')
            % ROS message struct
            specialMsgUtil = ros.internal.SpecialMsgUtil;
        else
            % ROS 2 message struct
            specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
        end
        returnedPlot = specialMsgUtil.plotPointCloud2(msg, varargin{:});
        % create a point plot if the input message is a LaserScan message
    elseif(strcmp(msg.MessageType,'sensor_msgs/LaserScan'))
        if isfield(msg, 'Ranges')
            % ROS message struct
            specialMsgUtil = ros.internal.SpecialMsgUtil;
        else
            % ROS 2 message struct
            specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
        end
        returnedPlot = specialMsgUtil.plotLaserScan(msg,varargin{:});
    else
        error(message('ros:utilities:message:MessageTypeNotSupportedError'));
    end

    % Return the scatter handle if requested by the user
    if nargout > 0
        plotHandles = returnedPlot;
    end
end
