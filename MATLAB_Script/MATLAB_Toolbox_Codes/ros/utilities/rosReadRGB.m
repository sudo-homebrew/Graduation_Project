function rgb = rosReadRGB(msg, varargin)
%rosReadRGB Returns the RGB color matrix from ROS/ROS 2 message struct
%   RGB = rosReadRGB(MSG) extracts the (r,g,b) for all points in
%   the point cloud message MSG and returns them as
%   an Nx3 or HxWx3 matrix of RGB color values.
%   Each 3-vector represents one RGB reading.
%
%   RGB = rosReadRGB(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments. You can specify
%   several name-value pair arguments in any order as Name1, Value1, ...,
%   NameN,valueN:
%
%      "PreserveStructureOnRead" - Optional property for preserving the
%                                  organizational structure of the point
%                                  cloud. When you preserve the structure,
%                                  the output matrices are of size
%                                  m-by-n-by-d, where m is the height, n is
%                                  the width, and d is the number of return
%                                  values for each point. Otherwise, all
%                                  points are returned as a x-by-d list.
%                                  Default: false
%
%   If the point cloud contains N points, and color information
%   for the points is stored in the message, the returned matrix
%   has Nx3 elements. An HxWx3 matrix is returned if the PreserveStructure
%   property is set to true.
%
%   This function will display an error if no RGB data is
%   stored in the point cloud.
%
%   Each RGB value is represented as a double in the range of
%   [0,1].
%
%
%   Example:
%      % Retrieve 3D point coordinates
%      xyz = rosReadXYZ(ptCloud);
%
%      % Read corresponding color values
%      rgb = rosReadRGB(ptCloud);

%   Copyright 2020 The MathWorks, Inc.
%#codegen

% Ensure the generated code is not inlined
    coder.inline('never');

    % Validate Input argument
    validateattributes(msg,{'struct'},{'scalar'},'rosReadRGB');

    % Code generation only supports sensor_msgs/Image as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'sensor_msgs/PointCloud2'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosReadRGB','sensor_msgs/PointCloud2');
    end

    % Get rgb from input message
    if isfield(msg, 'Data')
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    rgb = specialMsgUtil.readRGB(msg, varargin{:});
end
