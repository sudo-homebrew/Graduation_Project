function xyz = rosReadXYZ(msg, varargin)
%rosReadXYZ Returns the (x,y,z) coordinates from ROS/ROS 2 message struct
%   XYZ = rosReadXYZ(MSG) extracts the (x,y,z) coordinates from
%   all points in the point cloud message MSG and returns them
%   as an Nx3 or HxWx3 matrix of 3D point
%   coordinates. Each 3-vector represents one 3D point.
%
%   XYZ = rosReadXYZ(___,Name,Value) provides additional options
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
%   If the point cloud contains N points, the returned matrix
%   has Nx3 elements (or a HxWx3 matrix if the PreserveStructure
%   property is set to true).
%
%   If the point cloud does not contain the 'x', 'y' and 'z'
%   fields, this function will display an error.
%
%
%   Example:
%
%      % Retrieve 3D point coordinates
%      xyz = rosReadXYZ(ptCloud);
%
%      % Plot the points with a 3D scatter plot
%      scatter3(xyz(:,1),xyz(:,2),xyz(:,3),'.');

%   Copyright 2020 The MathWorks, Inc.
%#codegen

% Ensure the generated code is not inlined
    coder.inline('never');

    % Validate Input argument
    validateattributes(msg,{'struct'},{'scalar'},'rosReadXYZ');

    % Code generation only supports sensor_msgs/Image as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'sensor_msgs/PointCloud2'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosReadXYZ','sensor_msgs/PointCloud2');
    end

    % Get xyz from input message
    if isfield(msg, 'Data')
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    xyz = specialMsgUtil.readXYZ(msg, varargin{:});
end
