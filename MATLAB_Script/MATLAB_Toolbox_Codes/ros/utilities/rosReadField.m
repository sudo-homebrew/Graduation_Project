function fieldData = rosReadField(msg, fieldName, varargin)
%rosReadField Read data from ROS/ROS 2 message struct given field name
%   FIELDDATA = rosReadField(MSG,'FIELDNAME') reads the data from
%   the point field with name 'FIELDNAME' and returns it in the
%   FIELDDATA variable. If 'FIELDNAME' does not exist, the
%   function will display an error.
%
%   FIELDDATA = rosReadField(___,Name,Value) provides additional options
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
%   This function returns an NxC vector of values (or a HxWxC
%   matrix if the PreserveStructure property is set to true)
%   N is the number of points in the point cloud and C is the number of
%   values that is assigned for every point in this field. In
%   most cases, C will be 1.
%
%
%   Example:
%      % Create point fields for PointCloud2 message
%      xField = rosmessage("sensor_msgs/PointField","DataFormat","struct");
%      xField.Name = 'x';
%      xField.Datatype = uint8(7);
%      xField.Count = 1;
%      yField = rosmessage("sensor_msgs/PointField","DataFormat","struct");
%      yField.Name = 'y';
%      yField.Datatype = uint8(7);
%      yField.Count = 1;
%      zField = rosmessage("sensor_msgs/PointField","DataFormat","struct");
%      zField.Name = 'z';
%      zField.Datatype = uint8(7);
%      zField.Count = 1;
%      rgbField = rosmessage("sensor_msgs/PointField","DataFormat","struct");
%      rgbField.Name = 'rgb';
%      rgbField.Datatype = uint8(7);
%      rgbField.Count = 1;
%
%      % Create PointCloud2 message
%      msg = rosmessage("sensor_msgs/PointCloud2","DataFormat","struct");
%      msg.Fields(1,1) = xField;
%      msg.Fields(2,1) = yField;
%      msg.Fields(3,1) = zField;
%      msg.Fields(4,1) = rgbField;
%      msg.Height = uint32(480);
%      msg.Width = uint32(640);
%      msg.PointStep = uint32(32);
%      msg.RowStep = uint32(20480);
%      msg.Data = uint8(randi([0 255],9830400,1));
%
%      % Retrieve the X coordinates of all points
%      x = rosReadField(msg,"x");

%   Copyright 2020 The MathWorks, Inc.
%#codegen

% Ensure the generated code is not inlined
    coder.inline('never');

    % Validate Input argument
    validateattributes(msg,{'struct'},{'scalar'},'rosReadField');

    % Code generation only supports sensor_msgs/Image as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'sensor_msgs/PointCloud2'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosReadField','sensor_msgs/PointCloud2');
    end

    % Get specific field from input message
    if isfield(msg, 'Data')
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    fieldData = specialMsgUtil.getPointCloud2Field(msg,fieldName, varargin{:});
end
