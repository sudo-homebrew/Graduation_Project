function fieldNames = rosReadAllFieldNames(msg)
%rosReadAllFieldNames Return all field names from ROS/ROS 2 message struct
%   FIELDNAMES = rosReadAllFieldNames(MSG) returns the names of
%   all point fields that are stored in message struct MSG. FIELDNAMES
%   is a 1xN cell array of strings, where N is the number of fields.
%   If no fields are stored in the message, the return will be
%   an empty cell array.
%
%   Example:
%       % Create point fields for PointCloud2 message
%       xField = rosmessage("sensor_msgs/PointField","DataFormat","struct");
%       xField.Name = 'x';
%       yField = rosmessage("sensor_msgs/PointField","DataFormat","struct");
%       yField.Name = 'y';
%       zField = rosmessage("sensor_msgs/PointField","DataFormat","struct");
%       zField.Name = 'z';
%       rgbField = rosmessage("sensor_msgs/PointField","DataFormat","struct");
%       rgbField.Name = 'rgb';
%
%       % Create PointCloud2 message
%       msg = rosmessage("sensor_msgs/PointCloud2","DataFormat","struct");
%       msg.Fields(1,1) = xField;
%       msg.Fields(2,1) = yField;
%       msg.Fields(3,1) = zField;
%       msg.Fields(4,1) = rgbField;
%
%       % Read all field names from the message
%       fieldNames = rosReadAllFieldNames(msg);
%
%
%   Some of the allowable field names are documented on the ROS
%   Wiki: http://wiki.ros.org/pcl/Overview#Common_PointCloud2_field_names

%   Copyright 2020 The MathWorks, Inc.
%#codegen

% Ensure the generated code is not inlined
    coder.inline('never');

    % Validate Input argument
    validateattributes(msg,{'struct'},{'scalar'},'rosReadAllFieldNames');

    % Code generation only supports sensor_msgs/Image as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'sensor_msgs/PointCloud2'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosReadAllFieldNames','sensor_msgs/PointCloud2');
    end

    % Get all field names from input message
    if isfield(msg, 'Data')
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    fieldNames = specialMsgUtil.getAllFieldNames(msg);
end
