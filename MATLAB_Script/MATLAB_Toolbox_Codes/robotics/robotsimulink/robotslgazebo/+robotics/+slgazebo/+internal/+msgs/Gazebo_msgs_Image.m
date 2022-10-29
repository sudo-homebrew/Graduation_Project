classdef Gazebo_msgs_Image
%This function is for internal use only. It may be removed in the future.

%GAZEBO_MSGS_IMAGE This is the template for Image bus type

%   Copyright 2019 The MathWorks, Inc.

    properties
        MessageType = 'gazebo_msgs/Image'
        width = uint32(0)
        height = uint32(0)
        data uint8
        data_type = ''
    end
end
