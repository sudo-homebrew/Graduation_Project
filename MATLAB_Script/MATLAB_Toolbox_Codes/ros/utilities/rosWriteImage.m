function msg = rosWriteImage(msg, varargin)
%rosWriteImage Write a MATLAB image to a ROS/ROS 2 image message
%   MSG = rosWriteImage(MSG,IMG) converts the MATLAB image IMG and
%   stores the ROS compatible image data in the message MSG.
%
%   MSG = rosWriteImage(MSG,IMG,ALPHA) converts the MATLAB image IMG.
%   If the image encoding supports an alpha channel (rgba
%   or bgra family), this alpha channel can be specified as optional
%   input ALPHA. Alternatively, the input image IMG can also
%   store the alpha channel as its fourth channel.
%
%   MSG = rosWriteImage(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments. You can specify
%   several name-value pair arguments in any order as Name1, Value1, ...,
%   NameN,valueN:
%
%      "Encoding" - Optional parameter for setting up the encoding of the
%                   input image message. If the encoding field is not set
%                   in the input image message, you can use this parameter
%                   to manually set the encoding information.
%
%   All encodings supported for reading are also supported by
%   the writeImage function. For more information on supported
%   encodings and their representation in MATLAB, see the rosReadImage
%   function.
%
%   Bayer-encoded images (encodings 'bayer_rggb8',
%   'bayer_bggr8', 'bayer_gbrg8', 'bayer_grbg8' and their
%   16-bit equivalents) will not be Bayer-encoded but have to
%   be given as 8-bit or 16-bit single-channel images
%
%   Example:
%       % Create a random image
%       img = uint8(10*rand(128,128,3));
%
%       % Create a sensor_msgs/Image message
%       msg = rosmessage("sensor_msgs/Image","DataFormat","struct");
%
%       % Write the img data to the msg
%       msg = rosWriteImage(msg,img,"Encoding","rgb8");
%
%   See also: ROSREADIMAGE.

%   Copyright 2020 The MathWorks, Inc.
%#codegen

% Validate Input argument and create local fields for ROS or ROS 2
% sensor_msgs/Image

% Ensure the generated code is not inlined
    coder.inline('never');

    validateattributes(msg, {'struct'},{'scalar'},'rosWriteImage');

    % Code generation only supports sensor_msgs/Image as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'sensor_msgs/Image'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosWriteImage','sensor_msgs/Image');
    end

    if isfield(msg, 'Data')
        % ROS message struct
        specialMsgUtil = ros.internal.SpecialMsgUtil;
    else
        % ROS 2 message struct
        specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
    end
    msg = specialMsgUtil.writeImage(msg, varargin{:});
end
