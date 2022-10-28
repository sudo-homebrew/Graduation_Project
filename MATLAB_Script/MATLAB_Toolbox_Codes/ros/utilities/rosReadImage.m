function [img, alpha] = rosReadImage(msg, varargin)
%rosReadImage Convert ROS/ROS 2 image message struct into a MATLAB image
%   [IMG,ALPHA] = rosReadImage(MSG) converts the raw image data in
%   the message MSG into an image matrix IMG that is
%   appropriate for further image processing in MATLAB. If the
%   raw image contains an alpha channel, it will be returned in
%   ALPHA. If no alpha channel exists, ALPHA will be empty.
%
%   ROS image message data is stored in a format that is not
%   compatible with further image processing in MATLAB. Based
%   on the specified encoding, this function will convert the
%   data into an appropriate MATLAB image and return it.
%
%   [IMG,ALPHA] = rosReadImage(___,Name,Value) provides additional options
%   specified by one or more Name,Value pair arguments.
%
%      "Encoding" - Parameter for setting up the encoding of the input
%                   image message. This is required only for code
%                   generation workflow.
%
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
%       % Read image from msg
%       imgRead = rosReadImage(msg);
%
%   The following encodings for raw images of size MxN are supported:
%   * rgb8, rgba8, bgr8, and bgra8: IMG is an RGB image
%   of size MxNx3. The alpha channel is returned in ALPHA.
%   Each value in the outputs is represented as a uint8.
%
%   * rgb16, rgba16, bgr16, and bgra16: IMG is an RGB image
%   of size MxNx3. The alpha channel is returned in ALPHA.
%   Each value in the outputs is represented as a uint16.
%
%   * mono8 images are returned as gray-scale images of size
%   MxNx1. Each pixel value is represented as a uint8.
%
%   * mono16 images are returned as gray-scale images of size
%   MxNx1. Each pixel value is represented as a uint16.
%
%   * 32fcX images are returned as floating-point images of size
%   MxNxX, where X is either 1,2,3, or 4. Each pixel value is
%   represented as a single.
%
%   * 64fcX images are returned as floating-point images of size
%   MxNxX, where X is either 1,2,3, or 4. Each pixel value is
%   represented as a double.
%
%   * 8ucX images are returned as matrices of size
%   MxNxX, where X is either 1,2,3, or 4. Each pixel value is
%   represented as a uint8.
%
%   * 8scX images are returned as matrices of size
%   MxNxX, where X is either 1,2,3, or 4. Each pixel value is
%   represented as a int8.
%
%   * 16ucX images are returned as matrices of size
%   MxNxX, where X is either 1,2,3, or 4. Each pixel value is
%   represented as a uint16.
%
%   * 16scX images are returned as matrices of size
%   MxNxX, where X is either 1,2,3, or 4. Each pixel value is
%   represented as a int16.
%
%   * 32scX images are returned as matrices of size
%   MxNxX, where X is either 1,2,3, or 4. Each pixel value is
%   represented as a int32.
%
%   * bayer_X encoded images are either returned as raw Bayer
%   matrices of size MxNx1, or as de-bayered RGB images of size
%   MxNx3 (if a license of the Image Processing Toolbox is
%   present).
%
%   Other encodings are currently unsupported.
%
%   See also: ROSWRITEIMAGE

%   ROS stores channel data per image pixel, i.e.
%   rgbrgbrgb..., whereas MATLAB stores complete channels for
%   all image pixels, i.e. rrrr...gggg....bbbb...
%   This function implements the appropriate conversion.

%   Copyright 2020 The MathWorks, Inc.
%#codegen

% Ensure the generated code is not inlined
    coder.inline('never');

    % Validate Input argument and create local fields for ROS or ROS 2
    % sensor_msgs/Image
    validateattributes(msg, {'struct'},{'scalar'},'rosReadImage');

    % Code generation only supports sensor_msgs/Image as input message
    if ~isempty(coder.target)
        coder.internal.assert(strcmp(msg.MessageType,'sensor_msgs/Image'),...
                              'ros:mlroscpp:codegen:InvalidMsgForSpMsgFun',...
                              'rosReadImage','sensor_msgs/Image');
    end

    % Extract from sensor_msgs/Image
    if strcmp(msg.MessageType,'sensor_msgs/Image')
        if isfield(msg, 'Data')
            % ROS message struct
            specialMsgUtil = ros.internal.SpecialMsgUtil;
        else
            % ROS 2 message struct
            specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
        end
        [img, alpha] = specialMsgUtil.readImage(msg, varargin{:});
        % Extract from sensor_msgs/CompressedImage
    elseif strcmp(msg.MessageType,'sensor_msgs/CompressedImage')
        if isfield(msg, 'Data')
            % ROS message struct
            specialMsgUtil = ros.internal.SpecialMsgUtil;
        else
            % ROS 2 message struct
            specialMsgUtil = ros.internal.ros2.SpecialMsgUtil;
        end
        [img, alpha] = specialMsgUtil.readCompressedImage(msg);
    else
        coder.internal.error('ros:mlroscpp:image:InvalidMessageRead');
    end
end
