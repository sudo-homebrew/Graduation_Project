classdef Image < ros.msggen.sensor_msgs.Image
%Image Custom MATLAB implementation of sensor_msgs/Image type
%   This class provides a conversion between ROS images and their
%   respective MATLAB representations.
%
%   Image methods:
%      readImage    - Convert the ROS image data into a MATLAB image
%      writeImage   - Write a MATLAB image to the ROS image message
%
%   Note: To demosaic Bayer-encoded images, a license to the Image
%   Processing Toolbox is required.

%   Copyright 2014-2020 The MathWorks, Inc.

    properties (Access = ?matlab.unittest.TestCase)
        %Reader - The object for reading images
        Reader = ros.msg.sensor_msgs.internal.ImageReader

        %Writer - The object for writing images
        Writer = ros.msg.sensor_msgs.internal.ImageWriter
    end

    methods
        function obj = Image(varargin)
        %Image Constructor
        %   The arguments feed straight into the generated Image class.

            obj@ros.msggen.sensor_msgs.Image(varargin{:});
        end

        function [varargout] = readImage(obj)
        %readImage Convert the ROS image data into a MATLAB image
        %   [IMG,ALPHA] = readImage(OBJ) converts the raw image data in
        %   the message object OBJ into an image matrix IMG that is
        %   appropriate for further image processing in MATLAB. If the
        %   raw image contains an alpha channel, it will be returned in
        %   ALPHA. If no alpha channel exists, ALPHA will be empty.
        %
        %   ROS image message data is stored in a format that is not
        %   compatible with further image processing in MATLAB. Based
        %   on the specified encoding, this function will convert the
        %   data into an appropriate MATLAB image and return it.
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
        %   See also: writeImage

        %   This function reads the raw data from the ROS message
        %   and converts it into a representation that can be used by
        %   MATLAB's image or imshow functions.
        %   ROS stores channel data per image pixel, i.e.
        %   rgbrgbrgb..., whereas MATLAB stores complete channels for
        %   all image pixels, i.e. rrrr...gggg....bbbb...
        %   This function implements the appropriate conversion.


            varargout = cell(1, nargout);

            enc = ros.msg.sensor_msgs.internal.ImageEncoding.info(obj.Encoding);

            [varargout{:}] = obj.Reader.readImage( ...
                obj.Data, obj.Width, obj.Height, enc);

        end

        function writeImage(obj, varargin)
        %writeImage Write a MATLAB image to the ROS image message
        %   writeImage(OBJ, IMG) converts the MATLAB image IMG and
        %   stores the ROS compatible image data in the message object
        %   OBJ.
        %
        %   writeImage(OBJ, IMG, ALPHA) converts the MATLAB image IMG.
        %   If the image encoding supports an alpha channel (rgba
        %   or bgra family), this alpha channel can be specified as optional
        %   input ALPHA. Alternatively, the input image IMG can also
        %   store the alpha channel as its fourth channel.
        %
        %   The encoding of the input image has to be specified in the
        %   'encoding' property of the image message. If no encoding
        %   is specified before this function is called, the default
        %   encoding 'rgb8' will be used (3-channel RGB image with
        %   uint8 values).
        %
        %   All encodings supported for reading are also supported by
        %   the writeImage function. For more information on supported
        %   encodings and their representation in MATLAB, see the readImage
        %   function.
        %
        %   Bayer-encoded images (encodings 'bayer_rggb8',
        %   'bayer_bggr8', 'bayer_gbrg8', 'bayer_grbg8' and their
        %   16-bit equivalents) will not be Bayer-encoded but have to
        %   be given as 8-bit or 16-bit single-channel images
        %
        %   See also: readImage.

            if isempty(obj.Encoding)
                obj.Encoding = 'rgb8';
                warning(message('ros:mlroscpp:image:NoEncoding'));
            end

            enc = ros.msg.sensor_msgs.internal.ImageEncoding.info(obj.Encoding);

            [data, info] = obj.Writer.writeImage(enc, varargin{:});

            % Now write the data to the message object
            obj.Data = data;
            obj.Width = info.width;
            obj.Height = info.height;
            obj.Step = info.step;
        end
    end

    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file

        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msg.sensor_msgs.Image.empty(0,1);
                return
            end

            % Create an empty object and reload
            obj = ros.msg.sensor_msgs.Image(strObj);
        end
    end

end
