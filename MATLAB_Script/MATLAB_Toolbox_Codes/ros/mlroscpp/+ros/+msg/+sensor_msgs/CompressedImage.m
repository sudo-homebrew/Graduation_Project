classdef CompressedImage < ros.msggen.sensor_msgs.CompressedImage
%CompressedImage Custom MATLAB implementation of sensor_msgs/CompressedImage type
%   This class provides a conversion between compressed ROS images and
%   their respective MATLAB representations. No conversion from MATLAB
%   images to ROS images is provided.
%
%   Compressed images are published through the compressed_image_transport ROS
%   package. Two types of compression are supported, PNG and JPG, and
%   the type of compression can change on-the-fly.
%
%   CompressedImage methods:
%      readImage    - Convert the ROS image data into a MATLAB image

%   Copyright 2014-2020 The MathWorks, Inc.

    properties (Access = ?matlab.unittest.TestCase)
        %Reader - The object for reading images
        Reader = ros.msg.sensor_msgs.internal.ImageReader
    end

    methods
        function obj = CompressedImage(varargin)
        %CompressedImage Constructor
        %   The arguments feed straight into the generated CompressedImage class.

            obj@ros.msggen.sensor_msgs.CompressedImage(varargin{:});
        end

        function img = readImage(obj)
        %readImage Convert the ROS image data into a MATLAB image
        %   IMG = readImage(OBJ) converts the raw image data in
        %   the message object OBJ into an image matrix IMG that is
        %   appropriate for further image processing in MATLAB.
        %
        %   ROS image message data is stored in a format that is not
        %   compatible with further image processing in MATLAB. Based
        %   on the specified compression format, this function will
        %   decompress the image data and return it as a MATLAB image.
        %
        %   The following encodings for raw images of size MxN are supported:
        %   * rgb8, rgba8, bgr8, and bgra8: IMG is an RGB image
        %   of size MxNx3. Each pixel value has a uint8 data type.
        %
        %   * mono8 images are returned as gray-scale images of size
        %   MxNx1. Each pixel value is represented as a uint8.
        %
        %   Other encodings are currently unsupported in a compressed
        %   form. See ros.msg.sensor_msgs.Image for all encodings
        %   supported for uncompressed images.
        %
        %   See also: ros.msg.sensor_msgs.Image

            format = obj.Format;

            % An empty format string is not allowed
            if isempty(format)
                error(message('ros:mlroscpp:image:EmptyFormatRead'));
            end

            % Parse the format string
            % It has the form e.g. of "rgb8; png compressed bgr8"
            % We only need the string in front of the semicolon
            parts = strsplit(format, ';');
            if numel(parts) ~= 2
                error(message('ros:mlroscpp:image:FormatNoSemicolon', ...
                              format));
            end
            encoding = parts{1};

            % Return if no data was received
            if isempty(obj.Data)
                img = obj.Data;
                return;
            end

            % The decompression routine will take care of any color space
            % conversions. Only have to worry about channel numbers here.
            switch lower(encoding)
              case {'rgb8', 'rgba8', 'bgr8', 'bgra8'}
                % Only the RGB channels will be returned.
                % The alpha channel is not part of the compressed
                % image.
                img = obj.Reader.decompressImg(obj.Data, 3);

              case 'mono8'
                % Single-channel 8-bit gray-scale image
                img = obj.Reader.decompressImg(obj.Data, 1);

              otherwise
                error(message('ros:mlroscpp:image:UnsupportedFormatRead', ...
                              format));
            end
        end
    end

    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file

        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msg.sensor_msgs.CompressedImage.empty(0,1);
                return
            end

            % Create an empty object
            obj = ros.msg.sensor_msgs.CompressedImage(strObj);
        end
    end
end
