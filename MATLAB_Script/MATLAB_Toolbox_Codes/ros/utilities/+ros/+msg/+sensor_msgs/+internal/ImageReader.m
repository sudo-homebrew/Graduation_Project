classdef ImageReader
%This class is for internal use only. It may be removed in the future.

%ImageReader Class for converting ROS image messages to MATLAB images
%   This class contains utility functions for converting several image
%   encodings into representations that are usable within MATLAB.
%
%   See also: ros.msg.sensor_msgs.Image

%   Copyright 2014-2021 The MathWorks, Inc.

%#codegen

    methods (Static)

        function [img, alpha] = readImage(data, width, height, encoding)
        %readImage Convert the ROS image data into a MATLAB image
        %   See ros.msg.sensor_msgs.Image.readImage for
        %   details

        % Decode the actual image based on its encoding
            extractAlpha = (nargout == 2);

            numChannels = encoding.NumChannels;
            channelOrder = encoding.ChannelOrder;
            dataType = encoding.DataType;


            if encoding.HasAlpha
                % Color image with alpha channel
                if extractAlpha
                    [img, alpha] = ros.msg.sensor_msgs.internal.ImageReader.convertToRGBA( ...
                        data, width, height, numChannels, channelOrder, dataType);
                else
                    % Ignore alpha channel
                    img = ros.msg.sensor_msgs.internal.ImageReader.convertToImg( ...
                        data, width, height, numChannels, channelOrder(1:end-1), dataType);
                    alpha = [];
                end
            elseif encoding.IsBayer
                % Bayer image
                if isempty(coder.target)
                    img = ros.msg.sensor_msgs.internal.ImageReader.convertToBayer(data, ...
                                                                                  width, height, encoding.SensorAlignment, dataType);
                    alpha = [];
                else
                    % Code generation does not support debayer operation
                    coder.internal.error('ros:mlroscpp:codegen:InvalidMsgStruct','rosReadImage','Bayer');
                end
            else
                % All others
                img = ros.msg.sensor_msgs.internal.ImageReader.convertToImg( ...
                    data, width, height, numChannels, channelOrder, dataType);
                alpha = cast([],dataType);
            end
        end

        function img = convertToBayer(rawData, width, height, alignment, dataType)
        %convertToBayer Convert the image data to a MATLAB Bayer image
        %   If the Image Processing Toolbox is installed, the raw image
        %   will be debayered and returned as an RGB image. Otherwise,
        %   the image will be returned as a 1-channel image.
        %
        %   IMG = convertToBayer(RAWDATA, WIDTH, HEIGHT, ALIGNMENT, DATATYPE) converts
        %   the raw image data contained in the ROS image data RAWDATA
        %   into an image IMG that can be used for further processing in MATLAB.
        %   Its size will be HEIGHT x WIDTH. If the Image Processing
        %   Toolbox is installed, the return image will be an RGB image
        %   of size HEIGHT x WIDTH x 3. DATATYPE is the MATLAB data type for each
        %   stored pixel value in a channel. ALIGNMENT is the sensor
        %   element alignment
        %
        %   Example:
        %      img = convertToBayer(data, 640, 480, 'bggr', 'uint16');

            img = ros.msg.sensor_msgs.internal.ImageReader.convertToImg(...
                rawData, width, height, 1, [], dataType);

            % Also debayer the image if IPT is installed
            if ros.msg.sensor_msgs.internal.ImageLicense.isIPTLicensed
                img = demosaic(img, alignment);
            end

        end

        function [img,alpha] = convertToRGBA(rawData, width, height, numChannels, channelOrder, dataType)
        %convertToRGBA Convert the image data to RGB and Alpha
        %   The user has to specify information about the number of
        %   channels, channel order, and data type to correctly
        %   interpret the data.

        % Get full image first (height x width x 4)
            imgFull = ros.msg.sensor_msgs.internal.ImageReader.convertToImg(...
                rawData, width, height, numChannels, channelOrder, dataType);

            % Split it up into components
            img = imgFull(:,:,1:3);
            alpha = imgFull(:,:,4);
        end

        function img = convertToImg(rawData, width, height, numChannels, channelOrder, dataType)
        %convertToImg Convert raw image data to a MATLAB matrix
        %   This function is used as a backend for all image conversion
        %   functions. It is optimized for speed to provide a flexible
        %   and performant interface to retrieve image data.
        %
        %   IMG = convertToImg(RAWDATA, WIDTH, HEIGHT, NUMCHANNELS,
        %   CHANNELORDER, DATATYPE) converts the raw image data contained
        %   in the ROS image data RAWDATA and converts it into an image
        %   IMG that can be used for further processing in MATLAB. IMG
        %   will be a HEIGHT x WIDTH x NUMEL(CHANNELORDER) matrix.
        %   NUMCHANNELS is the number of channels that the image has
        %   and CHANNELORDER is a vector of integers that specify
        %   where in the raw stream the output channels can be found.
        %   DATATYPE is the MATLAB data type for each
        %   stored pixel value in a channel.
        %
        %   Examples:
        %       % Convert a raw BGRA 8-bit image into RGB MATLAB image
        %       img = convertToImg(data, 640, 480, 4, [3,2,1], 'uint8');
        %
        %       % Keep the alpha channel of the BGRA 8-bit image
        %       % Output img will be 480x640x4
        %       img = convertToImg(data, 640, 480, 4, [3,2,1,4], 'uint8');
        %
        %       % Convert a raw RGB 16-bit image into MATLAB image
        %       img = obj.convertToRGB(data, 640, 480, 3, [1,2,3], 'uint16');


        % Return if no data was received
            if isempty(rawData)
                img = typecast(rawData, dataType);
                return;
            end

            % Cast input data to the expected data type
            data = typecast(rawData, dataType);

            % The output image will be height x width x numOutputChannels
            numOutputChannels = numel(channelOrder);

            % Ensure that width, height, and number of channels are
            % consistent with raw stored data
            expectedElems = width*height*numChannels;
            if expectedElems ~= numel(data)
                coder.internal.error('ros:mlroscpp:image:DataLengthInconsistency', ...
                                     expectedElems, width, height, numChannels, numel(data));
            end

            % If this is a single-channel image, return right away
            if numChannels == 1
                img = reshape(data, width, height)';
                return;
            end

            % Construct the data access index. Since the channel ordering
            % might be arbitrary, this has to be done in a for loop
            indexBase = (1:numChannels:width*height*numChannels)';
            index = zeros(numel(indexBase),numOutputChannels);
            for i = 1:numOutputChannels
                % Construct it as column-major index to ease later reshape
                index(:, channelOrder(i)) = indexBase + i - 1;
            end

            % Reshape the data into a width x height x numOutputChannels matrix
            img = reshape(data(index), width, height, numOutputChannels);

            % Since reshape works column-wise, bring image back to
            % row-major order (transpose of 1st and 2nd dimension
            % Result will be a height x width x numOutputChannels image matrix
            img = permute(img, [2 1 3]);
        end

        function img = decompressImg(data, ~)
        %decompressImg Take raw data and decompress
            if coder.target('MATLAB')
                try
                    img = matlab.internal.imdecode(data);
                catch ex
                    newEx = ros.internal.ROSException( ...
                        message('ros:mlroscpp:image:DecompressError'));
                    throw(newEx.addCustomCause(ex));
                end
            else
                img = [];
            end
        end
    end
end
