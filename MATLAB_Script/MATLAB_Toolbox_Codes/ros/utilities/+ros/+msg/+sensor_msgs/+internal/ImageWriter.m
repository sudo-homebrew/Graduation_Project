classdef ImageWriter
%This class is for internal use only. It may be removed in the future.

%ImageWriter Class for converting MATLAB images into ROS image messages
%   This class contains utility functions for converting several image
%   encodings into representations that can be sent over ROS.
%
%   See also: ros.msg.sensor_msgs.Image

%   Copyright 2014-2021 The MathWorks, Inc.

%#codegen

    methods (Static)
        function [data, info] = writeImage(encoding, img, varargin)
        %writeImage Encode MATLAB image for use in a ROS message
        %   [DATA, INFO] = writeImage(ENCODING, IMG) serializes the
        %   MATLAB image, IMG, into a uint8 array, DATA, according to
        %   the specified ENCODING. INFO is a struct with the follow
        %   fields:
        %       - width: Width of the image in pixels
        %       - height: Height of the image in pixels
        %       - step: Number of bytes required to store one row of
        %         the image.
        %   [DATA, INFO] = writeImage(ENCODING, IMG, ALPHA) serializes
        %   the image, IMG, and transparency data, ALPHA, according to
        %   the specified ENCODING.
        %
        %   See also: ros.msg.sensor_msgs.Image.writeImage

        % Parse inputs to function
            defaults.alpha = [];
            img = ros.msg.sensor_msgs.internal.ImageWriter.parseWriteArguments(defaults, img, varargin{:});
            dataType = encoding.DataType;
            numChannels = encoding.NumChannels;
            [data, info] = ros.msg.sensor_msgs.internal.ImageWriter.convertToROSImg(img, encoding, numChannels, dataType);
        end

        function [data, imgInfo] = convertToROSImg(img, encoding, numChannels, dataType)
        %convertToROSImg Convert MATLAB image matrix to ROS image

            [height, width, channels] = size(img);

            % Verify input data type of the image
            if ~isa(img, dataType)
                coder.internal.error('ros:mlroscpp:image:TypeMismatch', encoding.Name, ...
                              dataType, class(img));
            end

            % Verify that expected number of channels exists
            if channels ~= numChannels
                coder.internal.error('ros:mlroscpp:image:InvalidChannelNumber', ...
                              encoding.Name, num2str(numChannels), num2str(channels));
            end

            % Convert image to a data array
            % Pattern: [R G B R G B ...raw-wise]

            % Flip dimensions
            image = permute(img,[3 2 1]);

            % Vectorize and typecast to uint8
            data = typecast(image(:), 'uint8');

            % Calculate number of bytes for data type
            if isempty(coder.target)
                numBytes = numel(typecast(feval(dataType, 0), 'uint8'));
            else
                % Code generation
                if strcmp(dataType,'uint8') || strcmp(dataType,'int8')
                    numBytes = 1;
                elseif strcmp(dataType,'uint16') || strcmp(dataType,'int16')
                    numBytes = 2;
                elseif strcmp(dataType,'uint32') || strcmp(dataType,'int32') || strcmp(dataType, 'single')
                    numBytes = 4;
                else
                    % double type
                    numBytes = 8;
                end
            end
            % Return image information as well
            imgInfo.step = width*channels*numBytes;
            imgInfo.width = width;
            imgInfo.height = height;
        end
    end

    methods (Static, Access = private)
        function img = parseWriteArguments(defaults, img, varargin)
        %parseWriteArguments Parse arguments for writeImage function

            validateattributes(img,{'numeric'},{'nonempty'},'Image','img');

            % Parse the alpha input
            alpha = defaults.alpha;
            if length(varargin)==1 || length(varargin)==3
                alpha = varargin{1};
                validateattributes(alpha, {'numeric'},{},'Image','alpha');
            end

            if isempty(alpha)
                return;
            end

            % Verify that alpha is the right size and type
            [width, height, ~] = size(img);
            validateattributes(alpha, {class(img)}, {'size', [width, ...
                                height, 1]}, 'ImageWriter', 'alpha');

            % Merge image and alpha channel
            %img(:,:,end+1) = alpha;
            img = cat(3,img,alpha);
        end
    end
end
