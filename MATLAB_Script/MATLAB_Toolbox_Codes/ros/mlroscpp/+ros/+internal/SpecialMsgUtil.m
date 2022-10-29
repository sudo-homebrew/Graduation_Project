classdef (Hidden) SpecialMsgUtil
%SpecialMsgUtil - Static utilities class for message structs

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

    methods(Static)
        % Utilities functions for Image and CompressedImage messages
        function [img, alpha] = readImage(msg, varargin)
        %readImage Read image from Image message struct
        % Validate input message struct
            validateattributes(msg.Width,{'uint32','double'},{'scalar'},'rosReadImage');
            validateattributes(msg.Height,{'uint32','double'},{'scalar'},'rosReadImage');

            if isempty(coder.target)
                validateattributes(msg.Encoding,{'char'},{'nonempty'},'rosReadImage');
                validateattributes(msg.Data,{'uint8'},{'vector'},'rosReadImage');

                % Use encoding information from message struct directly
                enc = ros.msg.sensor_msgs.internal.ImageEncoding.info(msg.Encoding);
            else
                % Ensure the input message contains valid data
                coder.internal.assert(~isempty(msg.Data),'ros:mlroscpp:codegen:EmptyInputMsg','rosReadImage');
                % Parse optional Encoding property for code generation
                nvPairs = struct('Encoding',uint32(0));
                pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
                pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{1:end});

                encoding = coder.internal.getParameterValue(pStruct.Encoding,'rgb8',varargin{1:end});
                %validateattributes(encoding,{'char','string'},...
                %                   {'nonempty'},'rosReadImage','Encoding');
                validatestring(encoding,{'rgb8', 'rgba8', 'rgb16', 'rgba16', ...
                                         'bgr8', 'bgra8', 'bgr16', 'bgra16', 'mono8', 'mono16', ...
                                         '32fc1', '32fc2', '32fc3', '32fc4', ...
                                         '64fc1', '64fc2', '64fc3', '64fc4', ...
                                         '8uc1', '8uc2', '8uc3', '8uc4', ...
                                         '8sc1', '8sc2', '8sc3', '8sc4', ...
                                         '16uc1', '16uc2', '16uc3', '16uc4', ...
                                         '16sc1', '16sc2', '16sc3', '16sc4', ...
                                         '32sc1', '32sc2', '32sc3', '32sc4', ...
                                         'bayer_rggb8', 'bayer_bggr8', 'bayer_gbrg8', 'bayer_grbg8', ...
                                         'bayer_rggb16', 'bayer_bggr16', 'bayer_gbrg16', 'bayer_grbg16'},'rosReadImage','Encoding');

                enc = coder.const(ros.msg.sensor_msgs.internal.ImageEncoding.info(encoding));
            end

            % Create Reader object and convert image data into MATLAB image
            Reader = ros.msg.sensor_msgs.internal.ImageReader;
            [img, alpha] = Reader.readImage(msg.Data,msg.Width,msg.Height,enc);
        end

        function [img, alpha] = readCompressedImage(msg, varargin)
        %readCompressedImage Read image from CompressedImage message struct

        % alpha is not applicable for compressed images
            alpha = [];

            % Setup local copy of image data information
            format = msg.Format;
            data = msg.Data;

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
            if isempty(data)
                img = data;
                return;
            end

            % Create Reader object and convert image data into MATLAB image
            Reader = ros.msg.sensor_msgs.internal.ImageReader;

            % The decompression routine will take care of any color space
            % conversions. Only have to worry about channel numbers here.
            switch lower(encoding)
              case {'rgb8', 'rgba8', 'bgr8', 'bgra8'}
                % Only the RGB channels will be returned.
                % The alpha channel is not part of the compressed
                % image.
                img = Reader.decompressImg(data, 3);

              case 'mono8'
                % Single-channel 8-bit gray-scale image
                img = Reader.decompressImg(data, 1);

              otherwise
                error(message('ros:mlroscpp:image:UnsupportedFormatRead', ...
                              format));
            end
        end

        function msg = writeImage(msg, varargin)
        %writeImage Write image to image message struct

        % Ensure the message contains enough space for saving streaming data
        %coder.internal.assert(isequal(numel(msg.Data),numel(varargin{1})),...
        %    'ros:mlroscpp:codegen:MismatchStreamDataSize');

        % Determine the start location of encoding input argument
            if nargin<=2
                % No alpha and encoding input
                startLoc = 2;
            elseif isnumeric(varargin{2})
                % User specified alpha input
                startLoc = 3;
            else
                % User did not specify alpha input
                startLoc = 2;
            end

            % Parse optional Encoding property for code generation
            nvPairs = struct('Encoding',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{startLoc:end});

            encoding = coder.internal.getParameterValue(pStruct.Encoding,'rgb8',varargin{startLoc:end});
            encoding = convertStringsToChars(encoding);
            validatestring(encoding,{'rgb8', 'rgba8', 'rgb16', 'rgba16', ...
                                     'bgr8', 'bgra8', 'bgr16', 'bgra16', 'mono8', 'mono16', ...
                                     '32fc1', '32fc2', '32fc3', '32fc4', ...
                                     '64fc1', '64fc2', '64fc3', '64fc4', ...
                                     '8uc1', '8uc2', '8uc3', '8uc4', ...
                                     '8sc1', '8sc2', '8sc3', '8sc4', ...
                                     '16uc1', '16uc2', '16uc3', '16uc4', ...
                                     '16sc1', '16sc2', '16sc3', '16sc4', ...
                                     '32sc1', '32sc2', '32sc3', '32sc4', ...
                                     'bayer_rggb8', 'bayer_bggr8', 'bayer_gbrg8', 'bayer_grbg8', ...
                                     'bayer_rggb16', 'bayer_bggr16', 'bayer_gbrg16', 'bayer_grbg16'},'rosReadImage','Encoding');

            if isempty(coder.target)
                % Extract message encoding information
                if isempty(msg.Encoding)
                    msg.Encoding = encoding;
                    if numel(varargin)<3
                        % Throw warning only when applying the default
                        % encoding
                        warning(message('ros:mlroscpp:image:NoEncoding'));
                    end
                end

                enc = msg.Encoding;
                validateattributes(enc,{'char'},{'nonempty'},'rosWriteImage');

                enc = ros.msg.sensor_msgs.internal.ImageEncoding.info(enc);
            else
                % Code generation
                msg.Encoding = encoding;
                enc = coder.const(ros.msg.sensor_msgs.internal.ImageEncoding.info(encoding));
            end

            Writer = ros.msg.sensor_msgs.internal.ImageWriter;
            [data, info] = Writer.writeImage(enc, varargin{:});

            % Assign to message
            msg.Data = data;
            msg.Width = cast(info.width,'like',msg.Width);
            msg.Height = cast(info.height,'like',msg.Height);
            msg.Step = cast(info.step,'like',msg.Step);
        end

        % Utilities functions for PointCloud2 messages
        function fieldNames = getAllFieldNames(msg)
        %getAllFieldNames Return all field names in PointCloud2 message

            numFieldNames = numel(msg.Fields);
            fieldNames = cell(1,numFieldNames);

            % If no point fields are available, return
            if isempty(msg.Fields)
                fieldNames = {};
                return;
            end
            validateattributes(msg.Fields(1).Name,{'char'},{'scalartext'},'rosReadAllFieldNames');

            for i = 1:numFieldNames
                fieldNames{i} = msg.Fields(i).Name;
            end
        end

        function outputs = writeCameraInfo(params, varargin)
        % writeCameraInfo Writes data from params to
        % sensor_msgs/CameraInfo message(s). The params input is either a
        % cameraParameters or stereoParameters struct.

        %     [fx'  0  cx' Tx]
        % P = [ 0  fy' cy' Ty]
        %     [ 0   0   1   0]
        % here, the prime notation indicates the new camera
        % parameters for the rectified image
        % For monocular cameras, Tx = Ty = 0.

            if ~isfield(params, 'CameraParameters1')
                msg1 = varargin{1};
                msg1 = ros.internal.SpecialMsgUtil.setCameraFields(msg1, params);
                % identity matrix
                msg1.R([1 5 9]) = 1;
                % projection matrix
                P = zeros(3, 4);
                P(1:3, 1:3) = params.IntrinsicMatrix';
                msg1.P = reshape(P', 12, 1);
                outputs = {msg1};

            else % must be a stereoParameters object
                msg1 = varargin{1};
                msg2 = varargin{2};
                msg1 = ros.internal.SpecialMsgUtil.setCameraFields(msg1, ...
                                                                   params.CameraParameters1);
                msg2 = ros.internal.SpecialMsgUtil.setCameraFields(msg2, ...
                                                                   params.CameraParameters2);
                [K_new, Rrect1, Rrect2] = ros.internal.SpecialMsgUtil.getRectifiedParameters(params);
                % rotation matrix
                msg1.R = reshape(Rrect1', 9, 1);
                msg2.R = reshape(Rrect2', 9, 1);
                % projection matrix
                P = zeros(3, 4);
                P(1:3, 1:3) = K_new';
                msg1.P = reshape(P', 12, 1);
                P(1:2, 4) = params.TranslationOfCamera2(1:2);
                msg2.P = reshape(P', 12, 1);
                outputs = {msg1, msg2};
            end

        end

        function msg = setCameraFields(msg, cameraParams)
            msg.DistortionModel = 'plumb_bob';
            k3 = cameraParams.RadialDistortion(end)*(cameraParams.NumRadialDistortionCoefficients > 2);
            msg.D = [cameraParams.RadialDistortion(1:2) cameraParams.TangentialDistortion k3]';
            msg.Height = uint32(cameraParams.ImageSize(1));
            msg.Width = uint32(cameraParams.ImageSize(2));
            % ROS intrinsic camera matrix format for the raw images.
            %     [fx  0 cx]
            % K = [ 0 fy cy]
            %     [ 0  0  1]
            % All ROS matrices are row-major
            msg.K = reshape(cameraParams.IntrinsicMatrix, 9, 1);
        end

        function [K_new, Rrect1, Rrect2] = getRectifiedParameters(stereoParams)
            cp1 = stereoParams.CameraParameters1;
            cp2 = stereoParams.CameraParameters2;
            Kl = cp1.IntrinsicMatrix;
            Kr = cp2.IntrinsicMatrix;
            K_new = Kl;
            % find new focal length
            f_new = min([Kr(1,1),Kl(1,1)]);
            % set new focal lengths
            K_new(1,1)=f_new; K_new(2,2)=f_new;
            % find new y center
            cy_new = (Kr(2,3)+Kl(2,3)) / 2;
            % set new y center
            K_new(2,3)= cy_new;
            % set the skew to 0
            K_new(1,2) = 0;
            H1 = stereoParams.RectificationParams.H1;
            H2 = stereoParams.RectificationParams.H2;
            Rrect1 = (K_new'\H1'*Kl');
            Rrect2 = (K_new'\H2'*Kr');
        end

        function fieldData = getPointCloud2Field(msg, fieldName, varargin)
        % Validate Input argument
            validateattributes(msg,{'struct'},{'scalar'},'rosReadField');
            validateattributes(msg.Data,{'uint8'},{'vector'},'rosReadField');
            validateattributes(msg.Width,{'uint32','double'},{'scalar'},'rosReadField');
            validateattributes(msg.Height,{'uint32','double'},{'scalar'},'rosReadField');
            fieldName = convertStringsToChars(fieldName);
            validateattributes(fieldName, {'char', 'string'}, {'scalartext'}, 'rosReadField', ...
                               'fieldName');

            %TypeConversion - Object handling ROS <--> MATLAB type conversions
            TypeConversion = ros.msg.sensor_msgs.internal.PointCloud2Types;

            % Get index of field. This will display an error if the field
            % name does not exist.
            fieldIdx = ros.internal.SpecialMsgUtil.getFieldNameIndex(msg,fieldName);

            datatype = msg.Fields(fieldIdx).Datatype;
            count = msg.Fields(fieldIdx).Count;

            mlType = TypeConversion.rosToMATLABType(datatype);

            pointIndices = 1: msg.Width * msg.Height;

            % Parse optional PreserveStructureOnRead property
            nvPairs = struct('PreserveStructureOnRead',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{1:end});

            reshapeOutput = coder.internal.getParameterValue(pStruct.PreserveStructureOnRead,false,varargin{1:end});
            validateattributes(reshapeOutput,{'logical'},...
                               {'scalar'},'rosReadField','PreserveStructureOnRead');

            % Extract the bytes corresponding to this field
            [byteIdx, pointIdxIsValid] = ros.internal.SpecialMsgUtil.getByteIndexForField(msg, fieldIdx, pointIndices, count);
            fieldDataTemp = ros.internal.SpecialMsgUtil.readFieldFromData(msg.Data, ...
                                                                          byteIdx, pointIdxIsValid, mlType, count);

            % Reshape the output if requested by the user
            if reshapeOutput && msg.Height ~= 1
                if count > 1
                    fieldData = reshape(fieldDataTemp, msg.Width, msg.Height, count);
                    fieldData = permute(fieldData, [2 1 3]);
                else
                    fieldData = reshape(fieldDataTemp, msg.Width, msg.Height).';
                end
            else
                fieldData = fieldDataTemp;
            end
        end

        function xyz = readXYZ(msg, varargin)
        %readXYZ Read XYZ from PointCloud2 message struct

        %TypeConversion - Object handling ROS <--> MATLAB type conversions
            TypeConversion = ros.msg.sensor_msgs.internal.PointCloud2Types;

            % Validate input message data
            validateattributes(msg.Data,{'uint8'},{'vector'},'rosReadXYZ');
            validateattributes(msg.Width,{'uint32','double'},{'scalar'},'rosReadXYZ');
            validateattributes(msg.Height,{'uint32','double'},{'scalar'},'rosReadXYZ');

            % Get field indices for X, Y, and Z coordinates
            xIdx = ros.internal.SpecialMsgUtil.getFieldNameIndex(msg,'x');
            yIdx = ros.internal.SpecialMsgUtil.getFieldNameIndex(msg,'y');
            zIdx = ros.internal.SpecialMsgUtil.getFieldNameIndex(msg,'z');

            pointIndices = 1: msg.Width * msg.Height;

            % Parse optional PreserveStructureOnRead property
            nvPairs = struct('PreserveStructureOnRead',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{1:end});

            reshapeOutput = coder.internal.getParameterValue(pStruct.PreserveStructureOnRead,false,varargin{1:end});
            validateattributes(reshapeOutput,{'logical'},...
                               {'scalar'},'rosReadField','PreserveStructureOnRead');

            % Get byte index only once (this is expensive)
            [byteIdx, pointIdxIsValid] = ros.internal.SpecialMsgUtil.getByteIndexForField(msg, xIdx, pointIndices, 1);

            % Recover MATLAB data type of field elements
            xMlType = TypeConversion.rosToMATLABType(msg.Fields(xIdx).Datatype);
            yMlType = TypeConversion.rosToMATLABType(msg.Fields(yIdx).Datatype);
            zMlType = TypeConversion.rosToMATLABType(msg.Fields(zIdx).Datatype);
            % Calculate the byte offsets for the different fields
            % This helps with performance, since we can re-use the
            % byte index computed below
            xOff = double(msg.Fields(xIdx).Offset);
            yOff = double(msg.Fields(yIdx).Offset) - xOff;
            zOff = double(msg.Fields(zIdx).Offset) - xOff;

            % Retrieve the XYZ data and concatenate into one matrix
            xyzTemp = ...
                [ros.internal.SpecialMsgUtil.readFieldFromData(msg.Data, byteIdx, pointIdxIsValid, xMlType,1), ...
                 ros.internal.SpecialMsgUtil.readFieldFromData(msg.Data, byteIdx + yOff, pointIdxIsValid, yMlType,1), ...
                 ros.internal.SpecialMsgUtil.readFieldFromData(msg.Data, byteIdx + zOff, pointIdxIsValid, zMlType,1)];

            % Reshape the output if requested by the user
            if reshapeOutput && msg.Height ~= 1
                xyz = reshape(xyzTemp, msg.Width, msg.Height, 3);
                xyz = permute(xyz, [2 1 3]);
            else
                xyz = xyzTemp;
            end
        end

        function rgb = readRGB(msg, varargin)
        %readRGB Read RGB from PointCloud2 message struct

        % Validate input message data
            validateattributes(msg.Data,{'uint8'},{'vector'},'rosReadRGB');
            validateattributes(msg.Width,{'uint32','double'},{'scalar'},'rosReadRGB');
            validateattributes(msg.Height,{'uint32','double'},{'scalar'},'rosReadRGB');

            % Get field index for the RGB field
            rgbIdx = ros.internal.SpecialMsgUtil.getFieldNameIndex(msg,'rgb');

            pointIndices = 1: msg.Height * msg.Width;

            % Parse optional PreserveStructureOnRead property
            nvPairs = struct('PreserveStructureOnRead',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{1:end});

            reshapeOutput = coder.internal.getParameterValue(pStruct.PreserveStructureOnRead,false,varargin{1:end});
            validateattributes(reshapeOutput,{'logical'},...
                               {'scalar'},'rosReadField','PreserveStructureOnRead');

            % Get byte index for the RGB field (this is expensive)
            [byteIdx, pointIdxIsValid] = ros.internal.SpecialMsgUtil.getByteIndexForField(msg, rgbIdx, pointIndices, 1);
            numPoints = numel(pointIndices);
            count = 4;
            rgbRaw = ros.internal.SpecialMsgUtil.readRGBFieldFromData(msg.Data, byteIdx, pointIdxIsValid, count);

            rgbTemp = NaN(numPoints,3);

            % Scale values from [0,255] to [0,1] range
            % Also convert from BGR -> RGB
            rgbTemp(pointIdxIsValid, :) = double(rgbRaw(pointIdxIsValid, 3:-1:1)) / 255;

            % Reshape the output if requested by the user
            if reshapeOutput && msg.Height ~= 1
                % Since image is row-major, this is a two-step process
                rgb = reshape(rgbTemp, msg.Width, msg.Height, 3);
                rgb = permute(rgb, [2 1 3]);
            else
                rgb = rgbTemp;
            end
        end

        function [cart, cartAngles] = readCartesian(msg, varargin)
        %readCartesian Read cartesian from LaserScan message

        % Validate input message data
            validateattributes(msg.Ranges,{'single','uint16'},{'vector'},'rosReadCartesian');
            validateattributes(msg.RangeMax,{'single','uint16','double','uint32'},{'scalar'},'rosReadCartesian');
            validateattributes(msg.RangeMin,{'single','uint16','double','uint32'},{'scalar'},'rosReadCartesian');

            defaults.RangeLimits = [msg.RangeMin,msg.RangeMax];
            args = ros.internal.SpecialMsgUtil.parseReadCartesianArguments(defaults, varargin{:});

            R = msg.Ranges(:);

            if isempty(coder.target)
                % If no ranges are available, return right away
                if isempty(R)
                    cart = single.empty(0,2);
                    cartAngles = single.empty(0,1);
                    return;
                end
            end

            % Discard all values that are below lower range limit or above the
            % upper range limit. Also discard infinite or NaN values
            validIdx = isfinite(R) & R >= args.RangeLimits(1) & R <= args.RangeLimits(2);
            validR = R(validIdx);

            % Filter scan angles for all valid range readings
            angles = rosReadScanAngles(msg);
            cartAngles = angles(validIdx);
            x = cos(cartAngles) .* validR;
            y = sin(cartAngles) .* validR;
            cart = double([x,y]);
        end

        function angles = readScanAngles(msg)
        %readScanAngles Read scan angles from LaserScan message

        % Validate input message data
            validateattributes(msg.Ranges,{'single','uint16'},{'vector'},'rosReadScanAngles');
            validateattributes(msg.AngleIncrement,{'single','uint16','double','uint32'},{'scalar'},'rosReadScanAngles');
            validateattributes(msg.AngleMin,{'single','uint16','double','uint32'},{'scalar'},'rosReadScanAngles');

            numReadings = numel(msg.Ranges);
            rawAngles = msg.AngleMin + (0:numReadings-1)' * msg.AngleIncrement;

            % Wrap the angles to the (-pi,pi] interval.
            angles = robotics.internal.wrapToPi(double(rawAngles));
        end

        function lidarScanObj = readLidarScan(msg)
        %readLidarScan Construct LidarScan object from LaserScan message

            validateattributes(msg.Ranges,{'single','uint16'},{'vector'},'rosReadLidarScan');

            angles = rosReadScanAngles(msg);
            lidarScanObj = lidarScan(double(msg.Ranges), angles);
        end

        function map = readBinaryOccupancyGrid(msg, varargin)
        %readBinaryOccupancyGrid return binary map from input message

        % Validate message values
            validateattributes(msg.Data,{'int8','double'},{'vector'},'rosReadBinaryOccupancyGrid');
            validateattributes(msg.Info.Width,{'uint32','double'},{'scalar'},'rosReadBinaryOccupancyGrid');
            validateattributes(msg.Info.Height,{'uint32','double'},{'scalar'},'rosReadBinaryOccupancyGrid');
            ros.internal.SpecialMsgUtil.validateOccGridMsgValues(msg);

            % Define default variables
            defaultOccupancyThreshold = 50;
            defaultValueForUnknown = 0;
            unknownValueInMsg = -1;
            opts.OccupancyThreshold = defaultOccupancyThreshold;
            opts.ValueForUnknown = defaultValueForUnknown;

            if ~isempty(varargin)
                opts = ros.internal.SpecialMsgUtil.parseReadBOGInputs(varargin{:});
            end

            resolution = 1/double(msg.Info.Resolution);

            % Handle eps errors in resolution due to single to double
            % conversion. Using 1e-5 as a practical maximum EPS of a single
            % resolution
            if abs(round(resolution) - resolution) < 1e-5
                resolution = round(resolution);
            end

            map = binaryOccupancyMap(double(msg.Info.Height), ...
                                     double(msg.Info.Width), resolution, 'grid');

            % Fill in the meta information
            map.GridLocationInWorld = double([msg.Info.Origin.Position.X,...
                                              msg.Info.Origin.Position.Y]);

            % Set grid values in the binaryOccupancyMap
            data = msg.Data;
            values = double(data);
            values(data == unknownValueInMsg) = opts.ValueForUnknown;
            values(data < opts.OccupancyThreshold & data ~= unknownValueInMsg) = 0;
            values(data >= opts.OccupancyThreshold) = 1;

            % Write the values in the binaryOccupancyMap
            map = ros.internal.SpecialMsgUtil.writeToOG(map, values);
        end

        function map = readOccupancyGrid(msg)
        %readOccupancyGrid return map from input message

        % Validate message values
            validateattributes(msg.Data,{'int8'},{'vector'},'rosReadOccupancyGrid');
            validateattributes(msg.Info.Width,{'uint32','double'},{'scalar'},'rosReadOccupancyGrid');
            validateattributes(msg.Info.Height,{'uint32','double'},{'scalar'},'rosReadOccupancyGrid');
            ros.internal.SpecialMsgUtil.validateOccGridMsgValues(msg);

            unknownValueInMsg = -1;
            MaxValueInMsg = 100;

            resolution = 1/double(msg.Info.Resolution);

            % Handle eps errors in resolution due to single to double
            % conversion. Using 1e-5 as a practical maximum EPS of a single
            % resolution
            if abs(round(resolution) - resolution) < 1e-5
                resolution = round(resolution);
            end

            map = occupancyMap(double(msg.Info.Height), ...
                               double(msg.Info.Width), resolution, 'grid');

            % Fill in the meta information
            map.GridLocationInWorld = double([msg.Info.Origin.Position.X,...
                                              msg.Info.Origin.Position.Y]);

            % Set grid values in the occupancyMap
            data = msg.Data;
            values = double(data);
            values(data == unknownValueInMsg) = MaxValueInMsg/2;
            values = values/MaxValueInMsg;

            % Write the values in the occupancyMap
            map = ros.internal.SpecialMsgUtil.writeToOG(map, values);
        end

        function msgAssigned = writeBinaryOccupancyGrid(msg, map)
        %writeBinaryOccupancyGrid Write map to binary OccupancyGrid message

            validateattributes(msg.Data,{'int8'},{'vector'},'rosWriteBinaryOccupancyGrid');

            validateattributes(map,{'binaryOccupancyMap'}, {'nonempty', 'scalar'});

            % Fill in the meta information
            msg.Info.Resolution = single(1/map.Resolution);
            msg.Info.Width = uint32(map.GridSize(2));
            msg.Info.Height = uint32(map.GridSize(1));
            msg.Info.Origin.Position.X = double(map.GridLocationInWorld(1));
            msg.Info.Origin.Position.Y = double(map.GridLocationInWorld(2));

            % Get grid values from the BinaryOccupancyGrid
            values = ros.internal.SpecialMsgUtil.readFromBOG(map);
            reshapedValues = values(:);

            % Create returned message based on computed values
            msgAssigned = msg;
            msgAssigned.Data = int8(reshapedValues);
        end

        function msgAssigned = writeOccupancyGrid(msg, map)
        %writeOccupancyGrid Write map to OccupancyGrid message

            validateattributes(map,{'occupancyMap'}, {'nonempty', 'scalar'});
            validateattributes(msg.Data,{'int8'},{'vector'},'rosWriteOccupancyGrid');

            % Fill in the meta information
            msg.Info.Resolution = single(1/map.Resolution);
            msg.Info.Width = uint32(map.GridSize(2));
            msg.Info.Height = uint32(map.GridSize(1));
            msg.Info.Origin.Position.X = double(map.GridLocationInWorld(1));
            msg.Info.Origin.Position.Y = double(map.GridLocationInWorld(2));

            % Get grid values from the OccupancyGrid
            values = ros.internal.SpecialMsgUtil.readFromOG(map);
            reshapedValues = values(:);

            % Create returned message based on computed values
            msgAssigned = msg;
            msgAssigned.Data = int8(reshapedValues);
        end

        function q = readQuaternion(msg)
        %readQuaternion Extract quaternion from input message

            q = quaternion(msg.W, msg.X, msg.Y, msg.Z);
        end

        function varargout = showDetails(msg)
        %showDetails Display details of a message

        % Print structure
            structLevel = 1;
            detailedDisplay = ros.msg.internal.MessageDisplay.printStruct(msg, structLevel);

            % Return the detailed data string, if requested by user
            if nargout > 0
                varargout{1} = detailedDisplay;
                return
            end

            % Otherwise, print to console
            disp(detailedDisplay)
        end

        function plotHandles = plotPointCloud2(msg, varargin)
        %plotPointCloud2 Plot PointCloud2 given message

            validateattributes(msg.Header.FrameId,{'char'},{'scalartext'},'rosPlot');

            % Parse input arguments
            defaults.Parent = [];
            defaults.MarkerEdgeColor = 'invalid';
            args = ros.internal.SpecialMsgUtil.parseScatterArguments(defaults, varargin{:});

            % Get XYZ coordinates
            xyz = rosReadXYZ(msg);

            % Determine color of point cloud and call scatter3
            sHandle = ros.internal.SpecialMsgUtil.colorScatter(msg, xyz, defaults, args);

            % Properly scale the data
            axis(args.Parent, 'equal');
            grid(args.Parent, 'on');
            title(args.Parent, message('ros:mlroscpp:pointcloud:PointCloud').getString);
            xlabel(args.Parent, 'X');
            ylabel(args.Parent, 'Y');
            zlabel(args.Parent, 'Z');

            % Show the camera toolbar and enable the orbit camera
            figureHandle = ancestor(args.Parent, 'figure');
            cameratoolbar(figureHandle, 'Show');
            cameratoolbar(figureHandle, 'SetMode', 'orbit');
            cameratoolbar(figureHandle, 'ResetCamera');

            % See http://www.ros.org/reps/rep-0103.html#axis-orientation
            % for information on ROS coordinate systems
            if contains((msg.Header.FrameId), '_optical')
                % For point clouds recorded from a camera system, a
                % different axis orientation applies: z forward, x right,
                % and y down.

                % Change the principal axis to -Y and show a good
                % perspective view
                cameratoolbar(figureHandle, 'SetCoordSys','Y');
                set(args.Parent, 'CameraUpVector', [0 -1 0]);
                camorbit(args.Parent, -110, -15, 'data', [0 1 0]);
            else
                % By default, use the standard ROS convention: x forward, y
                % left, and z up.
                cameratoolbar(figureHandle, 'SetCoordSys','Z');
                set(args.Parent, 'CameraUpVector', [0 0 1]);
                camorbit(args.Parent, -70, -15, 'data', [0 0 1]);
            end

            % Use the rotation icon instead of the standard pointer
            ptrData = setptr('rotate');
            set(figureHandle, ptrData{:});

            % Return the scatter handle if requested by the user
            if nargout > 0
                plotHandles = sHandle;
            end
        end

        function plotHandles = plotLaserScan(msg, varargin)
        %plotLaserScan Plot LaserScan given message

            validateattributes(msg.Ranges,{'single','uint16','double','uint32'},{'vector'},'rosPlot');
            validateattributes(msg.RangeMax,{'single','uint16','double','uint32'},{'scalar'},'rosPlot');

            if isa(msg.RangeMax,'double')
                msg.RangeMax = single(msg.RangeMax);
            end
            if isa(msg.Ranges,'double')
                msg.Ranges = single(msg.Ranges);
            end

            [cart, cartAngles] = rosReadCartesian(msg);

            % Parse input arguments
            % By default, use the maximum range for scaling
            defaults.Parent = [];

            % Only take the maximum range as default, if it is finite and
            % positive. Otherwise, use the maximum measured range as default.
            if isfinite(msg.RangeMax) && msg.RangeMax > 0
                defaults.MaximumRange = msg.RangeMax;
            else
                defaults.MaximumRange = max(msg.Ranges);
            end

            % keep metaClassObj
            metaClassObj.Name = 'laser';
            lineHandles = robotics.utils.internal.plotScan(metaClassObj, cart, cartAngles, defaults, varargin{:});

            % Return the line handle if requested by the user
            if nargout > 0
                plotHandles = lineHandles;
            end
        end

        function map = readOccupancyMap3D(msg)
        %readOccupancyMap3D Return an occupancyMap3D object

        % Error out for non OcTree Id (i.e., for ColorOcTree)
            if ~strcmp('OcTree', msg.Id)
                error(message('ros:mlroscpp:octomap:UnSupportedOctreeFormat', msg.Id));
            end

            % Construct the occupancyMap3D object and populate the data
            if isempty(coder.target)
                try
                    % readOccupancyMap3D requires both ROS Toolbox and NAV
                    % Toolbox. Since MapIO comes from backend, this call to
                    % occupancyMap3D ensures error message containing
                    % information about the Navigation Toolbox.
                    map = nav.algs.internal.MapIO.deserializeROSMsgData(msg.Binary, msg.Resolution, msg.Data);
                catch ex
                    validateattributes(msg.Data,{'int8'},{'vector'},'rosReadOccupancyMap3D');
                    validateattributes(msg.Id,{'char'},{'vector'},'rosReadOccupancyMap3D');
                    validateattributes(msg.Binary,{'logical'},{'scalar'},'rosReadOccupancyMap3D');
                    validateattributes(msg.Resolution,{'uint32','double'},{'scalar'},'rosReadOccupancyMap3D');
                    % Do license check by instantiating basic Navigation Toolbox class
                    occupancyMap3D;
                    % If not a license issue, propagate exception
                    rethrow(ex)
                end
            end
        end

        function msgProperties = tfMsgProps()
        %tfMsgProps returns proper message properties for
        %geometry_msgs/TransformStamped messages

            persistent rostfProps
            if isempty(rostfProps)
                % Field names for tf message and supported entity
                % messages
                rostfProps = struct('header',{'Header','header'},'frame_id',{'FrameId','frame_id'}, ...
                                    'child_frame_id',{'ChildFrameId','child_frame_id'},'transform',{'Transform','transform'},...
                                    'translation', {'Translation','translation'},'rotation',{'Rotation','rotation'},...
                                    'x',{'X','x'},'y',{'Y','y'},'z',{'Z','z'},'w',{'W','w'},'point',{'Point','point'},...
                                    'quaternion',{'Quaternion','quaternion'},'vector',{'Vector','vector'},...
                                    'pose',{'Pose','pose'},'orientation',{'Orientation','orientation'},...
                                    'position',{'Position','position'});
            end

            % Field names for ROS 1 msg structs
            msgProperties = rostfProps(1);
        end

        function msg = writeXYZ(msg, xyz)
        %writeXYZ Write points in (x,y,z) coordinates to the message
        %   writeXYZ(MSG, XYZ) writes the Nx3 matrix of 3D point
        %   coordinates to the point cloud message struct MSG.
        %   Each input row in XYZ is a 3-vector representing one 3D point.
        %   The number of rows of XYZ needs to be equal to the number of
        %   points currently stored in MSG.
        %   This function completely overwrite any existing XYZ point
        %   information and replaces it with the points given in XYZ.

        % Save local information for the message
            width = msg.Width;
            height = msg.Height;

            validateattributes(xyz, {'numeric'}, {'nonempty', 'ncols', 3, 'nrows', ...
                                                  height * width}, 'writeXYZ', 'xyz');

            % Get PointField information for x, y, and z data. If these do
            % not exist an error will be displayed.
            xIdx = ros.internal.SpecialMsgUtil.getFieldNameIndex(msg,'x');
            yIdx = ros.internal.SpecialMsgUtil.getFieldNameIndex(msg,'y');
            zIdx = ros.internal.SpecialMsgUtil.getFieldNameIndex(msg,'z');

            % Get byte index only once (this is expensive)
            byteIdx = ros.internal.SpecialMsgUtil.getByteIndexForField(msg, xIdx).';

            % Calculate the byte offsets for the different fields
            % This helps with performance, since we can re-use the
            % already calculated byte index
            xOff = double(msg.Fields(xIdx).Offset);
            yOff = double(msg.Fields(yIdx).Offset) - xOff;
            zOff = double(msg.Fields(zIdx).Offset) - xOff;

            % Write XYZ byte array back to message data property
            writeDataToField(xIdx, xyz(:,1), byteIdx);
            writeDataToField(yIdx, xyz(:,2), byteIdx + yOff);
            writeDataToField(zIdx, xyz(:,3), byteIdx + zOff);

            function writeDataToField(fieldIdx, fieldData, byteIdx)
            %writeDataToField Write data to a point field

                if isempty(coder.target)
                    %TypeConversion - Object handling ROS <--> MATLAB type
                    %conversions
                    TypeConversion = ros.msg.sensor_msgs.internal.PointCloud2Types;
                    % Recover MATLAB data type of field elements
                    datatype = msg.Fields(fieldIdx).Datatype;
                    mlFieldType = TypeConversion.rosToMATLABType(datatype);
                else
                    mlFieldType = 'single';
                end

                % Cast the input to the expected type
                if ~isa(fieldData, mlFieldType)
                    fieldData = cast(fieldData,mlFieldType);
                end
                % Treat field data as byte array
                fieldDataBytes = typecast(fieldData, 'uint8');

                % Now write the field data to the point field
                msg.Data(byteIdx) = fieldDataBytes;
            end
        end
    end

    methods (Static, Access = private)
        function fieldIdx = getFieldNameIndex(msg, fieldName)
        %getFieldNameIndex Get index of field in PointField array

            allFieldNames = ros.internal.SpecialMsgUtil.getAllFieldNames(msg);
            fieldIdx = 0;
            for i=1:length(allFieldNames)
                if strcmp(allFieldNames{i},fieldName)
                    fieldIdx = i;
                end
            end

            missingXYZ = isequal(fieldIdx,0) && (strcmp(fieldName,'x') || strcmp(fieldName,'y') || strcmp(fieldName,'z'));
            missingRGB = isequal(fieldIdx,0) && (strcmp(fieldName,'rgb'));
            missingOthers = isequal(fieldIdx,0) && (~(missingXYZ && missingRGB));

            coder.internal.errorIf(missingXYZ,'ros:mlroscpp:pointcloud:InvalidXYZData');
            coder.internal.errorIf(missingRGB,'ros:mlroscpp:pointcloud:NoColorData');
            coder.internal.errorIf(missingOthers,'ros:mlroscpp:pointcloud:InvalidFieldName',...
                                   fieldName, strjoin(allFieldNames, ', '));
        end

        function [byteIdx, pointIdxIsValid] = getByteIndexForField(msg, fieldIdx, pointIndices, count)
        %getByteIndexForField Get a vector of bytes indices for field
        %   at specific points

        % Save local information for the message
            offset = msg.Fields(fieldIdx).Offset;
            datatype = msg.Fields(fieldIdx).Datatype;

            %TypeConversion - Object handling ROS <--> MATLAB type conversions
            TypeConversion = ros.msg.sensor_msgs.internal.PointCloud2Types;

            if nargin < 3
                pointIndices = 1:msg.Height*msg.Width;
            end
            if nargin < 4
                count = msg.Fields(fieldIdx).Count;
            end

            % Number of points requested
            numPoints = numel(pointIndices);

            % Compute actual number of available points (accounting for
            % potential truncation)
            numPointsActual = min(msg.Height*msg.Width, fix(numel(msg.Data)/double(msg.PointStep)));

            % Recover field offset and MATLAB data type of field elements
            [~, numBytes] = TypeConversion.rosToMATLABType(datatype);
            numBytes = numBytes * double(count);

            % Extract the bytes corresponding to this field for all points
            byteIdx = zeros(numPoints, numBytes);
            pointIdxIsValid = (0 < pointIndices) & (pointIndices <= numPointsActual);
            validPointIndices = pointIndices(pointIdxIsValid);
            startIndices = double(offset ...
                                  + msg.PointStep*cast(validPointIndices(:)-1,'like',msg.PointStep));
            byteIdx(pointIdxIsValid,:) = bsxfun(@plus, startIndices, 1:numBytes);
            if nargout < 2
                byteIdx = byteIdx(pointIdxIsValid,:);
            end
        end

        function fieldPoints = readFieldFromData(data, byteIdx, pointIdxIsValid, mlType, count)
        %readFieldFromData Read data based on given field name
        %   This is a generic function for reading data from any field
        %   name specified
        %
        %   This function returns an NxC array of values. N is the
        %   number of points in the point cloud and C is the number of
        %   values that is assigned for every point in this field. In
        %   most cases, C will be 1.

            numPoints = numel(pointIdxIsValid);
            % Initialize output
            rawData = reshape(data(byteIdx(pointIdxIsValid,:)'),[],1);

            if isempty(coder.target)
                validPoints = reshape(typecast(rawData, mlType), count, []).';
                if any(~pointIdxIsValid(:))
                    if any(strcmp(mlType, {'single','double'}))
                        fieldPoints = NaN(numPoints, count, mlType);
                    else
                        fieldPoints = zeros(numPoints, count, mlType);
                    end
                    fieldPoints(pointIdxIsValid, :) = validPoints;
                else
                    fieldPoints = validPoints;
                end
            else
                % The Generated code would only support single (float) type
                % of data
                validPoints = reshape(typecast(rawData, 'single'), count, []).';
                if any(~pointIdxIsValid(:))
                    fieldPoints = NaN(numPoints, count, 'single');

                    fieldPoints(pointIdxIsValid, :) = validPoints;
                else
                    fieldPoints = validPoints;
                end
            end
        end

        function fieldPoints = readRGBFieldFromData(data, byteIdx, pointIdxIsValid, count)
        %readRGBFieldFromData Read data based on given rgb field
        %   This is a generic function for reading data from any field
        %   name specified
        %
        %   This function returns an NxC array of values. N is the
        %   number of points in the point cloud and C is the number of
        %   values that is assigned for every point in this field. In
        %   most cases, C will be 1.

            numPoints = numel(pointIdxIsValid);
            % Initialize output
            rawData = reshape(data(byteIdx(pointIdxIsValid,:)'),[],1);
            validPoints = reshape(rawData, count, []).';

            if any(~pointIdxIsValid(:))
                fieldPoints = zeros(numPoints, count, 'uint8');
                fieldPoints(pointIdxIsValid, :) = validPoints;
            else
                fieldPoints = validPoints;
            end
        end

        function args = parseReadCartesianArguments(defaults, varargin)
        %parseReadCartesianArguments Parse arguments for readCartesian
        %   function

        % Return right away if there is nothing to be parsed

            if isempty(varargin)
                args = defaults;
            end

            % Parse the range limits
            nvPairs = struct('RangeLimits',uint32(0));
            pOpts = struct('PartialMatching',true,'CaseSensitivity',false);
            pStruct = coder.internal.parseParameterInputs(nvPairs,pOpts,varargin{1:end});

            rangeLimits = coder.internal.getParameterValue(pStruct.RangeLimits,defaults.RangeLimits,varargin{1:end});
            validateattributes(rangeLimits,{'numeric'},...
                               {'vector', 'numel', 2, 'nonnan', 'nondecreasing'},'LaserScan','RangeLimits');
            args.RangeLimits = rangeLimits;
        end

        function validateOccGridMsgValues(msg)
        %validateOccGridMsgValues Validate message values

            UnknownValueInMsg = -1;
            MaxValueInMsg = 100;
            % Error if height and width do not match number of elements in
            % the data
            if isempty(coder.target)
                if numel(msg.Data) ~= msg.Info.Height*msg.Info.Width
                    error(message('ros:mlroscpp:occgrid:InvalidDataSize', ...
                                  num2str(numel(msg.Data)), num2str(msg.Info.Height), ...
                                  num2str(msg.Info.Width)));
                end
            end

            % Error if resolution is zero
            validateattributes(msg.Info.Resolution, {'numeric'}, ...
                               {'nonempty', 'nonnan', 'positive'}, ...
                               'readOccupancyGrid', 'Resolution');

            % Validate that Data is between [0, 100] or -1
            validateattributes(msg.Data, {'numeric'}, ...
                               {'nonempty', '>=' UnknownValueInMsg, '<=', MaxValueInMsg}, ...
                               'readOccupancyGrid', 'Data');

            % Validate origin
            origin = [msg.Info.Origin.Position.X, msg.Info.Origin.Position.Y];
            validateattributes(origin, {'numeric'}, {'nonnan', 'finite'}, ...
                               'readOccupancyGrid', 'Origin');
        end

        function opts = parseReadBOGInputs(varargin)
        %parseReadBOGInputs Input parser for read method

        % default occupancy threshold and value for unknown
            opts.OccupancyThreshold = 50;
            opts.ValueForUnknown = 0;

            if ~isempty(varargin)
                opts.OccupancyThreshold = varargin{1};
                validateattributes(opts.OccupancyThreshold, {'double'}, ...
                                   {'nonnegative', 'scalar', 'finite'});
            end

            if length(varargin)>1
                opts.ValueForUnknown = varargin{2};
                validateattributes(opts.ValueForUnknown, {'numeric','logical'}, ...
                                   {'nonnegative', 'scalar', 'binary'});
            end
        end

        function bog = writeToOG(og,values)
        %writeToOG Write values to binaryOccupancyMap or occupancyMap

        % Use reverse GridSize values because data will be transposed
        % in the next step
            matrix = reshape(values, og.GridSize(2), og.GridSize(1));

            % While writing the data needs to be transposed and flipped to
            % be compatible with ROS
            matrix = flip(matrix');
            [x, y] =  ndgrid(1:og.GridSize(1), 1:og.GridSize(2));
            og.setOccupancy([x(:) y(:)], matrix(:), 'grid');
            bog = og;
        end

        function values = readFromBOG(map)
        %readFromBOG Read values from binaryOccupancyMap

            occgrid = map.occupancyMatrix;

            %maxValueInMsg The maximum value in ROS nav_msgs/OccupancyGrid message
            maxValueInMsg = 100;

            % The data needs to be flipped and transposed to be compatible
            % with ROS
            values = double((flip(occgrid))');
            values(values == 1) = maxValueInMsg;
        end

        function values = readFromOG(map)
        %readFromOG Read values from occupancyMap

            occgrid = map.occupancyMatrix;

            %MaxValueInMsg The maximum value in ROS nav_msgs/OccupancyGrid message
            MaxValueInMsg = 100;

            % The data needs to be flipped and transposed to be compatible
            % with ROS
            values = (flip(occgrid))';
            values = values*MaxValueInMsg;
        end

        function args = parseScatterArguments(defaults, varargin)
        %parseScatterArguments Parse arguments for scatter3 function

        % Parse inputs
            parser = inputParser;

            % Cannot make any assumptions about data type
            addParameter(parser, 'Parent', defaults.Parent);

            addParameter(parser, 'MarkerEdgeColor', defaults.MarkerEdgeColor, ...
                         @(x) validateattributes(x, {'numeric', 'char'}, ...
                                                 {}, 'PointCloud2', 'MarkerEdgeColor'));

            % Return data
            parse(parser, varargin{:});
            args = parser.Results;

            % Delay the preparation of a new plot (and potentially the creation
            % of a new window until all other inputs have been parsed)
            if isnumeric(args.Parent) && isequal(args.Parent, defaults.Parent)
                args.Parent = newplot;
            end

            % Check that parent is a graphics Axes handle
            robotics.internal.validation.validateAxesHandle(args.Parent, ...
                                                            'ros:mlroscpp:pointcloud:ParentNotHandle');
        end

        function rgbExists = hasRGBData(msg)
        %hasRGBData Checks if this point cloud contains RGB data

            if isempty(coder.target)
                try
                    ros.internal.SpecialMsgUtil.getFieldNameIndex(msg,'rgb');
                    rgbExists = true;
                catch
                    rgbExists = false;
                end
            end
        end

        function scatterHandle = colorScatter(msg, xyz, defaults, args)
        %colorScatter Determine point cloud color and call scatter3
        %   This will be based on user input and the existence of
        %   RGB data in the point cloud.

        % If user specified a marker edge color use it for the scatter
        % plot
            if ~strcmp(args.MarkerEdgeColor, defaults.MarkerEdgeColor)
                color = args.MarkerEdgeColor;
                scatterHandle = scatter3(args.Parent, xyz(:,1), xyz(:,2), ...
                                         xyz(:,3), 1, '.', 'MarkerEdgeColor', color);
                return;
            end

            % Use point RGB color information (if it exists)
            if ros.internal.SpecialMsgUtil.hasRGBData(msg)
                color = rosReadRGB(msg);
                scatterHandle = scatter3(args.Parent, xyz(:,1), xyz(:,2), ...
                                         xyz(:,3), 1, color, '.');
                return;
            end

            % If no color specified by user, or no RGB information
            % available, use scatter3 defaults
            scatterHandle = scatter3(args.Parent, xyz(:,1), xyz(:,2), ...
                                     xyz(:,3), 1, '.');
        end
    end
end
