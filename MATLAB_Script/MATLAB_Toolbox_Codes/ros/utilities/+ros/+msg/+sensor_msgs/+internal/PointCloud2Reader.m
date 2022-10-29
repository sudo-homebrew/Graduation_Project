classdef PointCloud2Reader
%This class is for internal use only. It may be removed in the future.

%PointCloud2Reader
%
%   See also: ros.msg.sensor_msgs.PointCloud2

%   Copyright 2017-2020 The MathWorks, Inc.

%#codegen


    properties (Constant, Access = private)
        %TypeConversion - Object handling ROS <--> MATLAB type conversions
        TypeConversion = ros.msg.sensor_msgs.internal.PointCloud2Types
    end

    methods
        function xyz = readXYZ(obj, pc, xIdx, yIdx, zIdx, mlType, pointIndices)
        %readXYZ Returns the (x,y,z) coordinates of all points
            data = pc.Data;

            % Recover MATLAB data type of field elements
            if nargin < 6
                xMlType = obj.TypeConversion.rosToMATLABType( ...
                    pc.getFieldDatatype(xIdx));
                yMlType = obj.TypeConversion.rosToMATLABType( ...
                    pc.getFieldDatatype(yIdx));
                zMlType = obj.TypeConversion.rosToMATLABType( ...
                    pc.getFieldDatatype(zIdx));
            else
                xMlType = mlType;
                yMlType = mlType;
                zMlType = mlType;
            end

            if nargin < 7
                pointIndices = 1:pc.Width*pc.Height;
            end

            % Get byte index only once (this is expensive)
            [byteIdx, pointIdxIsValid] = obj.getByteIndexForField(pc, xIdx, pointIndices, 1);

            % Calculate the byte offsets for the different fields
            % This helps with performance, since we can re-use the
            % byte index computed below
            xOff = double(pc.getFieldOffset(xIdx));
            yOff = obj.relativeFieldOffset(pc, xOff, yIdx);
            zOff = obj.relativeFieldOffset(pc, xOff, zIdx);

            % Retrieve the XYZ data and concatenate into one matrix
            xyz = ...
                [obj.readFieldFromData(data, byteIdx, pointIdxIsValid, xMlType,1), ...
                 obj.readFieldFromData(data, byteIdx + yOff, pointIdxIsValid, yMlType,1), ...
                 obj.readFieldFromData(data, byteIdx + zOff, pointIdxIsValid, zMlType,1)];
        end

        function rgb = readRGB(obj, pc, rgbIdx, pointIndices)
        %readRGB Returns the RGB color matrix for all points

            if nargin < 4
                pointIndices = 1:pc.Height*pc.Width;
            end
            % Get byte index for the RGB field (this is expensive)
            [byteIdx, pointIdxIsValid] = obj.getByteIndexForField(pc, rgbIdx, pointIndices, 1);
            numPoints = numel(pointIndices);
            count = 4;
            rgbRaw = readFieldFromData(obj,  pc.Data, byteIdx, pointIdxIsValid, 'uint8', count);

            rgb = NaN(numPoints,3);

            % Scale values from [0,255] to [0,1] range
            % Also convert from BGR -> RGB
            rgb(pointIdxIsValid, :) = double(rgbRaw(pointIdxIsValid, 3:-1:1)) / 255;
        end

        function fieldData = readField(obj, pc, fieldIdx, mlType, count, pointIndices)
        %readField Read data based on given field index

        % Recover MATLAB data type of field elements when mlType
        % argument is empty or less than 4 arguments are specified
            if (nargin < 4) || isempty(mlType)
                datatype = pc.getFieldDatatype(fieldIdx);
                mlType = obj.TypeConversion.rosToMATLABType(datatype);
            end
            if nargin < 5
                count = pc.getFieldCount(fieldIdx);
            end
            if nargin < 6
                pointIndices = 1:pc.Width*pc.Height;
            end

            % Extract the bytes corresponding to this field
            [byteIdx, pointIdxIsValid] = obj.getByteIndexForField(pc, fieldIdx, pointIndices, count);
            fieldData = obj.readFieldFromData(pc.Data, ...
                                              byteIdx, pointIdxIsValid, mlType, count);
        end

        function [byteIdx, pointIdxIsValid] = getByteIndexForField(obj, pc, fieldIdx, pointIndices, count)
        %getByteIndexForField Get a vector of bytes indices for field
        %at specific points

            if nargin < 4
                pointIndices = 1:pc.Height*pc.Width;
            end
            if nargin < 5
                count = pc.getFieldCount(fieldIdx);
            end

            % Number of points requested
            numPoints = numel(pointIndices);

            % Compute actual number of available points (accounting for
            % potential truncation)
            numPointsActual = min(pc.Height*pc.Width, fix(numel(pc.Data)/double(pc.PointStep)));

            % Recover field offset and MATLAB data type of field elements
            offset = pc.getFieldOffset(fieldIdx);
            datatype = pc.getFieldDatatype(fieldIdx);
            [~, numBytes] = obj.TypeConversion.rosToMATLABType(datatype);
            numBytes = numBytes * double(count);

            % Extract the bytes corresponding to this field for all points
            pointStep = pc.PointStep;

            byteIdx = zeros(numPoints, numBytes);
            pointIdxIsValid = (0 < pointIndices) & (pointIndices <= numPointsActual);
            validPointIndices = pointIndices(pointIdxIsValid);
            startIndices = double(offset ...
                                  + pointStep*cast(validPointIndices(:)-1,'like',pointStep));
            byteIdx(pointIdxIsValid,:) = bsxfun(@plus, startIndices, 1:numBytes);
            if nargout < 2
                byteIdx = byteIdx(pointIdxIsValid,:);
            end
        end

        function fieldPoints = readFieldFromData(~, data, byteIdx, pointIdxIsValid, mlType, count)
        %readField Read data based on given field name
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
        end

        function targetOffset = relativeFieldOffset(~, pc, sourceOffset, fieldIndexTarget)
        %relativeFieldOffset Calculate the relative offset of a field
        %   TARGETOFFSET = relativeFieldOffset(OBJ, SOURCEOFFSET,
        %   FIELDINDEXTARGET) calculates the relative byte offset
        %   of the field with index FIELDINDEXTARGET from the
        %   position specified in SOURCEOFFSET. The relative offset
        %   will be returned in TARGETOFFSET.

            targetOffset = double(pc.getFieldOffset(fieldIndexTarget)) - sourceOffset;
        end
    end
end
