function [Height, Width, FPFieldLengths, PointStep,...
          RowStep, DataLength, Data, FPNames,...
          FPNameLengths, FPOffsets, FPDatatypes, FPCounts]...
          = WritePointCloudFncBlock(MaxDataLength, NumFields, NameLength, XYZ, RGB, FieldNamesStruct) %#codegen
%This funtion is for internal use only. It may be removed in the future.

%WriteImageFncBlock creates ROS point cloud data from input XYZ and RGB
%FieldNamesStruct amps the PointField name field to the encoding
% (e.g. single)

%   Copyright 2021 The MathWorks, Inc.

% convert RGB from double to uint8
    RGBConverted = uint8(RGB*255);
    % assign metadata
    if ismatrix(XYZ) % assumes that the points are unordered
        NumPts = size(XYZ, 1);
        Height = uint32(1);
        Width = uint32(NumPts);
        RGBFlat = reshape(RGBConverted', [], 1);
        XYZunordered = XYZ;
    else % assumes that the points are ordered
        Height = uint32(size(XYZ, 1));
        Width = uint32(size(XYZ, 2));
        NumPts = Height*Width;
        RGBFlat = reshape(permute(RGBConverted, [3 2 1]), [], 1);
        XYZunordered = reshape(permute(XYZ, [2 1 3]), [], 3);
    end
    Data = zeros(MaxDataLength, 1, 'uint8');
    FNames = fieldnames(FieldNamesStruct);
    % FP prefix desingates sensor_msgs/PointField properties
    % allocate properties
    FPFieldLengths = uint32(length(FNames));
    FPNames_M = zeros(NumFields, NameLength, 'uint8');
    FPNameLengths = zeros(NumFields, 1, 'uint32');
    FPOffsets = zeros(NumFields, 1, 'uint32');
    FPDatatypes  = zeros(NumFields, 1, 'uint8');
    FPCounts = zeros(NumFields, 1, 'uint32');
    % NumBytesArray used to calulate byte offset
    NumBytesArray = zeros(length(FNames), 1);
    % assign sensor_msgs/PointField properties
    TotalBytes = 0;
    for idx = 1:length(FNames)
        FName = FNames{idx};
        Type = FieldNamesStruct.(FName);
        FPNameLengths(idx) = uint32(numel(FName));
        FPNames_M(idx, 1:numel(FName)) = uint8(FName);
        [RosType, numBytes] = ros.msg.sensor_msgs.internal.PointCloud2Types.matlabToROSType(Type);
        NumBytesArray(idx) = numBytes;
        FPOffsets(idx) =  uint32(TotalBytes);
        TotalBytes = TotalBytes + numBytes;
        FPDatatypes(idx) = uint8(RosType);
        FPCounts(idx) = uint32(1);
    end
    % flatten names matrix to one dim
    FPNames = reshape(FPNames_M', [], 1);
    % assign metadata
    PointStep = uint32(sum(NumBytesArray));
    RowStep = uint32(sum(NumBytesArray));
    DataLength = uint32(RowStep*NumPts);
    % generate index matrix for convenient indexing
    StartIndices = repmat( double( 1 : RowStep : DataLength), 8, 1);
    Inds = StartIndices + repmat( ((0:7)'), 1, size(StartIndices, 2)); % no data type has more than 8 bytes
                                                                       % assign to Data array
    for idx = 1:length(FNames)
        FName = FNames{idx};
        byteOffset = double(FPOffsets(idx));
        switch FName
          case 'x'
            Data(Inds(1 : NumBytesArray(idx), :) + byteOffset) = typecast(XYZunordered(:, 1),'uint8');
          case 'y'
            Data(Inds(1 : NumBytesArray(idx), :) + byteOffset) = typecast(XYZunordered(:, 2),'uint8');
          case 'z'
            Data(Inds(1 : NumBytesArray(idx), :) + byteOffset) = typecast(XYZunordered(:, 3),'uint8');
          case 'rgb'
            Data(Inds(NumBytesArray(idx)-1:-1:1 , :) + byteOffset) = typecast(RGBFlat,'uint8');
          case 'rgba'
            Data(Inds([NumBytesArray(idx)-1:-1:1, NumBytesArray(idx)], :) + byteOffset) = typecast(RGBFlat,'uint8');
        end
    end

end
