function [Data, Height, Width, DataLength, Encoding, EncodingLength, Step]...
        = WriteImageFncBlock(image, Data, EncodingStruct, SelectedEncodingInd) 
%This function is for internal use only. It may be removed in the future.

%WriteImageFncBlock creates ROS image data from input image and specified
% encoding
% EncodingStruct contains information on ROS image encoding
% SelectedEncodingInd specifies the index into NonBayerEncodings list
% Does not support bayer encodings

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    Height = uint32(size(image, 1));
    Width = uint32(size(image, 2));
    SelectedEncoding = ros.msg.sensor_msgs.internal.ImageEncoding.NonBayerEncodings{SelectedEncodingInd};
    Encoding = zeros(128, 1, 'uint8');
    Encoding(1:length(SelectedEncoding)) = uint8(SelectedEncoding);
    EncodingLength = uint32(length(SelectedEncoding));
    [Data1, imgInfo] = ros.msg.sensor_msgs.internal.ImageWriter.writeImage(EncodingStruct, image);
    Step = uint32(imgInfo.step);
    numBytes = length(typecast(image(1),'uint8'));
    DataLength = uint32(Height*Width*EncodingStruct.NumChannels*numBytes);
    Data(1:DataLength) = Data1(1:DataLength);

end
