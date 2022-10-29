function out = convertStringToUint8(in)
%This function is for internal use only. It may be removed in the future.

%convertStringToUint8 converts string fields in a struct to uint8 array. If
%the field is string arrays, then it is converted to cell array of uint8
%array

%   Copyright 2019 The MathWorks, Inc.

    out = robotics.gazebo.internal.customMessageSupport.convertStringToUint8(in);

end
