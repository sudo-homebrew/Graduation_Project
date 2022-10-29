function out = convertStringToUint8(in)
%This function is for internal use only. It may be removed in the future.

%convertStringToUint8 converts string fields in a struct to uint8 array. If
%the field is string arrays, then it is converted to cell array of uint8
%array

%   Copyright 2019-2020 The MathWorks, Inc.

    toProcess = fieldnames(in);
    for idx = 1:numel(toProcess)

        multiData = [in.(toProcess{idx})]; % Need for repeated struct

        valToConvert = convertStringsToChars(multiData);

        if ischar(valToConvert)
            out.(toProcess{idx}) = uint8(valToConvert);
        elseif iscellstr(valToConvert)
            out.(toProcess{idx}) = uint8(char(join(valToConvert,newline)));
        elseif isstruct(valToConvert) || isa(valToConvert, 'robotics.slgazebo.internal.msgs.Gazebo_msgs_Time') || isa(valToConvert, 'robotics.slgazebo.internal.msgs.Gazebo_Msgs')
            out.(toProcess{idx}) = robotics.gazebo.internal.customMessageSupport.convertStringToUint8(valToConvert);
        else
            out.(toProcess{idx}) = valToConvert;
        end
    end
end
