function y = convertToDouble(x)
%This function is for internal use only. It may be removed in the future.

%convertToDouble converts unsupported datatypes like 'uint64' 'int64' to
%double

%   Copyright 2019-2020 The MathWorks, Inc.

    toProcess = fieldnames(x);
    y = x;
    for idx = 1:numel(toProcess)
        valToConvert = x.(toProcess{idx});
        if isa(valToConvert,'int64')
            % pad zero with implicit cast
            y.(toProcess{idx}) = double(valToConvert);
        elseif isa(valToConvert,'uint64')
            y.(toProcess{idx}) = double(valToConvert);
        elseif isstruct(valToConvert)
            y.(toProcess{idx}) = robotics.slgazebo.internal.util.convertToDouble(valToConvert);
        end
    end
end
