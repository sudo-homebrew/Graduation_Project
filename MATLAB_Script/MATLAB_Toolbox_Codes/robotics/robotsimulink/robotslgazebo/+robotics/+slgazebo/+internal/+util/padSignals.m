function y = padSignals(x)
%This function is for internal use only. It may be removed in the future.

%PADSIGNALS pad var-sized signals

%   Copyright 2019 The MathWorks, Inc.

    toProcess = fieldnames(x);
    y = x;
    for idx = 1:numel(toProcess)
        valToConvert = x.(toProcess{idx});
        if isempty(valToConvert)
            % pad zero with implicit cast
            y.(toProcess{idx})(end+1) = 0;
        elseif isstruct(valToConvert)
            y.(toProcess{idx}) = robotics.slgazebo.internal.util.padSignals(valToConvert);
        end
    end
end
