function truthIDs = defaultTruthIdentifier(x)
%DEFAULTTRUTHIDENTIFIER default truth identifier implementation
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

if isempty(x)
    truthIDs = nan(0,1);
else
    if coder.target('MATLAB')
        truthIDs = vertcat(x.PlatformID);
    else
        truthIDs = zeros(numel(x),1); % Always double. Cast it to right data type in caller
        for i = 1:numel(x)
            truthIDs(i) = x(i).PlatformID;
        end
    end
end
