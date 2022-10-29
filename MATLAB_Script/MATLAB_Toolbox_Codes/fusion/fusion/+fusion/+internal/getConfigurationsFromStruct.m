function config = getConfigurationsFromStruct(configStruct)
% This is an internal function and may be removed or updated in the future
%
% This function creates trackingSensorConfiguration using the input
% structure fields.
%   configStruct - toStruct(trackingSensorConfiguration(1))
%
%  Example:
%   configStruct = toStruct(trackingSensorConfiguration(1));
%   config = fusion.internal.getConfigurationsFromStruct(configStruct);
%
% Copyright 2021 The MathWorks, Inc.

%#codegen
num = numel(configStruct);
config = cell(1,num);
for i = 1:num
    config{i} = generateConfiguration(configStruct(i));
end
end

function config = generateConfiguration(s)
fnNames = fieldnames(s);
fnValues = struct2cell(s);
args = cell(1,2*numel(fnNames));
for i = 1:numel(fnNames)
    args{2*i-1} = fnNames{i};
    args{2*i} = fnValues{i};
end
config = trackingSensorConfiguration(args{:});
end

