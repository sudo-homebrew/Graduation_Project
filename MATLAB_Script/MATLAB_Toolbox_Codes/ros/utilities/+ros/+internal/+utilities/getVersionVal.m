function versionval = getVersionVal(verstr)
% This class is for internal use only. It may be removed in the future.

%   Copyright 2019 The MathWorks, Inc.
 
% getVersionVal will return a double value from a string which can be used
% to compare
% Simple trick to create a number out of version number
% 3.7.1 will be converted to 30000 + 700 + 1


strs = regexp(verstr,'\d+','match');
if numel(strs) < 1
    versionval = 0;
    return;
end
versionval = str2double(strs{1}) * 10000;
if numel(strs) > 1
    versionval = versionval + str2double(strs{2}) * 100;
    if numel(strs) > 2
        versionval = versionval + str2double(strs{3});
    end
end
