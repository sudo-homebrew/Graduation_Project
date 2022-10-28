function rmdirusingsys(dirToRemove)
% This class is for internal use only. It may be removed in the future.
 
% rmdir may sometimes fail on Linux and Mac. So using system command to rm
% directories with contents

% Copyright 2019 The MathWorks, Inc.

cmdmap = containers.Map({'win64','maci64','glnxa64'}, ...
    {'rmdir /s /q ', ...
     'rm -Rf ', ...
     'rm -Rf '});
[stat, res] = system([cmdmap(computer('arch')), dirToRemove]);
if stat ~= 0
    error(message('MATLAB:RMDIR:SomeDirectoriesNotRemoved',res));
end
