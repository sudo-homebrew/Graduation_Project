function [status, result] = runros2py(cmdline)
%This function is for internal use only. It may be removed in the future.

%runros2py runs ros2 python commands

%   Copyright 2019 The MathWorks, Inc.

% Users can set the prefix path as an environment variable. If set that is
% where ros2 is installed.
amentPrefixPath = ['"' ros.ros2.internal.getAmentPrefixPath '"'];

[~,~,pyenvDir] = ros.ros2.internal.createOrGetLocalPython;
pyenvDir = ['"' pyenvDir '"'];

cmdmap = containers.Map({'win64','maci64','glnxa64'}, ...
    {['"' fullfile(fileparts(mfilename('fullpath')),'runros2py') '"'], ...use .bat file
     ['"' fullfile(fileparts(mfilename('fullpath')),'runros2py.sh') '"'],... use .sh
     ['"' fullfile(fileparts(mfilename('fullpath')),'runros2py.sh') '"']});
cmd = cmdmap(computer('arch'));

[status, result] = system([cmd, ' ', amentPrefixPath, ' ', pyenvDir, ' ', cmdline]);
if isequal(status,0) %no error
    result = strtrim(result);
end
