function [status, result] = runros2cmd(cmdline)
%runros2cmd runs ros2 command

%   Copyright 2019 The MathWorks, Inc.

% Users can set the prefix path as an environment variable. If set that is
% where ros2 is installed.
amentPrefixPath = ['"' ros.ros2.internal.getAmentPrefixPath '"'];

[~,~,pyenvDir] = ros.ros2.internal.createOrGetLocalPython;
pyenvDir = ['"' pyenvDir '"'];

cmdmap = containers.Map({'win64','maci64','glnxa64'}, ...
    {['"' fullfile(fileparts(mfilename('fullpath')),'runros2cmd') '"'], ...use .bat file
     ['"' fullfile(fileparts(mfilename('fullpath')),'runros2cmd.sh') '"'],... use .sh
     ['"' fullfile(fileparts(mfilename('fullpath')),'runros2cmd.sh') '"']});
cmd = cmdmap(computer('arch'));

[status, result] = system([cmd, ' ', amentPrefixPath, ' ', pyenvDir, ' ', cmdline]);
if isequal(status,0) %no error
    result = strtrim(result);
end
