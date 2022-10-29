function rtwTargetInfo(tr)
%RTWTARGETINFO Register toolchain and target

% Copyright 2020 The MathWorks, Inc.
    tr.registerTargetInfo(@loc_createToolchain);
end

% %--------------------------------------------------------------------------
function config = loc_createToolchain
    rootDir = fileparts(mfilename('fullpath'));
    config = coder.make.ToolchainInfoRegistry; % initialize
    archName = computer('arch');
    config(end+1).Name                = 'Catkin';
    config(end).Alias               = {['CATKIN_TOOLS_', upper(archName)], 'catkinProject'};
    config(end).TargetHWDeviceType  = {'*'};
    config(end).FileName            = fullfile(rootDir, ['catkinproject_catkin_', archName, '_v1.0.mat']);
    config(end).Platform            = {archName};
end
