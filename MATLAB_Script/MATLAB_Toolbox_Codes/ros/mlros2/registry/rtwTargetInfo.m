function rtwTargetInfo(tr)
%RTWTARGETINFO Register toolchain and target

% Copyright 2019 The MathWorks, Inc.

    tr.registerTargetInfo(@loc_createToolchain);
end

% %--------------------------------------------------------------------------
function config = loc_createToolchain

    config = coder.make.ToolchainInfoRegistry; % initialize
    config(end)                     = coder.make.ToolchainInfoRegistry;
    config(end).Name                = 'Colcon Tools';
    config(end).Alias               = 'colconProject';
    config(end).FileName            = fullfile(fileparts(mfilename('fullpath')), ['colconproject_colcon_', computer('arch'), '_v1.0.mat']);
    config(end).TargetHWDeviceType  = {'*'};
    config(end).Platform            = {'maci64', 'win64', 'glnxa64'};
end
