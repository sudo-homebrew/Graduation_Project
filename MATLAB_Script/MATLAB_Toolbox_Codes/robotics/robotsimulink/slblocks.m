function blkStruct = slblocks
% SLBLOCKS Defines the block library for RST

%   Copyright 2014-2016 The MathWorks, Inc.

blkStruct.Name    = sprintf('Robotics System\nToolbox');
blkStruct.OpenFcn = 'robotsyslib';
blkStruct.MaskInitialization = '';
blkStruct.MaskDisplay = 'disp(''Robotics System Toolbox'')';

% Define the library list for the Simulink Library browser.
% Return the name of the library model and the name for it
Browser.Library = 'robotsyslib';
Browser.Name    = 'Robotics System Toolbox';
Browser.IsFlat  = 0; % Is this library "flat" (i.e. no subsystems)?
blkStruct.Browser = Browser;

% End of slblocks.m
