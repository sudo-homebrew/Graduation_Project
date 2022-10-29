function blkStruct = slblocks
% SLBLOCKS Defines the block library for Navigation Toolbox

%   Copyright 2018-2019 The MathWorks, Inc.

blkStruct.Name    = sprintf('Navigation Toolbox');
blkStruct.OpenFcn = 'navlib';
blkStruct.MaskInitialization = '';
blkStruct.MaskDisplay = 'disp(''Navigation Toolbox'')';

% Define the library list for the Simulink Library browser.
% Return the name of the library model
Browser.Library = 'navlib';
Browser.Name    = 'Navigation Toolbox';
Browser.IsFlat  = 0; % Is this library "flat" (i.e. no subsystems)?
blkStruct.Browser = Browser;

