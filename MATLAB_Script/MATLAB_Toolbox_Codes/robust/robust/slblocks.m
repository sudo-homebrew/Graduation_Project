function blkStruct = slblocks
% SLBLOCKS  Defines the Simulink library block representation
%           for the Simulink Parameter Estimation

% Copyright 2006 The MathWorks, Inc.

blkStruct.Name    = sprintf('Robust\nControl\nToolbox');
blkStruct.OpenFcn = 'RCTblocks';
blkStruct.MaskInitialization = '';
blkStruct.MaskDisplay = '';

% Define the library list for the Simulink Library browser.
% Return the name of the library model and the name for it.
Browser(1).Library = 'RCTblocks';
Browser(1).Name    = 'Robust Control Toolbox';
Browser(1).IsFlat  = 1; % Is this library "flat" (i.e. no subsystems)?

blkStruct.Browser = Browser;

% Define information for model updater for updating obsolete blocks
blkStruct.ModelUpdaterMethods.fhUpdateModel = @rctBlocksSlupdateHelper;

