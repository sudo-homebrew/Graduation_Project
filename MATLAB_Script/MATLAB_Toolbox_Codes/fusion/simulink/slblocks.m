function blkStruct = slblocks
%SLBLOCKS Defines the Simulink library block representation
%   for the Sensor Fusion and Tracking Toolbox.

%   Copyright 2019 The MathWorks, Inc.

blkStruct.Name    = sprintf('Sensor Fusion and Tracking Toolbox');
blkStruct.OpenFcn = 'fusionlib';
blkStruct.MaskInitialization = '';

% Define the library list for the Simulink Library browser.
% Return the name of the library model and the name for it
Browser(1).Library = 'fusionlib';
Browser(1).Name    = 'Sensor Fusion and Tracking Toolbox';
Browser(1).IsFlat  = 0;

blkStruct.Browser = Browser;
