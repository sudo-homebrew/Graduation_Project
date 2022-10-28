function blkStruct = slblocks
% SLBLOCKS Defines the block library for ROS Toolbox

%   Copyright 2019 The MathWorks, Inc.

productName = 'ROS Toolbox';
blkStruct.Name    = productName;
blkStruct.OpenFcn = 'roslib';
blkStruct.MaskInitialization = '';
blkStruct.MaskDisplay = ['disp(''' productName ''')'];
% Define the library list for the Simulink Library browser.
% Return the name of the library model
Browser(1).Library = 'roslib';
Browser(1).Name    = productName;
Browser(1).IsFlat  = 0; % Is this library "flat" (i.e. no subsystems)?
blkStruct.Browser = Browser;
