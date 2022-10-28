function subsystem =  createSubsystem(modelName,ArchitectureName,nodes)
%   Create a subsystem and return the full path to the subsystem..
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

% Create a sub-system
subsystem = [modelName,'/',ArchitectureName];
add_block('built-in/subsystem',subsystem);

%Create a mask for sub-system
p = Simulink.Mask.create(subsystem);
mask = 'disp(''Tracking Architecture'')';
p.set('Display',mask,'IconOpaque','opaque-with-ports');

%resize the sub-system based on number of inputs and outputs.
maxInput = numel(unique(cell2mat(cellfun(@(x)uint32(x.Inputs),nodes,'UniformOutput',false)))) + 1; % Additional time input
maxOutput = sum(cell2mat(cellfun(@(x)x.ToOutput,nodes,'UniformOutput',false)));
maxPorts = max([maxInput,maxOutput]); % This is required to adjust the final subsystem height
veriticalGap = 25;
blkWidth = 250;
blkHeight = maxPorts*veriticalGap;
fusion.simulink.internal.trackingArchitecture.resizeBlock(subsystem,blkHeight,blkWidth);
end