function tracksOut = formatOutput(obj)
%formatOutput  Formats the output for the tracking architecture
% tracksOut = formatOutput(OBJ) returns the cell array of outputs to be
% provided from the tracking architecture, OBJ, at this step.

% Copyright 2020 The MathWorks, Inc.

tracksOut = cell(1,numel(obj.OutputSelection));
for i = 1:numel(obj.OutputSelection)
    tracksOut{i} = obj.pCurrentStepTracks{obj.pNodeIndices==obj.OutputSelection(i)};
end
end