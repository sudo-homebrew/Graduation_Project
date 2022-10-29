function blkHndl = addBlockToModel(matlabObject,varargin)
%   Add a block corresponding to the matlabObject in a Simulink model.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

% ADDBLOCKTOMODEL(..., 'Model', MODEL), allows you to export the node to an
% existing Simulink model MODEL. MODEL can be the name or numeric handle of
% the Simulink model. If a Simulink model with name MODEL does not exist, a
% new model is created with name MODEL.
%
% ADDBLOCKTOMODEL(..., 'BlockName', Name), allows you to specify a name,
% NAME for the Simulink block.
%
% ADDBLOCKTOMODEL(..., 'Position', POS), allows you to specify block
% position, POS, in the model. POS must be a vector of coordinates, in
% pixels: [left top right bottom]
%
% ADDBLOCKTOMODEL(..., 'OpenModel', TF), allows you to specify a flag, TF,
% to indicate if the model should be opened after exporting the node. The
% default value for OpenModel is True.

[model,blockName,position,openModel] =  parseInputs(varargin{:});

if isempty(model)
    modelName = fusion.simulink.internal.initializeSimulinkModel();
else
    validateattributes(model, {'char','string','numeric'},{'nonempty'},'addBlockToModel','Model');
    modelName = fusion.simulink.internal.initializeSimulinkModel(model);
end

%Find the block type from class.
blkType = fusion.simulink.internal.fusionBlocksAndObjectMapping(class(matlabObject));

if isempty(blockName)
    % Default name for the block. Same as the block in the library.
    % blkType contains library name so extract blockName by removing
    % library name.
    blockName = extractAfter(blkType,'/');
end

%Add block into model
validateattributes(blockName, {'char','string'},{'nonempty'},'addBlockToModel','BlockName');
try
    blkHndl = add_block(blkType,strcat(modelName,'/',blockName));
catch ME
    if strcmpi(ME.identifier,'Simulink:Commands:AddBlockCantAdd')
        error(message('fusion:simulink:exportToSimulink:FailedToAddBlock',blockName,modelName));               
    else
        rethrow(ME);
    end
end


validateattributes(openModel, {'logical'},{},'addBlockToModel','OpenModel');
if openModel
    %open the Simulink model.
    open_system(modelName);
end

if ~any(isnan(position))
    %Position is specified. Set block position.
    validateattributes(position, {'numeric'},{'size', [1,4]},'addBlockToModel','Position');
    set_param(blkHndl,'position',position);
end
end

function [model,blockName,position,openModel] =  parseInputs(varargin)
%Parse input to support NV pair arguments.
p = inputParser;
addParameter(p, 'Model',char.empty);
addParameter(p, 'BlockName', char.empty);
addParameter(p, 'Position', NaN(1,4));
addParameter(p, 'OpenModel', true);

parse(p, varargin{:});
model  = p.Results.Model;
blockName = p.Results.BlockName;
position =  p.Results.Position;
openModel =  p.Results.OpenModel;
end