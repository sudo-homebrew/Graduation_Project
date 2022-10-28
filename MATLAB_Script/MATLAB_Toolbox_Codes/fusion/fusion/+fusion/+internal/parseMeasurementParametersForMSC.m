function [isRect,orient,hasEl] = parseMeasurementParametersForMSC(hasZStates,funcName,classToUse,varargin)
% This function is for internal use only and may be removed in a future
% release.
% parseMeasurementParametersForMSC parses the measurement parameters
% provided as inputs in the MeasurementFcn and MeasurementJacobianFcn.

% Copyright 2018 The MathWorks, Inc.

% The default values for the parser for measurement models are Rectangular
% with flags specfieid as true, which is not correct for MSC.
% This function creates the and parses the measurement parameters with
% correct defaults for for MSC functions.

%#codegen

if numel(varargin) > 0
    candidateStruct = varargin{1};
    if isstruct(candidateStruct)
        argsinCell = matlabshared.tracking.internal.fusion.measmodelsvalidateoptionalstruct(funcName, classToUse, candidateStruct);
        % Change default frame to spherical.
        if ~isfield(candidateStruct(1),'Frame')
            argsinCell{1} = 'spherical';
        end
        % Change default HasElevation to hasZStates
        if ~isfield(candidateStruct(1),'HasElevation')
            argsinCell{6} = hasZStates;
        end
    else %assume that the arguments are in a cell array
        % args can only be frame and orientation matrix. Make sure all
        % other flags are specied correctly as defaults. Specifying empty
        % will result in defaults.
        if numel(varargin) > 1
            argsinCell = {varargin{1},[],[],varargin{2},true,hasZStates};
        else
            argsinCell = {varargin{1},[],[],[],true,hasZStates};
        end
    end
else
    % default frame is spherical, default hasRange is false, default hasAz
    % is true, default hasEl is hasZStates.
    argsinCell = {'spherical',[],[],[], true, hasZStates};
end
% Only HasElevation, Orientation and if frame is rectangular information is
% required for MSC frame. As state is relative, sensor position, velocity
% must not affect the measurement as their affect is already included in
% the state.
[isRect, ~, ~, orient, ~, hasEl] = ...
    matlabshared.tracking.internal.fusion.measmodelsvalidateoptionalinput(funcName, classToUse, argsinCell{:});

end