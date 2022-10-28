function dhdx = ctrectmeasjac(state, detections)
% CTRECTMEASJAC Jacobian of constant turn-rate rectangular target measurement model
%   gmphd requires the definition of a MeasurementJacobianFcn for extended
%   targets as a function of current detections. This function provides the
%   measurement Jacobian for the constant turn-rate rectangular target
%   measurement model, ctrectmeas.
%
%   dhdx = CTRECTMEASJAC(states, detections)
%   provides the Jacobian for the current set of measured
%   observations, detections.
%
%   state is a 7-by-1 vector, defined according to the following
%   convention.
%
%       [x;y;s;theta;omega;length;width];
%
%   detections is a M-element cell array of objectDetection objects, where
%   each element corresponds to one observation. The MeasurementParameters
%   of the detection is assumed same for all detections and define the
%   transformation from the state-space to measurement-space.
%
%   dhdx is a P-by-7-by-M, where P is the dimension of
%   each measurement. For example, P = 3 for a position measurement.
%   dhdx(:,:,j) corresponds to the Jacobian from jth detection.
%
%   Notes:
%   1. The function uses numerical perturbations on state to compute the
%   Jacobian.
%   2. The relative step chosen for perturbations is sqrt(eps(dataType))
%   for each state, where dataType is the data type of the state.
%
%   Example: Generate Jacobian for expected detections calculation
%   --------------------------------------------------------------
%   % Load detections generated from a rectangular target
%   load ('rectangularTargetDetections.mat','detections','truthState');
%   
%   % Calculate Jacobian
%   tgtState = [3;48;0;60;0;5;1.9];
%   H = ctrectmeasjac(tgtState, detections);
%   
%   See also: objectDetection, gmphd, ctrect, ctrectjac, ctrectmeas,
%   initctrectgmphd

%   References:
%   [1] Granström, Karl, Christian Lundquist, and Umut Orguner. "Tracking
%   rectangular and elliptical extended targets using laser measurements."
%   14th International Conference on Information Fusion. IEEE, 2011.

%   Copyright 2019 The MathWorks, Inc.

%#codegen

narginchk(2,2);

% Dimension of state
stateDim = 7;

% Validate state input
validateattributes(state, {'single','double'},...
    {'real', 'finite', 'vector', 'nonsparse'}, 'ctrectmeasjac', 'state', 1);

coder.internal.assert(numel(state) == stateDim, 'fusion:ctrect:incorrectStateVecWithInfo');

stateCol = state(:);

% Validate detection input
validateattributes(detections, {'cell'}, ...
    {'vector','nonempty'},'ctrectmeas','detections',2);

% Perturbation eps
classToUse = class(state);
epsilon = sqrt(eps(classToUse));

% Call using vectorized capabilities of ctrectmeas
stateDeltas = bsxfun(@plus, stateCol ,epsilon*eye(stateDim,classToUse));
zExpDeltas = ctrectmeas([state stateDeltas],detections);

% Compute jacobian
dhdx = bsxfun(@minus,zExpDeltas(:,2:8,:), zExpDeltas(:,1,:))/epsilon;
