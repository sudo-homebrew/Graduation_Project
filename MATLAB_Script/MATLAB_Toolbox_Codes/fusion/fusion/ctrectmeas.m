function measurements = ctrectmeas(states, detections)
% CTRECTMEAS A constant turn-rate rectangular target measurement model
%   gmphd requires the definition of a MeasurementFcn for extended targets
%   as a function of current detections. This function provides the
%   expected measurements for a constant turn-rate rectangular target
%   model.
%
%   measurements = CTRECTMEAS(states, detections)
%   provides the expected measurements for the current set of measured
%   observations, detections.
%
%   states is a 7-by-N matrix, where N represents the number of states.
%   Each column of the matrix defines a state according to the following
%   convention:
%       [x;y;speed;theta;omega;length;width];
%
%   detections is a M-element cell array of objectDetection objects, where
%   each element corresponds to one observation. The MeasurementParameters
%   of the detection is assumed same for all detections and define the
%   transformation from the state-space to measurement-space.
%
%   measurements is a P-by-N-by-M matrix, where P is the dimension of
%   each measurement. For example, P = 3 for a position measurement.
%   measurements(:,i,j) corresponds to the measurement from ith state,
%   against jth detection.
%   
%   Example: Generate expected detections from position measurements
%   ----------------------------------------------------------------
%   % Load detections generated from a rectangular target
%   load ('rectangularTargetDetections.mat','detections','truthState');
%   
%   % Compute expected detections
%   tgtState = [3;48;0;60;0;5;1.9];
%   zExp = ctrectmeas(tgtState, detections);
%   
%   % Visualize detections, expected detections
%   % Initialize displays
%   theaterP = theaterPlot;
%   stateP = trackPlotter(theaterP,'DisplayName','State',...
%                                   'MarkerFaceColor','g')
%   truthP = trackPlotter(theaterP, 'DisplayName', 'Truth',...
%                                    'MarkerFaceColor', 'b');
%   detP = detectionPlotter(theaterP,'DisplayName','Detections',...
%                                    'MarkerFaceColor','r');
%   expDetP = detectionPlotter(theaterP,'DisplayName',...
%             'Expected Detections','MarkerFaceColor','y');
%   l = legend(theaterP.Parent);
%   l.AutoUpdate = 'on';
%   hold on;
%   assignP = plot(theaterP.Parent, nan, nan, '-.', 'DisplayName', 'Association');
% 
%   % Plot data
%   inDets = [detections{:}];
%   inMeas = horzcat(inDets.Measurement);
%   detP.plotDetection(inMeas');
%   
%   zExpPlot = reshape(zExp,3,[]);
%   expDetP.plotDetection(zExpPlot');
%   
%   % Plot association lines
%   zLines = nan(2,numel(detections)*3);
%   zLines(1,1:3:end) = zExpPlot(1,:);
%   zLines(2,1:3:end) = zExpPlot(2,:);
%   zLines(1,2:3:end) = inMeas(1,:);
%   zLines(2,2:3:end) = inMeas(2,:);
%   assignP.XData = zLines(1,:);
%   assignP.YData = zLines(2,:);
%
%   % Plot truth and state
%   truthPos = [truthState(1:2);0];
%   truthDims = struct('Length',truthState(6),...
%                      'Width',truthState(7),...
%                      'Height', 0,...
%                      'OriginOffset', [0 0 0]);
%   truthOrient = quaternion([truthState(4) 0 0],'eulerd', 'ZYX','frame');
%   truthP.plotTrack(truthPos', truthDims, truthOrient);
%
%   statePos = [tgtState(1:2);0];
%   stateDims = struct('Length',tgtState(6),...
%                      'Width',tgtState(7),...
%                      'Height', 0,...
%                      'OriginOffset', [0 0 0]);
%   stateOrient = quaternion([tgtState(4) 0 0],'eulerd', 'ZYX','frame');
%   stateP.plotTrack(statePos', stateDims, stateOrient);
%   
%   See also: objectDetection, gmphd, ctrect, ctrectjac, ctrectmeasjac,
%   initctrectgmphd

%   References:
%   [1] Granström, Karl, Christian Lundquist, and Umut Orguner. "Tracking
%   rectangular and elliptical extended targets using laser measurements."
%   14th International Conference on Information Fusion. IEEE, 2011.
%
%   Copyright 2019 The MathWorks, Inc.

%#codegen

narginchk(2,2);
funcName = mfilename;

% Validate state input
validateattributes(states, {'single','double'},...
    {'real', 'finite', '2d', 'nonsparse'}, 'ctrectmeas', 'state', 1);

classToUse = class(states);

% Check for row or column Orientation
[rowSize,colSize] = size(states);
isRowOrient = colSize == 7;
isColOrient = rowSize == 7;

% Only column orientation is supported for vectorized state
cond = (~isRowOrient || ~isvector(states)) && ~isColOrient;
coder.internal.errorIf(cond, 'fusion:ctrect:incorrectStateSizeWithInfo');

if ~isColOrient
    stateCol = states';
else
    stateCol = states;
end
N = size(stateCol,2);

% Validate detection input
validateattributes(detections, {'cell'}, ...
    {'vector','nonempty'},'ctrectmeas','detections',2);
M = numel(detections);
P = numel(detections{1}.Measurement);

% Allocate memory
measurements = zeros(P,N,M,classToUse);

if N == 0
    return;
end
% Concatenate all detection measurements
z = concatenateMeasurements(detections,classToUse);

% Measurement parameters
[isRect,sensorPos,sensorVel,sensorOrient,hasAz,hasEl,hasVel,hasRange] = ...
    matlabshared.tracking.internal.fusion.parseDetectionForMeasurementParameters(detections{1},funcName,classToUse);

% Assert the following
% Positions in sensor frame should be obtainable from detections
observable = isRect || (hasAz && hasRange);
coder.internal.assert(observable,'fusion:ctrect:noAzRange');

% Get x and y positions and az of detections in sensor frame.
if isRect
    xDets = z(1,:);
    yDets = z(2,:);
else
    azDets = z(1,:);    
    rngIndex = 2 + hasEl;
    rDets = z(rngIndex,:);
    if ~hasEl
        [xDets, yDets] = pol2cart(deg2rad(azDets),rDets);
    else
        elDets = z(2,:);
        [xDets, yDets] = sph2cart(deg2rad(azDets),deg2rad(elDets),rDets);
    end
end

% Normal angles of all states
sensorYaw = atan2d(sensorOrient(2,1),sensorOrient(1,1));
etas = bsxfun(@plus,deg2rad(bsxfun(@minus,stateCol(4,:),sensorYaw)),[0;pi/2;pi;3*pi/2]);
etas = fusion.internal.UnitConversions.interval(etas,[0 2*pi]);

% lengths of these sides. Order [w;l;w;l]
sideLengths = [stateCol(7,:);stateCol(6,:);stateCol(7,:);stateCol(6,:)]; 

% x and y of detections in this cell
zCell = [xDets;yDets];

% Compute covariance of the detections in this cell.
zMean = mean(zCell,2);
e = bsxfun(@minus, zCell, zMean);
nCell = size(zCell,2);
R = e*e'/nCell;
eigValues = sort(max(eps(classToUse),eig(R)));
is2Sided = eigValues(2)/eigValues(1) < 50 & nCell > 1;

if is2Sided
    zExpCell = fusion.internal.RectangularMeasurementUtilities.computeTwoSidedRectangularMeasurement(zCell, stateCol, etas, sideLengths, isRect, sensorPos, sensorVel, sensorOrient, hasAz, hasEl, hasVel, hasRange);
elseif nCell > 1
    isSorted = false;
    zExpCell = fusion.internal.RectangularMeasurementUtilities.computeOneSidedRectangularMeasurement(zCell, stateCol, etas, sideLengths, isRect, sensorPos, sensorVel, sensorOrient, hasAz, hasEl, hasVel, hasRange, isSorted);
else
    zExpCell = fusion.internal.RectangularMeasurementUtilities.computePointRectangularMeasurement(z, stateCol, isRect, sensorPos, sensorVel, sensorOrient, hasAz, hasEl, hasVel, hasRange);
end

% Edge cases. If spherical frame and azimuth angles are on boundary, return
% the value closer to the original value. For example if input is 179
% degrees and output is -179 degrees, return 181 for output
if ~isRect
    azAnglesIn = z(1,:); % This is 1xM array
    azAnglesInReshaped = reshape(azAnglesIn,1,1,M); % This is a 1x1xM array
    azAnglesOut = zExpCell(1,:,:); % This is a 1xNxM array
    azAnglesOutFlipped = azAnglesOut + 360; % This is a 1xNxM array
    dFlipped = abs(bsxfun(@minus,azAnglesOutFlipped,azAnglesInReshaped));
    dOriginal = abs(bsxfun(@minus,azAnglesOut,azAnglesInReshaped));
    needsFlipping = dFlipped < dOriginal;
    zExpCell(1,needsFlipping) = azAnglesOutFlipped(1,needsFlipping);
end
% Output
measurements = zExpCell;

end

function z = concatenateMeasurements(detections,classToUse)
if coder.target('MATLAB')
    allDets = [detections{:}];
    z = horzcat(allDets.Measurement);
else
    numDets = numel(detections);
    p = numel(detections{1}.Measurement);
    z = zeros(p,numDets,classToUse);
    for i = 1:numDets
        z(:,i) = detections{i}.Measurement;
    end
end
end