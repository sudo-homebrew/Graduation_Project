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

 %   Copyright 2019 The MathWorks, Inc.

