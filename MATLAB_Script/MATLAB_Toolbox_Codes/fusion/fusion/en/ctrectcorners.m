% CTRECTCORNERS Corners of constant turn-rate rectangular target model
%   zCorners = CTRECTCORNERS(state) provides the position of the corners in
%   rectangular frame. The state vector is defined as
%   [x;y;speed;theta;omega;L;W] If the input, state, is specified as a
%   matrix, states must be concatenated along columns, where each column
%   represents a state following the convention specified above. The output
%   will be a matrix with the same number of columns as the input and 4
%   pages. Each column represents the measurement from the corresponding
%   state and each page refers to the measurement from each corner.
%
%   zCorners = CTRECTCORNERS(state, sensorParameters) specifies the
%   sensor parameters as a struct or an array of struct. The struct
%   must have the following fields (or a subset of them)
%       Frame           - either 'rectangular' or 'spherical' or an enum
%                         with the same values.
%       OriginPosition  - a 3-by-1 real vector.
%       OriginVelocity  - a 3-by-1 real vector.
%       Orientation     - a 3-by-3 orthonormal orientation matrix.
%       HasAzimuth      - a logical scalar, true if azimuth is measured.
%       HasElevation    - a logical scalar, true if elevation is measured.
%       HasVelocity     - a logical scalar, true if velocity is measured.
%       HasRange        - a logical scalar, true if range is measured.
%		IsParentToChild - a logical scalar, true if the orientation is
%		                  given as a parent to child frame rotation.
%
%
%   When Frame is 'rectangular', HasVelocity determines if the measurement
%   is [x;y;z;vx;vy;vz] or [x;y;z].
%
%   When Frame is 'spherical', the output order is [az,el,r,rr]
%   HasAzimuth   - determines if output contains azimuth measurement
%   HasElevation - determines if output contains elevation measurement
%   HasRange     - determines if output contains range measurement
%   If HasRange is true:
%   HasVelocity  - determines if output contains range-rate.
%   If HasRange is false:
%   HasVelocity  - no effect on output.
%
%   Example 1: Get position of corners in the sensor coordinate system.
%   % Define the sensor coordinate system
%   sensorPosition = [-5;10;0];
%   sensorOrientation = rotmat(quaternion([30 0 0],'eulerd','ZYX','frame'),'frame');
%   sensorParams = struct('Frame','Rectangular', ...
%                         'OriginPosition',sensorPosition,...
%                         'Orientation', sensorOrientation);
%   % Define the constant turn-rate state for the rectangle
%   state = [10;5;1.6;30;0.5;4.7;1.8];
%
%   % Compute corner positions in sensor frame
%   corners = CTRECTCORNERS(state, sensorParams);
%
%   % Visualize results
%   % Create a theater plot
%   tp = theaterPlot;
%
%   % Plot the state using a track plotter
%   statePlotter = trackPlotter(tp, 'DisplayName', 'Target State');
%
%   % Plot the corners using a detection plotter
%   cornerPlotter = detectionPlotter(tp, 'DisplayName', 'Corners');
%
%   % Compute inputs for plotting
%   targetPos = [state(1) state(2) 0];
%   targetOrientation = rotmat(quaternion([state(4) 0 0],'eulerd','ZYX','frame'),'frame');
%   targetDims = struct('Length',state(6),...
%                       'Width',state(7),...
%                       'Height',5,...
%                       'OriginOffset',[0 0 0]);
%
%   cornerPosGlobal = sensorOrientation*corners(:,:) + sensorPosition;
%   statePlotter.plotTrack(targetPos, targetDims, targetOrientation);
%   cornerPlotter.plotDetection(cornerPosGlobal');
%
%   See also: trackingSensorConfiguration, gmphd, ctrect, ctrectmeas,
%   ctrectjac, ctrectmeasjac

 
% Copyright 2019 The MathWorks, Inc.

