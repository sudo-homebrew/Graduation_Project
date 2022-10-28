function state = constvelmsc(state,vNoise,varargin)
% CONSTVELMSC   Constant velocity (CV) motion model in MSC frame
%   CONSTVELMSC provides a constant velocity transition function in
%   modified spherical coordinates (MSC) using a non-additive noise
%   structure. The MSC frame assumes a single observer and the state is
%   defined relative to it.
%
%   state = CONSTVELMSC(state,vNoise) calculates the state at the next
%   time-step based on current state and target acceleration noise, vNoise,
%   in scenario frame. It assumes dt = 1 second and zero observer
%   acceleration in all dimensions.
%
%   state = CONSTVELMSC(state,vNoise,dt) allows specifying the time
%   interval, dt. It assumes zero observer acceleration in all dimensions.
%
%   state = CONSTVELMSC(state,vNoise,dt,u) allows specifying the observer
%   input, u, during the time interval, dt. The observer input can have the
%   following impact on state-prediction based on its dimensions:
%
%   Case 1: Number of elements in u = number of elements in state
%   u is assumed to be the maneuver performed by the observer during the
%   time interval, dt. A maneuver is defined as motion of the observer
%   higher than first order (or constant velocity)
%
%   Case 2: Number of elements in u = 1/2 x number of elements in state
%   u is assumed to be constant acceleration of the observer in scenario
%   frame during the time interval, dt.
%
%   state can be defined as a vector or matrix for 2 or 3 dimensions.
%   The 2-dimensional version of modified spherical coordinates (MSC)
%   is also referred to as the modified polar coordinates (MPC).
%       In a 2-D case: state = [az azRate 1/r vr/r];
%       In a 3-D case: state = [az omega el elRate 1/r vr/r];
%   If specified as a matrix, states must be concatenated along columns,
%   where each column represents a state following the convention specified
%   above.
%
%   The variables used in the convention have the following definitions:
%
%   az = azimuth angle (rad)
%   el = elevation angle (rad)
%   azRate = azimuth rate (rad/s)
%   elRate = elevation rate (rad/s)
%   omega = azRate * cos(el) (rad/s)
%   1/r = 1/range (1/m)
%   vr/r = range-rate/range or inverse time-to-go. (1/s)
%   
%   Class support
%   -------------
%   state must be finite real vector or matrix with the dimensions 
%   specified above.
%   vNoise must be a finite real vector or matrix with dimensions
%   corresponding to state.
%   dT must be a real finite numeric scalar.
%   u must be a finite real vector of dimensions specified above.
%
%   % Example 1: Predict a constant velocity MSC state with various
%   % different inputs
%   % ----------------------------------------------------------------------
%   % Define a state vector for 3-D MSC state
%   mscState = [0.1;0.01;0.1;0.01;0.001;1];
%   dt = 0.1;
%   % Predict the state with 0 observer acceleration
%   mscState = CONSTVELMSC(mscState,zeros(3,1),dt)
%
%   % Predict the state with [5;3;1] observer acceleration in x,y,z
%   % direction
%   mscState = CONSTVELMSC(mscState,zeros(3,1),dt,[5;3;1])
%
%   % Predict the state with observer maneuver and unit standard deviation
%   % random noise in target acceleration
%   % Let observer acceleration in time interval be [sin(t),cos(t)];
%   velManeuver = [1 - cos(dt);sin(dt);0];
%   posManeuver = [-sin(dt);cos(dt) - 1;0];
%   u = zeros(6,1);
%   u(1:2:end) = posManeuver;
%   u(2:2:end) = velManeuver;
%   mscState = CONSTVELMSC(mscState,randn(3,1),dt,u);
%
%
%   % Example 2: Predict and measure the state of constant velocity 
%   % target in modified spherical coordinates
%   % -------------------------------------------------------------
%   % Define a state vector for a motion model in 2-D
%   mscState = [0.5;0.02;1/1000;-10/1000];
%   dt = 2; % time step in seconds
%   
%   % As MSC state is relative, let the observer state be defined by a 
%   % constant acceleration model in 2-D.
%   observerState = [100;10;0.5;20;-5;0.1];
%   
%   % Use CONSTVELMSC to create a trajectory with constant velocity target
%   % and measure the angles using the measurement function, cvmeasmsc
%   % Pre-allocate memory
%   observerPositions = zeros(2,10);
%   targetPositions = zeros(2,10);
%   azimuthMeasurement = zeros(1,10);
%   bearingHistory = zeros(2,30);
%   rPlot = 2000; % range for plotting bearing measurements.
%
%   % Use a loop to predict the state multiple times.
%   for i = 1:10
%       obsAcceleration = observerState(3:3:end);
%       % Use zeros(2,1) as process noise to get true predictions
%       mscState = CONSTVELMSC(mscState,zeros(2,1),dt,obsAcceleration);
% 
%       % Update observer state using constant acceleration model
%       observerState = constacc(observerState,dt);
%       observerPositions(:,i) = observerState(1:3:end);
%
%       % Update bearing history with current measurement.
%       az = cvmeasmsc(mscState);
%       bearingHistory(:,3*i-2) = observerState(1:3:end);
%       bearingHistory(:,3*i-1) = observerState(1:3:end) + [rPlot*cosd(az);rPlot*sind(az)];
%       bearingHistory(:,3*i) = [NaN;NaN];
%
%       % Use the 'rectangular' frame to get relative positions of the
%       % target using cvmeasmsc function.
%       relativePosition = cvmeasmsc(mscState,'rectangular');
%       relativePosition2D = relativePosition(1:2);
%       targetPositions(:,i) = relativePosition2D + observerPositions(:,i);
%   end
%   plot(observerPositions(1,:),observerPositions(2,:)); hold on;
%   plot(targetPositions(1,:),targetPositions(2,:));
%   plot(bearingHistory(1,:),bearingHistory(2,:),'-.');
%   title('Constant velocity model in modified spherical coordinates');xlabel('X[m]'); ylabel('Y[m]')
%   legend('Observer Positions', 'Target Positions', 'Bearings Measurements'); hold off;
%  
%   See also: trackingEKF, trackingMSCEKF, constvelmscjac

%   References
%   [1] Stallard, David V. "Angle-only tracking filter in modified
%       spherical coordinates." Journal of guidance, control, and dynamics
%       14.3 (1991): 694-696.

%   Copyright 2018 The MathWorks, Inc.


%#codegen

narginchk(2,4);

[stateCol,vNoise,dT,obsManeuver,hasZStates,isRowVector] = ...
    fusion.internal.parseAndValidateMSCInputs(state,vNoise,mfilename,varargin{:});

% convert msc-state to cartesian coordinates
cartState = fusion.internal.mscToCartesian(stateCol);

% Predict cartesian state
G1d = [dT^2/2;dT];
B = blkdiag(G1d,G1d,G1d);
cartStateT = bsxfun(@minus,constvel(cartState,dT) + B*vNoise,obsManeuver);

% Convert state back to MSC coordinates
stateCol = fusion.internal.cartesianToMSC(cartStateT);

% Return the state with correct dimension and orientation
if ~hasZStates
    stateInOrder = stateCol([1:2,5:6],:);
else
    stateInOrder = stateCol;
end

if isRowVector
    state = stateInOrder';
else
    state = stateInOrder;
end

end