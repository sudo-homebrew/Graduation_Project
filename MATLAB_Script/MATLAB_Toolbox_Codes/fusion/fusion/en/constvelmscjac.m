% CONSTVELMSCJAC  Jacobian of constant velocity (CV) motion model in MSC frame
%   The trackingEKF object allows you to specify the
%   StateTransitionJacobianFcn property. CONSTVELMSCJAC provides the
%   Jacobian of the constant velocity state transition function in modified
%   spherical coordinates (MSC), constvelmsc, with respect to the state and
%   acceleration noise.
%
%   [jacobianState,jacobianNoise] = CONSTVELMSCJAC(state, vNoise) calculates
%   the jacobian matrix of the motion model with respect to the state
%   vector and the noise. state defines the current state and vNoise
%   defines the target acceleration noise in scenario frame. It
%   assumes dt = 1 second and zero observer acceleration in all dimensions.
%
%   [jacobianState,jacobianNoise] = CONSTVELMSCJAC(state, vNoise, dt) allows
%   specifying the time interval, dt. It assumes zero observer
%   acceleration in all dimensions.
%
%   [jacobianState,jacobianNoise] = CONSTVELMSCJAC(state, vNoise, dt, u)
%   allows specifying the observer input, u, during the time interval, dt.
%   The observer input can have the following impact based on its
%   dimensions:
%
%   Case 1: Number of elements in u = number of elements in state
%   u is assumed that the maneuver performed by the observer during the
%   time interval, dt. A maneuver is defined as motion of the observer
%   higher than first order (or constant velocity)
%
%   Case 2: Number of elements in u = 1/2 x number of elements in state
%   u is assumed to be constant acceleration of the observer, specified in
%   the scenario frame during the time interval, dt.
%
%   The jacobianState is a n-by-n matrix, where n is the number of states
%   in the state vector.
%   The jacobianNoise is a n-by-m matrix, where m is the number of process
%   noise terms i.e. m = 2 for 2-Dimensional state and m = 3 for
%   3-Dimensional state.
%
%   Class support
%   -----------------------------------------------------------------------
%   state must be a finite real vector of dimensions specified above.
%   vNoise must be finite real vector of dimensions specified above.
%   dt must be a real finite numeric scalar
%   u must be real finite vector of dimensions specified above.
%
%   % Example: Compute the jacobian of state transition using various different inputs.
%   % --------------------------------------------------------------------------------
%   % Define a state vector for 2-D MSC.
%   state = [0.5;0.01;0.001;0.01];
%
%   % Calculate the Jacobian matrix assuming dt = 1 second, no observer
%   % maneuver and zero target acceleration noise.
%   [jacobianState, jacobianNoise] = constvelmscjac(state, zeros(2,1))
%
%   % Calculate the Jacobian matrix, given dt = 0.1 second, no observer
%   % maneuver and unit standard deviation target acceleration noise.
%   [jacobianState, jacobianNoise] = constvelmscjac(state,randn(2,1),0.1);
%
%   % Calculate the Jacobian matrix, given dt = 0.1 and observer
%   % acceleration = [0.1 0.3] in scenario frame.
%   [jacobianState,jacobianNoise] = constvelmscjac(state,randn(2,1),0.1,[0.1;0.3]);
%
%   See also: trackingEKF, constvelmsc

 
%   Copyright 2018 The MathWorks, Inc.

