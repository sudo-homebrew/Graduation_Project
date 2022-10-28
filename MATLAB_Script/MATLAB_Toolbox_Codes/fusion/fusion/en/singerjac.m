% SINGERJAC  Jacobian of Singer acceleration motion model.
%   The trackingEKF object allows you to specify the
%   StateTransitionJacobianFcn property. SINGERJAC provides the Jacobian
%   of the Singer acceleration state transition function, singer, with
%   respect to the state.
%
%   dfdx = SINGERJAC(state) calculates the Jacobian matrix of the
%   Singer acceleration motion model with respect to the state vector. It
%   is assumed that dt = 1 second.
%
%   dfdx = SINGERJAC(state, dt) allows you to specify a time interval, dt.
%
%   dfdx = SINGERJAC(state, dt, tau) additionally, allows you the specify
%   the target maneuver time constant, tau. If not specified, it is assumed
%   to be 20 seconds.
%
%   The dfdx Jacobian is an n-by-n matrix, where n is the number of states
%   in the state vector. If rm = exp(-dt/tau), then
%   In a 1-D motion, the Jacobian is:
%       A = [1 dt tau^2(-1*dt/tau+rm); 0 1 tau*(1-rm); 0 0 rm].
%   In a 2-D motion, the Jacobian is blkdiag(A, A)
%   In a 3-D motion, the Jacobian is blkdiag(A, A, A)
%   Positions are in meters; velocities are in meters/second; accelerations
%   are in meters/second^2.
%
%   Class support
%   -------------
%   state must be a finite double or single precision vector of the
%   dimensions specified above.
%   dt must be a real finite double or single precision scalar.
%   tau must be a real finite double or single precision vector or scalar.
%   If specified as vector, it must have the same number of elements as the
%   number of dimensions. If specified as a scalar, it will be expanded to
%   the number of dimensions.
%
%   Example
%   -------
%   % Define a state for 2-D Singer acceleration motion
%   state = [1;1;1;2;1;0]; 
%
%   % Calculate the Jacobian matrix assuming dt = 1 second
%   dfdx = SINGERJAC(state)
%
%   % Calculate the Jacobian matrix given dt = 0.1 second
%   dfdx = SINGERJAC(state, 0.1)
%
%   See also trackingEKF, initsingerekf, singer

 
%   Copyright 2020 The MathWorks, Inc.

