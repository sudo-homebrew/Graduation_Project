% CTRECTJAC Jacobian of constant turn-rate rectangular target motion model
%   gmphd allows you to specify the StateTransitionJacobianFcn property.
%   CTRECTJAC provides the Jacobian of constant turn-rate rectangular model
%   state transition function, ctrect, with respect to the state.
%
%   dfdx = CTRECTJAC(state) calculates the Jacobian matrix of the constant
%   turn-rate rectangular motion model with respect to the state vector. It
%   assumes that dt = 1 second.
%
%   dfdx = CTRECTJAC(state, dt) allows you to specify a time interval, dt.
%
%   [dfdx, dfdw] = CTRECTJAC(state, w, dt) allows you to specify a noise
%   term, w and a time interval, dt and returns the Jacobian of the
%   transition with respect to the noise, dfdw. w is a 2-element vector
%   defining the linear and yaw acceleration.
%
%   The Jacobian with respect to state, dfdx, is a 7-by-7 matrix.
%   The Jacobian with respect to the noise, dfdw, is a 7-by-2 matrix.
%
%   Example
%   -------
%   % Define a state vector for the model
%   state = [1;2;2;30;1;4.7;1.8];
%
%   % Compute the Jacobian assuming dt = 1 second
%   dfdx = CTRECTJAC(state);
%
%   % Compute the Jacobian given dt = 0.1 second
%   dfdx = CTRECTJAC(state, 0.1);
%
%   % Compute the Jacobian with respect to noise terms
%   [dfdx, dfdw] = CTRECTJAC(state, zeros(2,1), 0.1);
%
%   See also: ctrect, ctrectmeas, ctrectmeasjac, gmphd, initctrectgmphd

 
% Copyright 2019 The MathWorks, Inc.

