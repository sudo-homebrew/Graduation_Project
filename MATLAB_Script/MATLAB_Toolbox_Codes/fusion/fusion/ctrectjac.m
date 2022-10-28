function [dfdx, dfdw] = ctrectjac(state, varargin)
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

%#codegen

% Validate state input
validateattributes(state, {'single','double'},...
    {'real', 'finite', 'vector', 'nonsparse'}, 'ctrectjac', 'state', 1);

% If dt is not given, assume dt = 1
if nargin==1 || (nargin==2 && isnumeric(varargin{1}) && ~isscalar(varargin{1}))
    dT = ones(1, 1, 'like', state);
else  % dt is always at the end of varargin
    validateattributes(varargin{end}, {'numeric'}, ...
        {'real', 'finite', 'scalar', 'nonsparse'}, 'ctrectjac', 'dt', nargin);
    dT = varargin{end};
end

coder.internal.assert(numel(state) == 7, 'fusion:ctrect:incorrectStateVecWithInfo');

classToUse = class(state);
dfdx = eye(7,classToUse);

s = state(3);
omega = deg2rad(state(5));
theta = deg2rad(state(4));

sinwt = sin(omega*dT);
coswt = cos(omega*dT);
sinTheta = sin(theta);
cosTheta = cos(theta);

if abs(omega) > sqrt(eps)
    % Jacobians of x transition
    dfdx(1,3) = 1/omega*(cosTheta*sinwt - sinTheta*(1 - coswt));
    dfdx(1,4) = s/omega*(-sinTheta*sinwt - cosTheta*(1 - coswt));
    dfdx(1,5) = -s/omega^2*(cosTheta.*sinwt - sinTheta.*(1 - coswt)) + s*dT/omega*(cosTheta*coswt - sinTheta*sinwt);
    
    % Jacobians of y transition
    dfdx(2,3) = 1/omega*(sinTheta*sinwt + cosTheta*(1 - coswt));
    dfdx(2,4) = s/omega*(cosTheta*sinwt - sinTheta*(1 - coswt));
    dfdx(2,5) = -s/omega^2*(sinTheta*sinwt + cosTheta*(1 - coswt)) + s*dT/omega*(sinTheta*coswt + cosTheta*sinwt);
else % omega = 0
    % Use limits with respect to omega
    dfdx(1,3) = dT*cosTheta;
    dfdx(1,4) = -s*sinTheta*dT;
    dfdx(1,5) = -s*sinTheta*dT^2/2;
    
    dfdx(2,3) = dT*sinTheta;
    dfdx(2,4) = s*dT*cosTheta;
    dfdx(2,5) = s*cosTheta*dT^2/2;
end

% Jacobians with respect to degree states.
deg2radOne = deg2rad(ones(1,classToUse));
dfdx(1:3,4) = deg2radOne*dfdx(1:3,4);
dfdx(1:3,5) = deg2radOne*dfdx(1:3,5);

% Jacobian of theta transition
dfdx(4,5) = dT;

% Jacobian with respect to noise;
dfdw = zeros(7,2,classToUse);
dfdw(1,1) = cosTheta*dT^2/2;
dfdw(2,1) = sinTheta*dT^2/2;
dfdw(3,1) = dT;
dfdw(4,2) = dT^2/2;
dfdw(5,2) = dT;

end