function Rc = cov2radius(covariance)
%COV2RADIUS Calculates equivalent 95% containment radius 
% Rc = cov2radius(cov) converts a 2D positive definite matrix, cov, to a
% scalar radius, Rc, that represents the 95% circular bound of containment.
%
% This is an internal function and may be removed or modified in a future
% release.

% References: 
% R. Chamlou, "TIS-B: calculation of navigation accuracy
% category for position and velocity parameters," The 23rd Digital Avionics
% Systems Conference, 2004, pp. 1.D.3-11

%   Copyright 2020 The MathWorks, Inc.

%#codegen

% This implements the heuristic formula derived in the referenced paper

eigvals = eig(covariance);
major = sqrt(max(eigvals));
minor = sqrt(min(eigvals));
ratio =  (major + eps) / minor; % prevent 0/0 --> NaN

k = 0.4852 / ratio^3 + 1.9625; 
Rc = k * major;

end

