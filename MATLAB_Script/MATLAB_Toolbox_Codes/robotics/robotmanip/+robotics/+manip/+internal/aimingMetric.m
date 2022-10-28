function [g, dgdv, dgdz] = aimingMetric(v,z)
%This function is for internal use only. It may be removed in the future.

% aimingMetric Compute the squared cosine of the angle between two vectors
%   Copyright 2016 The MathWorks, Inc.

%#codegen

% The square cosine of the angle, a,  between the z-axis of the end
% effector and the line between the end-effector origin and
% target is given by:
%   g := cos(a)^2 = (v'*z)^2/(v'*v)
g = (v'*z+eps)^2/(v'*v + eps);
dgdz = (2*g/(v'*z+eps))*v';
dgdv = (2*g/(v'*z+eps))*z' - (2*g/(v'*v+eps))*v';

