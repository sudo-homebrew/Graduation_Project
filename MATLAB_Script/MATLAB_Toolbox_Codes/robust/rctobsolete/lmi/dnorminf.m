% [gain,peakf]=dnorminf(sys,tol)
%
% Computes the peak gain of the discrete-time transfer
% function
%                                   -1
%             G (z) = D + C (zE - A)  B
%
% The norm is finite if and only if (A,E) has no eigenvalue
% on the unit circle
%
% Input:
%   SYS        system matrix description of G(z)  (see LTISYS)
%   TOL        relative accuracy required (default = 0.01)
%
% Output:
%   GAIN       peak gain  (RMS gain when G is stable)
%   PEAKF      "frequency" at which this norm is attained, i.e.:
%
%                               j*PEAKF
%                       || G ( e        ) ||  =  GAIN
%
%
% See also  NORMINF.

% Author: P. Gahinet and A.J. Laub 10/93
% Copyright 1995-2011 The MathWorks, Inc.

function [gain,peakf]=dnorminf(sys,tol)

narginchk(1,2);
ni = nargin;

if ~islsys(sys)
    error(message('Robust:obsolete:dnorminf1'));
end
if ni<2
    tol=1e-2;
end

[a,b,c,d,e]=ltiss(sys);
Ts=1; %
sys=dss(a,b,c,d,e,Ts);
[gain,peakf] = getPeakGain(sys,tol); % compute gain

% LocalWords:  peakf RMS Gahinet Laub
