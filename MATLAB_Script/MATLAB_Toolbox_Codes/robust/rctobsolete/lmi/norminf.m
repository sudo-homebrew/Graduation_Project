% [gain,peakf]=norminf(sys,tol)
%
% Computes the peak gain of the frequency response of
%                                   -1
%             G (s) = D + C (sE - A)  B
%
% The norm is finite if and only if (A,E) has no eigenvalue
% on the imaginary axis.
%
% Input:
%   SYS        system matrix description of G(s)  (see LTISYS)
%   TOL        relative accuracy required  (default = 0.01)
%
% Output:
%   GAIN       peak gain  (RMS gain if G is stable)
%   PEAKF      frequency at which this norm is attained.
%              That is, such that
%
%                     || G ( j * PEAKF ) ||  =  GAIN
%
%
% See also  DNORMINF, QUADPERF, MUPERF.

% Authors: P. Gahinet and A.J. Laub  10/93
% Copyright 1995-2011 The MathWorks, Inc.

function [gain,peakf]=norminf(sys,tol)

narginchk(1,2);
ni = nargin;

if ~islsys(sys)
    error(message('Robust:obsolete:norminf1'));
end
if ni<2
    tol = 1e-2;
end

[a,b,c,d,e] = ltiss(sys);
sys = dss(a,b,c,d,e);
[gain,peakf] = getPeakGain(sys,tol);
% LocalWords:  peakf RMS Gahinet Laub
