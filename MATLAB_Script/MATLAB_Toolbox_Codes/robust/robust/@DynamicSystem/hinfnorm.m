function [gpeak,fpeak] = hinfnorm(sys,tol)
%HINFNORM  Compute H-infinity norm of a linear dynamic system.
%
%   NINF = HINFNORM(SYS) returns the H-infinity norm NINF of the dynamic  
%   system SYS, defined as the peak gain of the frequency response when 
%   SYS is stable, and Inf when SYS is unstable. For MIMO systems, the
%   gain at a particular frequency is measured by the largest singular 
%   value of the frequency response at this frequency.
%
%   NINF = HINFNORM(SYS,TOL) specifies the relative accuracy TOL for the
%   computed value NINF. By default NINF is computed with 1% accuracy
%   (TOL=1e-2).
%
%   [NINF,FPEAK] = HINFNORM(SYS,...) also returns the frequency FPEAK
%   (in rad/TimeUnit) at which the gain achieves its peak value NINF.
%   HINFNORM returns FPEAK=NaN when SYS is unstable.
% 
%   If SYS is an array of dynamic systems, HINFNORM returns an array 
%   of the same size where NINF(k) = HINFNORM(SYS(:,:,k)).
%
%   Note: HINFNORM is the same as getPeakGain for stable systems.
%   
%   See also getPeakGain, freqresp, sigma, DynamicSystem.

%   Copyright 1986-2011 The MathWorks, Inc.
narginchk(1,2)
ni = nargin;

% Validate optional arguments
if ni<2 || isempty(tol)
   tol = 1e-2;
else
   if ~(isnumeric(tol) && isscalar(tol) && isreal(tol) && tol>0)
      error(message('Control:analysis:getPeakGain1'))
   end
   tol = max(100*eps,double(tol));
end

% Compute peak gain
try
   [gpeak,fpeak] = norminf_(sys,tol,[],true);
catch E
   ltipack.throw(E,'command','hinfnorm',class(sys))
end
