function out = rcond(m)
% RCOND  LAPACK reciprocal condition estimator.
%
%  RCOND(X) is an estimate for the reciprocal of the
%  condition of X in the 1-norm obtained by the LAPACK
%  condition estimator. If X is well conditioned, RCOND(X)
%  is near 1.0. If X is badly conditioned, RCOND(X) is
%  near EPS.
% 
%  See also cond, norm, condest, normest.

%   Copyright 2003-2011 The MathWorks, Inc.

m = absorbDelay(m);
FUnit = m.FrequencyUnit;
szm = size(m);
% Data
[mResponseData,Frequency,Ts] = frdata(m);
Nfreq = length(Frequency);
ResponseData = zeros([1 1 Nfreq szm(3:end)]);

for k = 1:prod(szm(3:end))
   for i = 1:Nfreq
      ResponseData(:,:,i,k) = rcond(mResponseData(:,:,i,k));
   end
end
out = frd(ResponseData,Frequency,Ts,'FrequencyUnit',FUnit);
