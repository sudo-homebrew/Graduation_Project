function [ActLims,NormLims] = getLimits(blk)
%GETLIMITS   Validity range for uncertain real parameters.
%
%   For analysis purposes, uncertain real parameter (UREAL) are modeled as 
%
%       u = u_nom + a delta / (1 - b delta) ,   a > 0
%
%   where "u_nom" is the nominal value and "delta" is the normalized value.
%   When the uncertainty range is centered at the nominal value, b=0 and
%   there are no restrictions on the values u or delta can take. When 
%   the uncertainty range [u_nom-DL,u_nom+DR] is skewed, the mapping from 
%   actual to normalized values (u to delta) has a singularity at delta=1/b. 
%   To ensure continuity, the values delta,u are then restricted to 
%      For b>0 (DL<DR):  delta<1/b  and  u>u_nom-a/b
%      For b<0 (DL>DR):  delta>1/b  and  u<u_nom-a/b.
% 
%   [ACTLIMS,NORMLIMS] = getLimits(UBLK) computes the intervals ACTLIMS
%   and NORMLIMS of actual and normalized values that the uncertain real 
%   parameter UBLK can take. For meaningful analysis results, the actual
%   and normalized values of UBLK must be kept in these intervals at all
%   time and values outside these intervals are essentially meaningless.
%   In other words, ACTLIMS and NORMLIMS are the ranges of validity of the
%   uncertainty model for real parameters.
%
%   Examples
%      % Create a UREAL with range centered at the nominal value
%      u = ureal('a',1,'range',[-1 3])
%      % Here b=0 so U and DELTA are unconstrained
%      [ActLims,NormLims] = getLimits(u)
%
%      % Now skew the range to the right of the nominal value
%      u.Range = [0 3]
%      % The values U and DELTA can take are now restricted to 
%      [ActLims,NormLims] = getLimits(u)
%
%   See also UREAL, ACTUAL2NORMALIZED, NORMALIZED2ACTUAL.

%   Copyright 2017 The MathWorks, Inc.
[~,~,T] = normalizeBlock(blk);
u_nom = T(1,1);
a = T(1,2)*T(2,1);
b = T(2,2);
if b==0
   ActLims = [-Inf,Inf];
   NormLims = [-Inf,Inf];
elseif b>0
   ActLims = [u_nom-a/b,Inf];
   NormLims = [-Inf,1/b];
else
   ActLims = [-Inf,u_nom-a/b];
   NormLims = [1/b,Inf];
end