function dRange = getDynamicRange(p)
% Estimate dynamic range from continuous-time poles.

%   Copyright 2016 The MathWorks, Inc.
wp = abs(p);
wp = wp(wp>1e-8);
if isempty(wp)
   dRange = [1 1];
else
   wp = sort(wp);
   dRange = [wp(1) wp(end)];
end
