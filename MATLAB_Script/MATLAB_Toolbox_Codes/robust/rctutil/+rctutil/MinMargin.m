function [GM,PM] = MinMargin(mg,Bnd)
% Computes disk margins

%   Author(s): MUSYN
%   Copyright 2004-2011 The MathWorks, Inc.
idx = find(mg<=Bnd);
PM = zeros(size(mg));
GM = zeros(size(mg));
if isempty(idx)
   GM = (1+1./mg)./(1-1./mg);
   PM = (180/pi)*2*atan(1./mg);
else
   GM(idx) = inf;
   PM(idx) = 90;
   gidx = 1:numel(mg);
   gidx(idx) = [];
   if ~isempty(gidx)
      GM(gidx) = (1+1./mg(gidx))./(1-1./mg(gidx));
      PM(gidx) = (180/pi)*2*atan(1./mg(gidx));
   end
end
