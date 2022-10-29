function [NL,NR,AL,AR] = boundIP(C)
% Computes ranges of validity for the actual <-> normalized
% mapping. [NL,NR] is the largest range of normalized values
% that maps continuously to an interval of actual values, and
% [AL,AR] is the largest range of actual values that maps 
% continuously to an interval of normalized values. UREAL
% values should be kept in these ranges at all times to maintain 
% a meaningful correspondence between actual and normalized values.
%
% C is a cell array of uncertain blocks.

%   Copyright 2010-2017 The MathWorks, Inc.
nblk = numel(C);
NL = -inf(nblk,1); NR = inf(nblk,1);
AL = -inf(nblk,1); AR = inf(nblk,1);
for i=1:numel(C)
   atom = C{i};
   if isa(atom,'ureal')
      [ActLims,NormLims] = getLimits(atom);
      if any(isfinite(NormLims))
         NL(i) = NormLims(1);
         NR(i) = NormLims(2);
         AL(i) = ActLims(1);
         AR(i) = ActLims(2);
      end
   end
end


   







