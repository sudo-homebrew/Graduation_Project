function [muBndsArray,DeltaArray,OutOfRangePert] = ...
   checkUREALRangeFRD(muBndsArray,DeltaArray,NL,NR,blk)
% Enforces limits on normalized range for UREALs.

%   Copyright 2010-2016 The MathWorks, Inc.

% muBndsArray, 1-by-2-by-N
% DeltaArray,  r-by-c-by-N
Limit = min([abs(NL);NR]);
OutOfRangePert = (Limit*muBndsArray(1,2,:)<1);

if isfinite(Limit)
   % Normalized range is constrained by some UREALs. Limit the margin to
   % the range [-Limit,Limit] where normalized2actual is well defined,
   % and handle case when destabilizing delta is outside [NL,NR].
   minMU = 1/Limit;
   rtol = 1e3*eps;
   tolAbs = rtol * Limit;
   
   muBndsArray = max(minMU,muBndsArray);
   
   for k=1:size(muBndsArray,3)
      rp = 1;
      cp = 1;
      for i=1:size(blk,1)
         if blk(i,1)<0
            delta = DeltaArray(rp,cp,k);
            if delta<NL(i)-tolAbs
               % Nearest valid value is normalized2actual(inf)
               delta = Limit/rtol;
            elseif delta>NR(i)+tolAbs
               % Nearest valid value is normalized2actual(-inf)
               delta = -Limit/rtol;
            else
               delta = max(NL(i)+tolAbs,min(delta,NR(i)-tolAbs));
            end
            DeltaArray(rp,cp,k) = delta;
         end
         rp = rp + abs(blk(i,1));
         if blk(i,2)==0
            cp = cp + abs(blk(i,1));
         else
            cp = cp + blk(i,2);
         end
      end
   end
end