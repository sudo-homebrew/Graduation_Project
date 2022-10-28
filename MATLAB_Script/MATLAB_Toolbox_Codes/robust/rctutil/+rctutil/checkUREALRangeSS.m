function [muBnds,UBcert,LBcert,OutOfRangePert] = ...
   checkUREALRangeSS(muBnds,UBcert,LBcert,NL,NR,blk)
% Enforces limits on normalized range for UREALs.

%   Copyright 2010-2017 The MathWorks, Inc.

% For UREALs with asymmetric range, [NL,NR] is the interval where 
% normalized2actual is well defined. Otherwise [NL,NR] = [-Inf,Inf].
Limit = min([abs(NL);NR]);
OutOfRangePert = (Limit * cat(1,LBcert.LB)<1);

if isfinite(Limit)
   % Normalized range is constrained by some UREALs. Limit the margin to
   % the range [-Limit,Limit] where normalized2actual is well defined,
   % and handle case when destabilizing delta is outside [NL,NR].
   minMU = 1/Limit;
   rtol = 1e3*eps;
   tolAbs = rtol * Limit;
   
   muBnds = max(minMU,muBnds);
   for k=1:numel(UBcert)
      UBcert(k).gUB = max(minMU, UBcert(k).gUB);
   end

   for k=1:numel(LBcert)
      LBcert(k).LB = max(minMU,LBcert(k).LB);
      % Adjust worst-case perturbation to fit inside normalized range [NL,NR]
      rp = 1;
      cp = 1;
      for i=1:size(blk,1)
         if blk(i,1)<0
            % UREAL block
            delta = LBcert(k).Delta(rp,cp);
            if delta<NL(i)-tolAbs
               % Nearest valid value is normalized2actual(inf)
               delta = Limit/rtol;
            elseif delta>NR(i)+tolAbs
               % Nearest valid value is normalized2actual(-inf)
               delta = -Limit/rtol;
            else
               delta = max(NL(i)+tolAbs,min(delta,NR(i)-tolAbs));
            end
            LBcert(k).Delta(rp,cp) = delta;
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