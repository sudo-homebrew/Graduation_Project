function [FACT,Ml,Nl] = lncf(sys)
%LNCF  Compute left normalized coprime factorization.
%
%   LNCF computes the left normalized coprime factorization of the
%   SISO or MIMO linear system SYS:
%
%      SYS = Ml \ Nl,  Ml Ml~ + Nl Nl~ = I,  [Ml,Nl] stable.
%
%   FACT = LNCF(SYS) returns a state-space realization FACT of [Ml,Nl].
%
%   [FACT,Ml,Nl] = LNCF(SYS) also returns the coprime factors Ml and Nl 
%   separately. Note that FACT, Ml, and Nl all share the same (A,C) 
%   matrices and have the same number of states as SYS.
%
%   See also RNCF, NCFMR, NCFSYN.

%   Copyright 2018 MathWorks and MUSYN Inc.
try
   % Convert before transposing to ensure consistency with LNCF(SS(SYS))
   sys = ss(sys);
catch
   error(message('Robust:analysis:rncf2'))
end
try
   FACT = rncf(sys.');
catch ME
   throw(ME)
end
FACT = FACT.';
if nargout>1
   [p,m] = size(sys);
   Ml = subparen(FACT,{':',1:p});
   Nl = subparen(FACT,{':',p+1:p+m});
end