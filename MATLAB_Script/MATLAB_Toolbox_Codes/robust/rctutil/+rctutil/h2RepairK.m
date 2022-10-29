function [KOPT,dC1,dD12,dB1,dD21,K,CL,GAM,CTFF] = ...
         h2RepairK(dC1,dD12,dB1,dD21,A,B1,B2,C1,C2,D11,D12,D21,Ts,Sx,ValidateFcn)
% Repairs stability inconsistencies between the scaled/transformed coordinates 
% used for synthesis (rctutil.h2K) and the original plant coordinates. This 
% involves iteratively increasing the regularization size until inconsistencies 
% disappear.

%   Author(s): P. Gahinet
%   Copyright 2018 The MathWorks, Inc.
KOPT = [];  K = [];  CL = [];  GAM = Inf;  CTFF = false;
if size(dD12,1)>0 || size(dD21,2)>0
   for ct=1:5
      % Increase perturbation size
      dC1 = 10*dC1;  dD12 = 10*dD12;  dB1 = 10*dB1;  dD21 = 10*dD21;
      [D11r,C1r,D12r,B1r,D21r] = rctutil.regSynData(D11,C1,D12,B1,D21,dC1,dD12,dB1,dD21);
      % Recompute H2-optimal controller
      KOPT = rctutil.h2K(A,B1r,B2,C1r,C2,D11r,D12r,D21r,Ts,Sx);
      if isempty(KOPT)
         break
      end
      % Validate
      [K,CL,GAM,CTFF] = ValidateFcn(KOPT);
      if GAM<Inf
         break
      end
   end
end