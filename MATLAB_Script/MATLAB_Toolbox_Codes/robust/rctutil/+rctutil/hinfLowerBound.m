function GAMX = hinfLowerBound(A,B1,B2,C1,D11,D12,Ts,SX,RelTol)
% Computes lower bound GAMX, the smallest positive real such that
% X Riccati pencil has no eigenvalue on the stability boundary for
% all GAM>GAMX. Assumes P12 has no zero on the stability boundary.

%   Author(s): P. Gahinet
%   Copyright 2018-2021 The MathWorks, Inc.
[ne,nu] = size(D12);
if ne<nu
   % GAMX=Inf when P12 is wide
   GAMX = Inf;
elseif ne==nu
   % GAMX=0 when P12 is square invertible a.e.
   GAMX = 0;
else
   RelTol = min(1e-6,RelTol);
   % Apply scaling
   A = SX .* A ./ SX';  B1 = SX .* B1;  B2 = SX .* B2;  C1 = C1 ./ SX';
   % Test frequencies
   r = eig(A);
   if Ts>0
      r = log(r)/Ts;
   end
   fTest = abs(r(isfinite(r)));
   fTest = unique([-fTest ; 0 ; fTest]);  % for complex data
   % Compute peak gain of GammaX = (I-P12*P12+)*P11
   GAMX = ltipack.peakResidual(A,B1,B2,C1,D11,D12,[],Ts,RelTol,fTest);
   % Ensure that GAMX is not underestimating true value
   GAMX = GAMX*(1+RelTol);
end