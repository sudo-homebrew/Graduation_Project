function [KBEST,dC1,dD12,K,CL,GAM,GMAX,OPTS] = ...
   hinfRepairKFI(dC1,dD12,A,B1,B2,C1,D11,D12,Ts,Sx,...
   ValidateFcn,GMAX,GSTOP,OPTS)
% Repairs stability or performance inconsistencies between the scaled 
% coordinates used for synthesis (hinfKFI) and the original plant
% coordinates. This involves iteratively relaxing the tolerance and
% increasing the regularization size until the inconsistencies disappear.

%   Author(s): P. Gahinet
%   Copyright 2018-2021 The MathWorks, Inc.

if OPTS.ShowProgress
   fprintf('  ')
   disp(getString(message('Robust:design:hinfsyn17')))
end

% Sizes and parameters
nEr = size(dC1,1);
nD = size(D11,2);
NITER = ceil(-log10(OPTS.TOLREG)/3);
ABSTOL = 10^(-2*NITER);

while true
   % Relax tolerance and increase gamma value GMAX. New acceptance level is GPASS
   OPTS.RelTol = max(sqrt(OPTS.RelTol),2*OPTS.RelTol);
   GMAX = (1+OPTS.RelTol)*GMAX;
   GPASS = rctutil.hinfRepairThreshold(GMAX,OPTS.RelTol,OPTS.AbsTol);

   % Exit for runaway GMAX
   if GMAX>GSTOP
      % No consistent solution found in specified range
      KBEST = [];  K = [];  CL = [];  GAM = Inf;  return
   end

   % Baseline controller for new tolerance
   [D11r,C1r,D12r] = rctutil.regSynData(D11,C1,D12,[],[],dC1,dD12,[],[]);
   [KBEST,GINFO] = rctutil.hinfKFI(GMAX,A,B1,B2,C1r,D11r,D12r,Ts,Sx,OPTS);
   if isempty(KBEST)
      % g2583176: Can't happen in theory, but numerically is another matter
      GAM = Inf;
   else
      if OPTS.ShowProgress
         rctutil.hinfDisplayKFI(GINFO)
      end
      [K,CL,GAM] = ValidateFcn(KBEST);
      if OPTS.Instrument
         fprintf('RELTOL=%.3g: GMAX = %.7g vs GAM = %.7g, gap = %.3g\n',OPTS.RelTol,GMAX,GAM,abs(1-GMAX/GAM))
      end
   end
   if GAM<=GPASS
      return
   end
   
   % Increase regularization until either GAM<GPASS or GAM infeasible
   MAXREGX = localComputeHeadRoom(GMAX,A,B1,B2,C1r,D11r,D12r,Ts,Sx,dC1,dD12,ABSTOL);
   REG = 1;  REGX = 0;
   for ct=1:NITER
      REG = 10*REG;
      BOOSTX = (REG<MAXREGX);
      if BOOSTX
         C1x = [C1r ; REG*dC1];
         D12x = [D12r ; REG*dD12];
         D11x = [D11r ; zeros(nEr,nD)];
         [KTRY,GINFO] = rctutil.hinfKFI(GMAX,A,B1,B2,C1x,D11x,D12x,Ts,Sx,OPTS);
         if OPTS.ShowProgress
            rctutil.hinfDisplayKFI(GINFO)
         end
         if isempty(KTRY)
            break
         end
         KBEST = KTRY;
         REGX = REG;
         [K,CL,GAM] = ValidateFcn(KTRY);
         if OPTS.Instrument
            fprintf('Regularization boost: REGX = %.3g\n',REGX)
            fprintf('RELTOL=%.3g: GMAX = %.7g vs GAM = %.7g, gap = %.3g\n',OPTS.RelTol,GMAX,GAM,abs(1-GMAX/GAM))
         end
         if GAM<=GPASS
            break
         end
      end
   end
   % Apply largest regularization compatible with GMAX
   REGX = sqrt(1+REGX^2);   dC1 = REGX*dC1;   dD12 = REGX*dD12;
   if GAM<=GPASS
      return
   end
end


%------------------------------------------
function MAXREGX = localComputeHeadRoom(GAM,A,B1,B2,C1,D11,D12,Ts,Sx,dC1,dD12,ABSTOL)
% Compute max multiplicative factors that can be applied to the 
% regularizing perturbation [dC1,dD12] without losing existence of 
% the Riccati solutions Xoo.

% Sizes and parameters
nX = size(A,1);
nU = size(B2,2);
[nE,nD] = size(D11);
E = eye(nX);
RELTOL = 0.1;

% Compute head room for X
nEr = size(dC1,1);
if nEr>0
   % Compute max scaling factor that maintains GAMX<GAM (existence of Xoo)
   AX = Sx .* A ./ Sx';
   B1X = Sx .* B1;
   B2X = Sx .* B2;
   C1X = C1 ./ Sx';
   B = [B2X,B1X/GAM,zeros(nX,nE)];
   S = [zeros(nX,nD+nU) C1X'];
   aux = [D12 , D11/GAM];
   R = [diag([zeros(nU,1);-ones(nD,1)]) aux';aux -eye(nE)];
   tMaxLB = ltipack.specmax(AX,E,B,S,R,...
      zeros(nX,nEr),dC1',[dD12';zeros(nE+nD,nEr)],zeros(nEr),Ts,ABSTOL,RELTOL);
   tMaxUB = tMaxLB + max(ABSTOL,RELTOL*abs(tMaxLB));
   % Note: Use upper bound to guarantee Xoo exists
   MAXREGX = 1/sqrt(max(0,tMaxUB));
else
   MAXREGX = NaN;
end