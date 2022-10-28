function [KBEST,dC1,dD12,dB1,dD21,K,CL,GAM,GMAX,OPTS] = ...
   hinfRepairKC(dC1,dD12,dB1,dD21,A,B1,B2,C1,C2,D11,D12,D21,Ts,Sx,...
   ValidateFcn,GMAX,GSTOP,OPTS)
% Repairs stability or performance inconsistencies between the scaled 
% coordinates used for synthesis (hinfKC) and the original plant
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
nDr = size(dB1,2);
NITER = ceil(-log10(OPTS.TOLREG)/3);
ABSTOL = 10^(-2*NITER);

while true
   % Relax tolerance and increase gamma value GMAX. New acceptance level is GPASS
   OPTS.RelTol = max(sqrt(OPTS.RelTol),2*OPTS.RelTol);
   GMAX = (1+OPTS.RelTol)*GMAX;
   GPASS = rctutil.hinfRepairThreshold(GMAX,OPTS.RelTol,OPTS.AbsTol);

   % Exit for runaway GMAX
   if GMAX>GSTOP
      % No consistent solution found in specified range (GSTOP is either
      % the user-specified GMAX when finite, or twice the closed-loop 
      % performance of the H2 controller (GAM=GMAX=Inf)
      KBEST = [];  K = [];  CL = [];  GAM = Inf;  return
   end

   % Baseline controller for new GMAX
   [D11r,C1r,D12r,B1r,D21r] = rctutil.regSynData(D11,C1,D12,B1,D21,dC1,dD12,dB1,dD21);
   [KBEST,GINFO] = rctutil.hinfKC(GMAX,A,B1r,B2,C1r,C2,D11r,D12r,D21r,Ts,Sx,OPTS);
   if isempty(KBEST)
      % g2583176: Can't happen in theory, but numerically is another matter
      GAM = Inf;
   else
      if OPTS.ShowProgress
         rctutil.hinfDisplayKC(GINFO)
      end
      [K,CL,GAM] = ValidateFcn(KBEST);
      if OPTS.Instrument
         fprintf('RELTOL=%.3g: GMAX = %.7g vs GAM = %.7g, gap = %.3g\n',OPTS.RelTol,GMAX,GAM,abs(1-GMAX/GAM))
      end
   end
   if GAM<=GPASS
      % Consistency restored for new GMAX
      return
   end
   
   % Increase regularization until either GAM<GPASS or GMAX infeasible
   [MAXREGX,MAXREGY] = localComputeHeadRoom(GMAX,...
      A,B1r,B2,C1r,C2,D11r,D12r,D21r,Ts,Sx,dC1,dD12,dB1,dD21,ABSTOL);
   C1x = C1r;  D12x = D12r;  B1x = B1r;  D21x = D21r;  
   REG = 1;  REGX = 0;  REGY = 0;
   for ct=1:NITER
      REG = 10*REG;
      BOOSTX = (REG<MAXREGX);
      BOOSTY = (REG<MAXREGY);
      if BOOSTX
         C1x = [C1r ; REG*dC1];
         D12x = [D12r ; REG*dD12];
      end
      if BOOSTY
         B1x = [B1r , REG*dB1];
         D21x = [D21r , REG*dD21];
      end
      if BOOSTX || BOOSTY
         D11x = blkdiag(D11r,zeros(BOOSTX*nEr,BOOSTY*nDr));
         [KTRY,GINFO] = rctutil.hinfKC(GMAX,A,B1x,B2,C1x,C2,D11x,D12x,D21x,Ts,Sx,OPTS);
         if OPTS.ShowProgress
            rctutil.hinfDisplayKC(GINFO)
         end
         if isempty(KTRY)
            break % GMAX no longer feasible
         end
         KBEST = KTRY;
         if BOOSTX
            REGX = REG;
         end
         if BOOSTY
            REGY = REG;
         end
         [K,CL,GAM] = ValidateFcn(KTRY);
         if OPTS.Instrument
            fprintf('Regularization boost: REGX = %.3g, REGY = %.3g\n',REGX,REGY)
            fprintf('RELTOL=%.3g: GMAX = %.7g vs GAM = %.7g, gap = %.3g\n',OPTS.RelTol,GMAX,GAM,abs(1-GMAX/GAM))
         end
         if GAM<=GPASS
            % Abort when consistency restored
            break
         end
      end
   end

   % Apply largest regularization compatible with GMAX
   REGX = sqrt(1+REGX^2);  dC1 = REGX*dC1;   dD12 = REGX*dD12;
   REGY = sqrt(1+REGY^2);  dB1 = REGY*dB1;   dD21 = REGY*dD21;
   if GAM<=GPASS
      % Consistency restored with this regularization boost
      return
   end

end

%------------------------------------------
function [MAXREGX,MAXREGY] = localComputeHeadRoom(GAM,...
   A,B1,B2,C1,C2,D11,D12,D21,Ts,Sx,dC1,dD12,dB1,dD21,ABSTOL)
% Compute max multiplicative factors that can be applied to the 
% regularizing perturbations [dC1,dD12] and [dB1;dD21] without
% losing existence of the Riccati solutions Xoo and Yoo.

% Sizes and parameters
nX = size(A,1);
nU = size(B2,2);
nY = size(C2,1);
[nE,nD] = size(D11);
E = eye(nX);
RELTOL = 0.1;

% Compute head room for X
nEr = size(dC1,1);
if nEr>0
   % Compute max scaling factor that maintains GAMX<GAM (for existence of X)
   AX = Sx.X .* A ./ Sx.X';
   B1X = Sx.X .* B1;
   B2X = Sx.X .* B2;
   C1X = C1 ./ Sx.X';
   B = [B2X,B1X/GAM,zeros(nX,nE)];
   S = [zeros(nX,nD+nU) C1X'];
   aux = [D12 , D11/GAM];
   R = [diag([zeros(nU,1);-ones(nD,1)]) aux';aux -eye(nE)];
   tMaxLB = ltipack.specmax(AX,E,B,S,R,...
      zeros(nX,nEr),dC1',[dD12';zeros(nE+nD,nEr)],zeros(nEr),Ts,ABSTOL,RELTOL);
   tMaxUB = tMaxLB + max(ABSTOL,RELTOL*abs(tMaxLB));
   % Note: Use upper bound to guarantee X exists
   MAXREGX = 1/sqrt(max(0,tMaxUB));
else
   MAXREGX = NaN;
end

% Compute head room for Y
nDr = size(dB1,2);
if nDr>0
   % Compute max scaling factor that maintains GAMY<GAM (for existence of Y)
   AY = Sx.Y .\ A .* Sx.Y';
   B1Y = Sx.Y .\ B1;
   C1Y = C1 .* Sx.Y';
   C2Y = C2 .* Sx.Y';
   B = [C2Y' C1Y'/GAM zeros(nX,nD)];
   S = [zeros(nX,nE+nY) B1Y];
   aux = [D21 ; D11/GAM];
   R = [diag([zeros(nY,1);-ones(nE,1)]) aux;aux' -eye(nD)];
   tMaxLB = ltipack.specmax(AY',E',B,S,R,...
      zeros(nX,nDr),dB1,[dD21;zeros(nE+nD,nDr)],zeros(nDr),Ts,ABSTOL,RELTOL);
   tMaxUB = tMaxLB + max(ABSTOL,RELTOL*abs(tMaxLB));  % upper bound
   MAXREGY = 1/sqrt(max(0,tMaxUB));
else
   MAXREGY = NaN;
end
   