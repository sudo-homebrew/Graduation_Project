function [KBEST,dC1,dD12,dB1,dD21,K,CL,GAM] = hinfLimitGain(KBEST,dC1,dD12,dB1,dD21,K,CL,GAM,...
      A,B1,B2,C1,C2,D11,D12,D21,Sx,ValidateFcn,GMAX,OPTS)
% Iterative refinement of controller to reduce high gains due to large
% ||B2 D12+|| or ||D21+ C2|| (continuous time only).

%   Author(s): P. Gahinet
%   Copyright 2018 The MathWorks, Inc.

% Absorb regularization
nD0 = size(D11,2);
[D11,C1,D12,B1,D21] = rctutil.regSynData(D11,C1,D12,B1,D21,dC1,dD12,dB1,dD21);

% Apply state scalings and check for high gains
AX = Sx.X .* A ./ Sx.X'; 
B2X = Sx.X .* B2;
CK = KBEST.C ./ Sx.X';
nB2CK = norm(B2X*CK,1);
BOOSTX = (nB2CK>1+10*norm(AX,1));  % B2*CK large

AY = Sx.Y .\ A .* Sx.Y';
C2Y = C2 .* Sx.Y';
BK = Sx.Y .\ KBEST.B;
nBKC2 = norm(BK*C2Y,1);
BOOSTY = (nBKC2>1+10*norm(AY,1));  % BK*C2 large

if ~(BOOSTX || BOOSTY)
   return
end
AK0 = KBEST.A;

% Sizes and parameters
nX = size(A,1);
nU = size(B2,2);
nY = size(C2,1);
[nE,nD] = size(D11);
E = eye(nX);
MAXREG = 1e8;         % up to 8 iterations
ABSTOL = 1/MAXREG^2;  % only care about tMax>1/MAXREG^2
RELTOL = 0.1;

% Compute initial regularization level
MAXREGX = NaN;  MAXREGY = NaN;
if BOOSTX
   B1X = Sx.X .* B1;
   C1X = C1 ./ Sx.X';
   SCALE12 = 1+norm([AX B2X;C1X D12],1);
   DELTA12 = max(norm(dD12,1),sqrt(SCALE12*OPTS.TOLREG)) * eye(nU);
   % Compute max level to maintain GAMX<GMAX
   B = [B2X,B1X/GMAX,zeros(nX,nE)];
   S = [zeros(nX,nD+nU) C1X'];
   aux = [D12 , D11/GMAX];  NULL = zeros(nX,nU);
   R = [diag([zeros(nU,1);-ones(nD,1)]) aux';aux -eye(nE)];
   tMaxLB = ltipack.specmax(AX,E,B,S,R,NULL,NULL,[DELTA12';zeros(nE+nD,nU)],zeros(nU),0,ABSTOL,RELTOL);  
   tMaxUB = tMaxLB + max(ABSTOL,RELTOL*abs(tMaxLB));
   % Note: Use upper bound to guarantee X exists
   MAXREGX = min(MAXREG,1/sqrt(max(0,tMaxUB)));
end
if BOOSTY
   B1Y = Sx.Y .\ B1;
   C1Y = C1 .* Sx.Y';
   SCALE21 = 1+norm([AY B1Y;C2Y D21],1);
   DELTA21 = max(norm(dD21,inf),sqrt(SCALE21*OPTS.TOLREG)) * eye(nY);
   % Compute max level to maintain GAMY<GMAX
   B = [C2Y' C1Y'/GMAX zeros(nX,nD)];
   S = [zeros(nX,nE+nY) B1Y];
   aux = [D21 ; D11/GMAX];  NULL = zeros(nX,nY);
   R = [diag([zeros(nY,1);-ones(nE,1)]) aux;aux' -eye(nD)];
   tMaxLB = ltipack.specmax(AY',E',B,S,R,...
      NULL,NULL,[DELTA21';zeros(nE+nD,nY)],zeros(nY),0,ABSTOL,RELTOL);  
   tMaxUB = tMaxLB + max(ABSTOL,RELTOL*abs(tMaxLB));  % upper bound
   MAXREGY = min(MAXREG,1/sqrt(max(0,tMaxUB)));
end

if OPTS.Instrument
   fprintf('MAXREGX = %.3g, MAXREGY = %.3g\n',MAXREGX,MAXREGY)
end

% Increase regularization level until violating GMAX bound. Note that
%   * GAMX and GAMY are monotonically increasing with REG
%   * inv(X) and inv(Y) are monotonically decreasing with REG
MAXREG = 0.9*max(MAXREGX,MAXREGY);
NITER = floor(log10(3*MAXREG));  % mid-decade cutoff
if NITER>0 && OPTS.ShowProgress
   fprintf('  ')
   disp(getString(message('Robust:design:hinfsyn18')))
end
D12r = D12;  C1r = C1;  D21r = D21;  B1r = B1;
REG = 1;  REGX = 0;  REGY = 0;
for ct=1:NITER
   REG = min(10*REG,MAXREG);
   BOOSTX = BOOSTX && (REG<=MAXREGX);
   BOOSTY = BOOSTY && (REG<=MAXREGY);
   if BOOSTX
      D12r = [D12 ; REG*DELTA12];
      C1r = [C1 ; zeros(nU,nX)];
   end
   if BOOSTY
      D21r = [D21 , REG*DELTA21];
      B1r = [B1 , zeros(nX,nY)];
   end
   if BOOSTX || BOOSTY
      % Regularization increased for D12 or D21
      D11r = blkdiag(D11,zeros(size(C1r,1)-nE,size(B1r,2)-nD));
      [KTRY,GINFO] = rctutil.hinfKC(GMAX,A,B1r,B2,C1r,C2,D11r,D12r,D21r,0,Sx,OPTS);
      if OPTS.ShowProgress
         rctutil.hinfDisplayKC(GINFO)
      end
      if isempty(KTRY)
         break  % GMAX not achievable for REG level
      end
      % Compute actual closed-loop performance and assess gain reduction
      % Note: Using ||AX|| leads to premature termination when A has fast poles (LS1.PBAD)
      [K_,CL_,GAM_] = ValidateFcn(KTRY);
      gX = norm(B2X*(KTRY.C./Sx.X'),1);  gDROPX = (gX<0.9*nB2CK);
      gY = norm((Sx.Y.\KTRY.B)*C2Y,1);   gDROPY = (gY<0.9*nBKC2);
      if ~(gDROPX || gDROPY)
         % No progress: abort
         break
      elseif GAM_<GMAX*(1+OPTS.RelTol)
         % Some gain reduction. Only accept KTRY when actual closed-loop
         % performance satisfies the relative tolerance in force.
         % Otherwise just keep reducing gains until inconsistencies are 
         % resolved
         KBEST = KTRY;  K = K_;  CL = CL_;  GAM = GAM_;
         if BOOSTX
            REGX = REG;
         end
         if BOOSTY
            REGY = REG;
         end
         BOOSTX = gDROPX;   BOOSTY = gDROPY;  nB2CK = gX;   nBKC2 = gY;
      end
   end
end

if OPTS.Instrument
   fprintf('REGX = %.5g, REGY = %.5g\n',REGX,REGY)
   fprintf('Gain reduction loop: ||AK|| reduced from %.3g to %.3g\n',norm(AK0,1),norm(KBEST.A,1));
end

% Update regularized matrices
if REGX>0
   [Q,dD12] = qr([dD12 ; REGX*DELTA12],0);
   dC1 = Q(1:size(dC1,1),:)' * dC1;
end
if REGY>0
   [Q,dD21] = qr([dD21 , REGY*DELTA21]',0);
   dD21 = dD21';
   dB1 = dB1 * Q(1:size(dB1,2),:);
   KBEST.Kw = [KBEST.Kw(1:nD0,:) ; Q'*KBEST.Kw(nD0+1:end,:)];
end