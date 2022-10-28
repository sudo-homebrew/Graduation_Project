function [K,CL,GAM,INFO] = h2syn(P,nY,nU,opt)
%H2SYN  H2 controller synthesis.
%
%   [K,CL,GAM] = H2SYN(P,NMEAS,NCON) calculates the H2-optimal controller
%   u = K y for the LTI plant P with state-space equations:
%
%       dx =  A x +  B1 w +  B2 u
%        z = C1 x + D11 w + D12 u
%        y = C2 x + D21 w + D22 u
%
%   NMEAS and NCON specify the numbers of measurements y and controls u
%   (u and y must be the last inputs and outputs of P). The controller K
%   minimizes the H2 norm of CL = LFT(P,K), the closed-loop transfer
%   function from disturbance signals w to error signals z.
%
%   [K,...] = H2SYN(P,NMEAS,NCON,OPT) specifies additional options. Use
%   h2synOptions to create the option set OPT.
%
%   [K,CL,GAM,INFO] = H2SYN(P,NMEAS,NCON,...) also returns a structure
%   INFO with the following synthesis data:
%           X   Solution of state-feedback Riccati equation
%           Y   Solution of observer Riccati equation
%          Ku   State-feedback gain
%       Lx,Lu   Observer gains
%        Preg   Regularized plant used to compute K
%       NORMS   Vector [FI OE DF FC] where FI is the full-information cost,
%               OE is the output-estimation cost, DF is the disturbance-
%               feedforward cost, and FC is the full-control cost:
%                  FI^2 + OE^2 = DF^2 + FC^2 = GAM^2
%         KFI   Full-information state-feedback gain u = KFI * [x;w]
%         GFI   Full-information closed-loop transfer from w to z
%               (H2 norm of GFI is FI).
%
%   The observer form of the H2-optimal controller K is
%       dxe = A xe + B2 u + Lx e
%         u = Ku xe + Lu e
%   where
%         e = y - C2 xe - D22 u
%   is the "innovation."
%
%   Reference: "Robust and Optimal Control" by Zhou, Doyle, and Glover.
%
%   See also H2SYNOPTIONS, LQG, LTRSYN, HINFSYN, SYSTUNE.

%   Copyright 2003-2018 Musyn Inc. and The MathWorks, Inc.

% Process Inputs
narginchk(1,4)
nin=nargin;
if nin==1
   %  Extract nY and nU for tito plants
   [tito,~,U2,~,Y2]=istito(P);
   if ~tito
      error(message('Robust:design:h2syn1'))
   end
   nU=length(U2);
   nY=length(Y2);
elseif nin<3
   error(message('Robust:design:h2syn1'))
end
if nin<4
   opt = rctoptions.h2syn;
elseif ~isa(opt,'rctoptions.h2syn')
   error(message('Robust:design:h2syn4'))
end

% Check consistency
ios = iosize(P);
if nY>ios(1)
   error(message('Robust:design:h2syn2'))
elseif nU>ios(2)
   error(message('Robust:design:h2syn3'))
elseif nmodels(P)>1
   error(message('Robust:design:h2syn6','h2syn'))
elseif hasdelay(P)
   % Deal with delays in DT systems.  Absorb them into plant directly.
   if isdt(P)
      P = absorbDelay(P);
   else
      error(message('Control:general:NotSupportedTimeDelayC','h2syn'))
   end
end

% Convert plant model P to @ss
try
   P = ss(P,'explicit');
catch ME
   error(message('Robust:design:h2syn5'))
end

% Suspend warnings (e.g, in Riccati solvers)
hw = ctrlMsgUtils.SuspendWarnings; %#ok<NASGU>
OPTS = struct(...
   'TOLREG',1e-12 * strcmp(opt.Regularize,'on'),...
   'LimitGain',false,...   % High gain may be required/desired (small R)
   'ScaleXUY',strcmp(opt.AutoScale,'on'),...
   'Instrument',false);

% Get state-space data from plant
[A,B1,B2,C1,C2,D11,D12,D21,D22,Ts] = titodata(P,nY,nU);
TU = P.TimeUnit;
nX = size(A,1);

if nX==0
   % Static problem
   [KOPT,Su,Sy] = localStaticH2(D11,D12,D21,OPTS);
   [K,CL,GAM,CTFF] = localValidate(KOPT,P,Ts,TU,D11,D22,Su,Sy);
   
else
   % Scaling and regularization
   Sx = struct('X',[],'Y',[]);
   if OPTS.ScaleXUY
      % Regularize and scale X and Y Riccati equations
      [Sx.X,Su,dC1,dD12] = rctutil.regScaleP12(A,B1,B2,C1,D11,D12,Ts,Inf,OPTS);
      [Sx.Y,Sy,dB1,dD21] = rctutil.regScaleP12(A',C1',C2',B1',D11',D21',Ts,Inf,OPTS);
   else
      % Regularize Riccati equations in plant coordinates
      [dC1,dD12] = rctutil.regP12(A,B2,C1,D12,Ts,OPTS);
      [dB1,dD21] = rctutil.regP12(A',C2',B1',D21',Ts,OPTS);
      Sx.X = ones(nX,1);  Sx.Y = ones(nX,1);  Su = 1;  Sy = 1;
   end
   dB1 = dB1';  dD21 = dD21';
   ValidateFcn = @(KOPT) localValidate(KOPT,P,Ts,TU,D11,D22,Su,Sy);
   
   % Apply Su and Sy and build regularized plant
   B2 = B2.*Su';  D12 = D12.*Su';  dD12 = dD12.*Su';
   C2 = Sy.*C2;  D21 = Sy.*D21;  dD21 = Sy.*dD21;
   [D11r,C1r,D12r,B1r,D21r] = rctutil.regSynData(D11,C1,D12,B1,D21,dC1,dD12,dB1,dD21);
   
   % Compute H2-optimal controller
   KOPT = rctutil.h2K(A,B1r,B2,C1r,C2,D11r,D12r,D21r,Ts,Sx);
   if isempty(KOPT)
      % Not stabilizable or detectable
      K = [];  CL = [];  GAM = Inf;  INFO = [];  return
   end
   
   % Validate controller
   [K,CL,GAM,CTFF] = ValidateFcn(KOPT);
   
   % Repair inconsistencies when CL unstable in original coordinates (can
   % happen for badly scaled, nearly singular problems like AC10).
   % Try increasing regularization level until consistency is restored.
   if isinf(GAM) && ~CTFF
      [KOPT,dC1,dD12,dB1,dD21,K,CL,GAM,CTFF] = ...
         rctutil.h2RepairK(dC1,dD12,dB1,dD21,A,B1,B2,C1,C2,D11,D12,D21,Ts,Sx,...
         ValidateFcn);
      if isempty(KOPT)
         clear hw, warning(message('Robust:design:h2syn8'))
         return
      end
   end
end

% Construct INFO
if nargout>3
   INFO = rctutil.h2MakeINFO(P,nY,nU,KOPT,dC1,dD12,dB1,dD21,Su,Sy);
end
if CTFF
   clear hw, warning(message('Robust:design:h2syn7'))
end

%---------------------------------
function [K,CL,GAM,CTFF] = localValidate(KOPT,P,Ts,TU,D11,D22,Su,Sy)
% Validates optimal controller
CTFF = false;
K = ss(KOPT.A,KOPT.B.*Sy',Su.*KOPT.C,Su.*KOPT.D.*Sy',Ts,'TimeUnit',TU);
if any(D22(:)~=0)
   K = feedback(K,ss(D22,'Ts',Ts,'TimeUnit',TU));
end
CL = lft(P,K);
if Ts==0
   if norm(CL.D,1)<=1e3*eps*norm(D11,1)
      % Prevent GAM=Inf due to Dcl=0+rounding errors
      CL.D = 0;
   else
      CTFF = true;  % there is a w->z feedthrough in CT
   end
end
GAM = norm(CL);

%---------------------------------
function [KOPT,Su,Sy] = localStaticH2(D11,D12,D21,OPTS)
% Solve static problem
% Scale D12/D21 to normalize their columns/rows
if OPTS.ScaleXUY
   Su = localColNorm(D12);    Su(Su==0) = 1;  Su = pow2(-round(log2(Su))).';
   Sy = localColNorm(D21.');  Sy(Sy==0) = 1;  Sy = pow2(-round(log2(Sy))).';
   D12 = D12 .* Su';
   D21 = Sy .* D21;
else
   Su = 1;  Sy = 1;
end
Z2X = pinv(D12);
Z2Y = pinv(D21');
DK = -Z2X * D11 * Z2Y';
[nU,nY] = size(DK);
KOPT = struct('A',[],'B',zeros(0,nY),'C',zeros(nU,0),'D',DK,...
   'X',[],'Y',[],'Ku',zeros(nU,0),'Lx',zeros(0,nY),'Z2X',Z2X,'Z2Y',Z2Y);

%-------------------------------------------------
function cnorm = localColNorm(M)
% Computes norm of columns of M
nc = size(M,2);
cnorm = zeros(1,nc);
for ct=1:nc
   cnorm(ct) = norm(M(:,ct));
end
