function [K,CL,GAM,INFO]=hinfsyn(P,nY,nU,varargin)
%HINFSYN  H-infinity controller synthesis.
%
%   [K,CL,GAM] = HINFSYN(P,NMEAS,NCON) calculates the Hoo-optimal
%   controller u = K y for the LTI plant P with state-space equations:
%
%       dx =  A x +  B1 w +  B2 u
%        z = C1 x + D11 w + D12 u
%        y = C2 x + D21 w + D22 u
%
%   NMEAS and NCON specify the numbers of measurements y and controls u
%   (u and y must be the last inputs and outputs of P). The controller K
%   minimizes the H-infinity norm of CL = LFT(P,K), the closed-loop
%   transfer function from disturbance signals w to error signals z.
%
%   [K,CL,GAM] = HINFSYN(P,NMEAS,NCON,GAMTRY) calculates the H-infinity
%   controller for the performance level GAMTRY. When GAMTRY is feasible,
%   GAM is the actual closed-loop performance obtained with this controller
%   (GAM<=GAMTRY). Otherwise GAM=Inf and K and CL are set to [].
%
%   [K,CL,GAM] = HINFSYN(P,NMEAS,NCON,[GMIN,GMAX]) searches the range
%   [GMIN,GMAX] for the best achievable performance GAM. HINFSYN returns
%   a controller K with performance
%      * GAM <= GMIN when GMIN is achievable
%      * GMIN < GAM <= GMAX when GMAX but not GMIN is achievable
%   If GMAX itself is not achievable, HINFSYN returns GAM=Inf and K=CL=[].
%
%   [K,...] = HINFSYN(P,NMEAS,NCON,...,OPT) specifies additional options.
%   Use hinfsynOptions to create the option set OPT.
%
%   [K,CL,GAM,INFO] = HINFSYN(P,NMEAS,NCON,...) also returns a structure
%   INFO with additional synthesis data. For Riccati-based synthesis,
%   this includes
%       gamma   Performance level used to compute K
%           X   Riccati solution Xoo for this performance level
%           Y   Riccati solution Yoo for this performance level
%       Ku,Kw   State-feedback gains
%       Lx,Lu   Observer gains
%        Preg   Regularized plant used to compute K
%          AS   All controllers with performance INFO.gamma are given by
%               K = lft(INFO.AS,Q) where Q is any stable transfer function
%               of size [NCON NMEAS] with peak gain less than INFO.gamma.
%   For LMI-based synthesis, INFO contains the best performance GAMMA and
%   the corresponding LMI solutions R and S.
%
%   The observer form of the controller K is
%       dxe = A xe + B1 we + B2 u + Lx e
%         u = Ku xe + Lu e
%        we = Kw xe
%   where we is an estimate of the worst-case perturbation and
%         e = y - C2 xe - D21 we - D22 u
%   plays the role of "innovation."
%
%   See also HINFSYNOPTIONS, HINFFI, HINFFC, HINFSTRUCT, H2SYN, LTRSYN.

%   Copyright 2003-2017 Musyn Inc. and The MathWorks, Inc.

% Process Inputs
nin=nargin;
nout=nargout;
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

% Parse remaining inputs
nin = nin-3;
if nin>0 && isnumeric(varargin{1})
   gamRange = varargin{1};
   varargin = varargin(2:end);  nin=nin-1;
   if isscalar(gamRange)
      if ~(isreal(gamRange) && gamRange>=0)
         error(message('Robust:design:hinfsyn5'))
      end
      gamRange = gamRange(:,[1 1]);
   elseif ~(numel(gamRange)==2 && isreal(gamRange) && ...
         all(gamRange>=0) && gamRange(1)<=gamRange(2))
      error(message('Robust:design:hinfsyn6'))
   end
else
   gamRange = [0,Inf];   % default
end
if nin>0 && isa(varargin{1},'rctoptions.hinfsyn')
   opt = varargin{1};
   varargin = varargin(2:end);  nin=nin-1;
else
   opt = hinfsynOptions();
end
if rem(nin,2)==0
   % PV pairs from per-R2018a syntax
   try
      [opt,gamRange] = localRemapOptions(opt,gamRange,varargin{:});
   catch ME
      throw(ME)
   end
else
   error(message('Robust:design:hinfsyn7'))
end
   
   
% Check consistency
ios = iosize(P);
if nY>ios(1)
   error(message('Robust:design:h2syn2'))
elseif nU>ios(2)
   error(message('Robust:design:h2syn3'))
elseif nmodels(P)>1
   error(message('Robust:design:h2syn6','hinfsyn'))
elseif hasdelay(P)
   % Deal with delays in DT systems.  Absorb them into plant directly.
   if isdt(P)
      P = absorbDelay(P);
   else
      error(message('Control:general:NotSupportedTimeDelayC','hinfsyn'))
   end
end

% Convert plant model P to explicit state-space
% REVISIT: LMI only ultimately
try
   P = ss(P,'explicit');
catch ME
   error(message('Robust:design:h2syn5'))
end

try
   switch opt.Method
      case 'LMI'
         % LMI-based synthesis
         LMIopt = [opt.LimitRS([1 1]) , opt.TolRS, strcmp(opt.Display,'off')];
         Ts = getTs(P);
         if Ts==0
            [gopt,K,R,~,S] = hinflmi(lti2mat(P),[nY,nU],gamRange(1),opt.RelTol,LMIopt);
         else
            [gopt,K,R,~,S] = dhinflmi(lti2mat(P),[nY,nU],gamRange(1),opt.RelTol,LMIopt);
         end
         if isempty(K)
            CL = [];  GAM = Inf;  INFO = [];  return
         end
         K = setTs(mat2lti(K),Ts);
         CL = lft(P,K);
         GAM = getPeakGain(CL,min(1e-6,opt.RelTol));
         if GAM>gamRange(2)
            K = [];  CL = [];  GAM = Inf;  INFO = [];  return
         end
         INFO = struct('gamma',gopt,'R',R,'S',S);
         
      case 'RIC'
         % Riccati-based synthesis
         if nout<4
            [K,CL,GAM] = hinfric(P,nY,nU,gamRange(1),gamRange(2),opt);
         else
            [K,CL,GAM,INFO] = hinfric(P,nY,nU,gamRange(1),gamRange(2),opt);
         end
         
      case 'MAXE'
         % Max-entropy controller
         [K,CL,GAM,INFO] = hinfric(P,nY,nU,gamRange(1),gamRange(2),opt);
         if ~isempty(INFO.AS) && isct(P)
            K = localMaxEntropyK(P,nY,nU,INFO,opt.S0);
            CL = lft(P,K);
            %[getPeakGain(CL,1e-6) INFO.gamma]
            GAM = getPeakGain(CL,min(1e-6,opt.RelTol));
         end
         
      otherwise
         % Call Andy/Pete's code
         [K,CL,GAM,INFO] = hinfsynSimpleEpsIterScale(P,nY,nU,gamRange,opt);
   end
catch ME
   throw(ME)
end


%--------------------------------------------------------------------------
function [opt,gamRange] = localRemapOptions(opt,gamRange,varargin)
% Remap pre-R2018a PV pairs to new options
%      'GMAX'   | real   | initial upper bound on GAM (Inf default)
%      'GMIN'   | real   | initial lower bound on GAM (0 default)
%      'TOLGAM' | real   | relative error tolerance for GAM (.01 default)
%      'METHOD' |        | H-infinity solution method:
%   	          |'ric'   | - (default) standard 2-Riccati solution, DGKF1989
% 	             |'lmi'   | - LMI solution, Packard 1992, Gahinet 1994
%               |'maxe'  | - maximum entropy, HINFSYNE, Glover-Doyle 1988
%      'S0'     | real   |  (default=Inf) frequency S0 at which entropy is
%               |        |      evaluated, only applies to METHOD 'maxe'
%      'DISPLAY'|'on/off'| display synthesis information to screen,
%               |        |  (default = 'off')
for ct=1:2:numel(varargin)
   Key = ltipack.matchKey(varargin{ct},...
      {'GMAX','GMIN','TOLGAM','METHOD','S0','DISPLAY','LMIOPTIONS','OPTIONS'});
   if isempty(Key)
      error(message('Robust:design:hinfsyn8'))
   else
      Value = varargin{ct+1};
      try
         switch Key
            case 'GMAX'
               gamRange(2) = Value;
            case 'GMIN'
               gamRange(1) = Value;
            case 'TOLGAM'
               opt.RelTol = Value;
            case 'METHOD'
               opt.Method = Value;
            case 'S0'
               opt.S0 = Value;
            case 'DISPLAY'
               opt.Display = Value;
            case {'OPTIONS','LMIOPTIONS'}
               opt.LimitRS = max(Value(1:2));
               opt.TolRS = Value(3);
         end
      catch
         error(message('Robust:design:hinfsyn9',Key))
      end
   end
end
% Validate range
if ~(isnumeric(gamRange) && numel(gamRange)==2 && isreal(gamRange) && ...
      all(gamRange>=0) && gamRange(1)<=gamRange(2))
   error(message('Robust:design:hinfsyn10'))
end


function K = localMaxEntropyK(P,nY,nU,INFO,S0)
% Compute max-entropy controller
AS = INFO.AS;
[nEY,nDU] = size(P);
gammaPass = INFO.gamma;
nD = nDU - nU;
nE = nEY - nY;
idxU = nD+1:nDU;
idxY = nE+1:nEY;
cla = lft(P,AS,nU,nY);
[a,b,c,d] = ssdata(cla);
nX = size(a,1);
if isinf(S0)
   Phi = d(idxY,idxU);
else
   Phi = d(idxY,idxU) + (c(idxY,:)/(eye(nX)*S0-a))*b(:,idxU);
end
%[norm(Phi) 1/gammaPass]
K = lft(AS,gammaPass^2*Phi');


