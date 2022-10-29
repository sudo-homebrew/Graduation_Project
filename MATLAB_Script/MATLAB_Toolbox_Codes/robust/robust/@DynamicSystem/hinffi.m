function [K,CL,GAM,INFO] = hinffi(P,nU,varargin)
%HINFFI  Full-information H-infinity synthesis.
%
%   Full-information synthesis assumes the controller has access to both
%   the state vector x and disturbance w. Use HINFSYN for the more general
%   output-feedback case when only output measurements are available.
%
%   [K,CL,GAM] = HINFFI(P,NCON) calculates the Hoo-optimal control law
%   u = K [x;w] for the LTI plant P with state-space equations:
%
%       dx =  A x +  B1 w +  B2 u
%        z = C1 x + D11 w + D12 u
%
%   NCON specifies the numbers of controls u (must be the last inputs of P).
%   The gain matrix K minimizes the H-infinity norm of the closed-loop
%   transfer function from disturbance signals w to error signals z.
%
%   [K,CL,GAM] = HINFFI(P,NCON,GAMTRY) calculates a gain matrix K that
%   achieves the closed-loop performance level GAMTRY. If GAMTRY is not
%   achievable, HINFFI returns GAM=Inf and K=CL=[].
%
%   [K,CL,GAM] = HINFFI(P,NCON,[GMIN,GMAX]) searches the range [GMIN,GMAX]
%   for the best achievable performance GAM. HINFFI returns a gain matrix
%   K with performance
%      * GAM <= GMIN when GMIN is achievable
%      * GMIN < GAM <= GMAX when GMAX but not GMIN is achievable
%   If GMAX itself is not achievable, HINFFI returns GAM=Inf and K=CL=[].
%
%   [K,...] = HINFFI(P,NCON,...,OPT) specifies additional options.
%   Use hinfsynOptions to create the option set OPT.
%
%   [K,CL,GAM,INFO] = HINFFI(P,NCON,...) also returns a structure INFO
%   with the following synthesis data:
%        gamma   Performance level used to compute K
%            X   Riccati solution Xoo for this performance level
%         Preg   Regularized plant used to compute K
%
%   Reference: "State-Space Solutions to Standard H2 and Hoo Control
%   Problems", by John Doyle, Keith Glover, Pramod Khargonekar, and Bruce
%   Francis, IEEE. Trans. Automatic Control, 34 (1989), pp. 831-847.
%
%   See also HINFSYNOPTIONS, HINFFC, HINFSYN.

%   Copyright 1986-2020 The MathWorks, Inc.

% Process Inputs
narginchk(2,4)
nin=nargin-2;
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
   if strcmp(opt.Method,'LMI')
      error(message('Robust:design:hinffi1'))
   end
else
   opt = hinfsynOptions();
end

% Check consistency
ios = iosize(P);
if nU>ios(2)
   error(message('Robust:design:h2syn3'))
elseif nmodels(P)>1
   error(message('Robust:design:h2syn6','hinffi'))
elseif hasdelay(P)
   % Deal with delays in DT systems.  Absorb them into plant directly.
   if isdt(P)
      P = absorbDelay(P);
   else
      error(message('Control:general:NotSupportedTimeDelayC','hinffi'))
   end
end

% REVISIT
try
   P = ss(P,'explicit');
catch ME
   error(message('Robust:design:h2syn5'))
end

% Setup
hw = ctrlMsgUtils.SuspendWarnings; %#ok<NASGU>
K = [];  CL = [];  GAM = Inf;  INFO = [];
GMIN0 = gamRange(1);
GMAX0 = gamRange(2);
AbsTol = opt.AbsTol;
RelTol = opt.RelTol;
ndigits = 1+max(1,ceil(-log10(RelTol)));  % number significant digits
GAMFORM = sprintf('%%.%dg',ndigits);
OPTS = struct(...
   'AbsTol',AbsTol,...
   'RelTol',RelTol,...
   'TOLREG',1e-12 * strcmp(opt.Regularize,'on'),...
   'LimitGain',true,...   % Regularize D12/D21 conservatively to avoid high gain
   'ShowProgress',strcmp(opt.Display,'on'),...
   'ndigit',ndigits,...   % GAMMA display precision
   'ScaleXUY',strcmp(opt.AutoScale,'on'),...
   'Instrument',false);

% Get data
[A,B1,B2,C1,~,D11,D12,~,~,Ts] = titodata(P,0,nU);
TU = P.TimeUnit;
if isempty(A)
   % Static problem
   nD = size(D11,2);
   [KBEST,Su,~,dD12] = rctutil.hinfStatic(D11,D12,eye(nD),GMIN0,GMAX0,OPTS);
   if isempty(KBEST)
      return
   else
      KBEST = struct('KFI',KBEST.D,'GAM',KBEST.GAM,'X',[]);
   end
   dC1 = zeros(size(dD12,1),0);
   [K,CL,GAM] = localValidate(KBEST,A,B1,B2,C1,D11,D12,Ts,TU,Su,RelTol);
   
else
   % Scaling and regularization
   if OPTS.ScaleXUY
      % Estimate GAMXY = max(GAMX,GAMY)
      GAMXE = rctutil.hinfEstimateLB(A,B1,B2,C1,D11,D12,Ts);
      % Regularize and scale X and Y Riccati equations
      [Sx,Su,dC1,dD12] = rctutil.regScaleP12(A,B1,B2,C1,D11,D12,Ts,GAMXE,OPTS);
   else
      % Regularize Riccati equations in plant coordinates
      [dC1,dD12] = rctutil.regP12(A,B2,C1,D12,Ts,OPTS);
      Sx = 1;  Su = 1;  GAMXE = NaN;
   end
   ValidateFcn = @(KSTRUCT) localValidate(KSTRUCT,A,B1,B2,C1,D11,D12,Ts,TU,Su,RelTol);
   
   % Apply Su and build regularized plant
   B2 = B2.*Su';  D12 = D12.*Su';  dD12 = dD12.*Su';
   % REVISIT: Reorganize syntax
   [D11r,C1r,D12r] = rctutil.regSynData(D11,C1,D12,[],[],dC1,dD12,[],[]);
   
   
   % Refine GMIN, abort if GMIN>GMAX
   % NOTE: Needed because Riccati solvers cannot reliably identify when
   % Hamiltonians have eigenvalues on the stability boundary. This can result
   % in false positives (central controller is stabilizing but does not
   % satisfy performance bound)
   GAMX = rctutil.hinfLowerBound(A,B1,B2,C1r,D11r,D12r,Ts,Sx,RelTol);
   if OPTS.Instrument
      fprintf('gamX = %.7g (%.7g)\n\n',GAMX,GAMXE);
   end
   GMIN = max([GMIN0,GAMX]);
   if GMIN>GMAX0
      if OPTS.ShowProgress
         disp(getString(message('Robust:design:hinfsyn12',...
            sprintf(GAMFORM,GMAX0),sprintf(GAMFORM,GMIN))))
      end
      return
   end

   % Try GMAX, abort if not achievable
   [KBEST,GINFO,GMAX] = rctutil.hinfKFI(GMAX0,A,B1,B2,C1r,D11r,D12r,Ts,Sx,OPTS);
   if isempty(KBEST)
      if OPTS.ShowProgress
         if isinf(GMAX0)
            disp(getString(message('Robust:design:hinffi2')))
         else
            disp(getString(message('Robust:design:hinfsyn14',sprintf(GAMFORM,GMAX0))))
         end
      end
      return
   end
   if GMAX0==Inf
      % For display sake, secure at least one gamma iteration when
      % H2 controller performance is optimal in range [GMIN0,GMAX0]
      GMAX = max(GMAX,(1+2*RelTol)*GMIN+2*AbsTol);
   end
   GSTOP = min(GMAX0,2*GMAX+AbsTol);  % for hinfRepairKI
   if OPTS.ShowProgress
      fprintf('\n  ')
      disp(getString(message('Robust:design:hinfsyn15',...
         sprintf(GAMFORM,GMIN),sprintf(GAMFORM,GMAX))))
      offset = floor(ndigits/2);
      fprintf('\n  %s      X>=0      p/f\n',...
         [blanks(offset) 'gamma' blanks(ndigits-offset)])
      if GMAX0<Inf || GMIN0==Inf
         % Do not display result for GMAX0=Inf unless GMINX0=Inf as well
         rctutil.hinfDisplayKFI(GINFO)
      end
   end
   
   % GAMMA iteration
   while GMAX>(1+RelTol)*GMIN+AbsTol
      if GMIN>0
         GTRY = sqrt(GMIN*GMAX);
      else
         GTRY = GMAX/10;
      end
      [KTRY,GINFO] = rctutil.hinfKFI(GTRY,A,B1,B2,C1r,D11r,D12r,Ts,Sx,OPTS);
      if OPTS.ShowProgress
         rctutil.hinfDisplayKFI(GINFO)
      end
      if isempty(KTRY)
         GMIN = GTRY;
      else
         GMAX = GTRY;  KBEST = KTRY;
      end
   end
   
   % Validate controller
   [K,CL,GAM] = ValidateFcn(KBEST);
   if OPTS.Instrument
      fprintf('RELTOL=%.3g: GMAX = %.7g vs GAM = %.7g, gap = %.3g\n',RelTol,GMAX,GAM,abs(1-GMAX/GAM))
   end
   if GAM > rctutil.hinfRepairThreshold(GMAX,RelTol,AbsTol)
      % Repair inconsistencies between original and scaled state coordinates.
      % Such inconsistencies may arise when
      %   1. The plant regularization is too small and results in a mix of
      %      very slow/very fast poles
      %   2. The original state coordinates are poorly scaled
      [KBEST,dC1,dD12,K,CL,GAM,~,OPTS] = rctutil.hinfRepairKFI(...
         dC1,dD12,A,B1,B2,C1,D11,D12,Ts,Sx,ValidateFcn,GMAX,GSTOP,OPTS);
   end
end

% Final display
if ~isempty(KBEST) && OPTS.ShowProgress
   fprintf('\n  ')
   disp(getString(message('Robust:design:hinfsyn16',sprintf(GAMFORM,GAM))))
end
   
if nargout>3
   % Build INFO
   [A,B1,B2,C1,~,D11,D12] = titodata(P,0,nU);
   [D11r,C1r,D12r] = rctutil.regSynData(D11,C1,D12,[],[],dC1,dD12./Su',[],[]);
   PREG = ss(A,[B1 B2],C1r,[D11r D12r],Ts);
   INFO = struct('gamma',KBEST.GAM,'X',KBEST.X,'Preg',PREG);
end

%---------------------------------
function [K,CL,GAM] = localValidate(KSTRUCT,A,B1,B2,C1,D11,D12,Ts,TU,Su,RelTol)
% Validates FI gain.
[nX,nD] = size(B1);
K = Su.*KSTRUCT.KFI;
Kx = K(:,1:nX);
Kw = K(:,nX+1:nX+nD);
CL = ss(A+B2*Kx,B1+B2*Kw,C1+D12*Kx,D11+D12*Kw,Ts,'TimeUnit',TU);
GAM = hinfnorm(CL,min(RelTol,1e-6));
