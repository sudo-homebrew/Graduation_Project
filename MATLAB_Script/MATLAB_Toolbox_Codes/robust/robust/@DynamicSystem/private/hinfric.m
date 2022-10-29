function [K,CL,GAM,INFO] = hinfric(P,nY,nU,GMIN0,GMAX0,opt)
% Riccati-based H-infinity synthesis.
%
% Inputs:
%   P          Plant model (@ss).
%   nY         Number of measurements.
%   nU         Number of controls.
%   GMIN0      User-specified lower bound on optimal performance.
%   GMAX0      User-specified upper bound on optimal performance.
%   OPT        Options (see HINFSYNOPTIONS).
%
% Outputs:
%   K          Optimal controller (@ss)
%   CL         Corresponding closed-loop model (@ss).
%   GAM        Optimal closed-loop performance.
%   INFO       Structure with additional synthesis data.
%
% K and CL are set to [] when the performance GMAX0 is not achievable.

% Copyright 2003-2017 Musyn Inc. and The MathWorks, Inc.

% Setup
hw = ctrlMsgUtils.SuspendWarnings; %#ok<NASGU>
K = [];  CL = [];  GAM = Inf;  INFO = [];
AbsTol = max(1e-12,opt.AbsTol);  % mitigate breakdown when GOPT=0
RelTol = opt.RelTol;
ndigits = 1+max(1,ceil(-log10(RelTol)));  % number of significant digits
GAMFORM = sprintf('%%.%dg',ndigits);
OPTS = struct(...
   'AbsTol',AbsTol,...
   'RelTol',RelTol,...
   'TOLREG',1e-12 * strcmp(opt.Regularize,'on'),...
   'LimitGain',true,...   % Regularize D12/D21 conservatively to avoid high gain
   'ndigit',ndigits,...   % GAMMA display precision
   'ShowProgress',strcmp(opt.Display,'on'),...
   'ScaleXUY',strcmp(opt.AutoScale,'on'),...
   'Instrument',false);

% Get data
[A,B1,B2,C1,C2,D11,D12,D21,D22,Ts] = titodata(P,nY,nU);
TU = P.TimeUnit;
nX = size(A,1);
if nX==0
   % Static problem
   [KBEST,Su,Sy,dD12,dD21] = rctutil.hinfStatic(D11,D12,D21,GMIN0,GMAX0,OPTS);
   if isempty(KBEST)
      return
   end
   dC1 = zeros(size(dD12,1),0);
   dB1 = zeros(0,size(dD21,2));
   [K,CL,GAM] = localValidate(KBEST,P,Ts,TU,D22,Su,Sy,RelTol);
   
else
   % Scaling and regularization
   Sx = struct('X',[],'Y',[]);
   if OPTS.ScaleXUY
      % Estimate GAMXY = max(GAMX,GAMY)
      GAMXE = rctutil.hinfEstimateLB(A,B1,B2,C1,D11,D12,Ts);
      GAMYE = rctutil.hinfEstimateLB(A',C1',C2',B1',D11',D21',Ts);
      GAMXYE = max(GAMXE,GAMYE);
      % Regularize and scale X and Y Riccati equations
      [Sx.X,Su,dC1,dD12,XINFO] = rctutil.regScaleP12(A,B1,B2,C1,D11,D12,Ts,GAMXYE,OPTS);
      [Sx.Y,Sy,dB1,dD21,YINFO] = rctutil.regScaleP12(A',C1',C2',B1',D11',D21',Ts,GAMXYE,OPTS);
      % [Sx.X,Su,Sy,dB1,dC1,dD12,dD21,XINFO,YINFO] = rctutil.hinfRegScaleXY(A,B1,B2,C1,C2,D11,D12,D21,Ts,GAMXYE,OPTS);
      % Sx.Y = 1./Sx.X;
   else
      % Regularize Riccati equations in plant coordinates
      [dC1,dD12] = rctutil.regP12(A,B2,C1,D12,Ts,OPTS);
      [dB1,dD21] = rctutil.regP12(A',C2',B1',D21',Ts,OPTS);
      Sx.X = 1;  Sx.Y = 1;  Su = 1;  Sy = 1;
      GAMXE = NaN;   GAMYE = NaN;
   end
   dB1 = dB1';  dD21 = dD21';
   ValidateFcn = @(KSTRUCT) localValidate(KSTRUCT,P,Ts,TU,D22,Su,Sy,RelTol);
   
   % Apply Su and Sy and build regularized plant
   B2 = B2.*Su';  D12 = D12.*Su';  dD12 = dD12.*Su';
   C2 = Sy.*C2;  D21 = Sy.*D21;  dD21 = Sy.*dD21;
   [D11r,C1r,D12r,B1r,D21r] = rctutil.regSynData(D11,C1,D12,B1,D21,dC1,dD12,dB1,dD21);
   
   % Visual output
   if OPTS.Instrument
      fprintf('Su = %s\n',mat2str(Su',2))
      fprintf('Sy = %s\n',mat2str(Sy',2))
      % Perturbation size
      figure(1), clf,
      subplot(211), sigma(XINFO.P12,XINFO.dP12,XINFO.dP12R,{1e-8,1e8}), grid
      title('P12 regularization')
      legend('P12','dP12 init','dP12 final')
      subplot(212), sigma(YINFO.P12,YINFO.dP12,YINFO.dP12R,{1e-8,1e8}), grid
      title('P21 regularization')
      legend('P21','dP21 init','dP21 final')
      % Sensitivity
      figure(2), clf
      subplot(211), sigma(XINFO.XSENS1,XINFO.XSENS1S,XINFO.XSENS2,XINFO.XSENS2S), grid
      title('X sensitivity')
      legend('dP12 init','dP12 initS','dP12 final','dP12 finalS')
      subplot(212), sigma(YINFO.XSENS1,YINFO.XSENS1S,YINFO.XSENS2,YINFO.XSENS2S), grid
      title('Y sensitivity')
      legend('dP21 init','dP21 initS','dP21 final','dP21 finalS')
   end
   
   
   % Refine GMIN, abort if GMIN>GMAX0
   % NOTE: Needed because Riccati solvers cannot reliably identify when
   % Hamiltonians have eigenvalues on the stability boundary. This can result
   % in false positives (central controller is stabilizing but does not
   % satisfy performance bound)
   GAMX = rctutil.hinfLowerBound(A,B1r,B2,C1r,D11r,D12r,Ts,Sx.X,RelTol);
   GAMY = rctutil.hinfLowerBound(A',C1r',C2',B1r',D11r',D21r',Ts,Sx.Y,RelTol);
   if OPTS.Instrument
      fprintf('gamX = %.7g (%.7g), gamY = %.7g (%.7g)\n\n',GAMX,GAMXE,GAMY,GAMYE);
   end
   GMIN = max([GMIN0,GAMX,GAMY]);
   if GMIN>GMAX0
      if OPTS.ShowProgress
         disp(getString(message('Robust:design:hinfsyn12',...
            sprintf(GAMFORM,GMAX0),sprintf(GAMFORM,GMIN))))
      end
      return
   end
   
   % Try user-specified GMAX0, abort if not achievable
   [KBEST,GINFO,GMAX] = rctutil.hinfKC(GMAX0,A,B1r,B2,C1r,C2,D11r,D12r,D21r,Ts,Sx,OPTS);
   if isempty(KBEST)
      if OPTS.ShowProgress
         if isinf(GMAX0)
            disp(getString(message('Robust:design:hinfsyn13')))
         else
            disp(getString(message('Robust:design:hinfsyn14',sprintf(GAMFORM,GMAX0))))
         end
      end
      return
   end
   if GMAX0==Inf
      % For display sake, secure at least one gamma iteration when 
      % H2 controller performance is optimal in range [GMIN0,GMAX0]
      % NOTE: If GMAX0<Inf, then GMAX=GMAX0>=GMIN
      GMAX = max(GMAX,(1+2*RelTol)*GMIN+2*AbsTol);
   end
   GSTOP = min(GMAX0,2*GMAX+AbsTol);  % for hinfRepairKC
   if OPTS.ShowProgress
      fprintf('\n  ')
      disp(getString(message('Robust:design:hinfsyn15',...
         sprintf(GAMFORM,GMIN),sprintf(GAMFORM,GMAX))))
      offset = floor(ndigits/2);
      fprintf('\n  %s      X>=0        Y>=0       rho(XY)<1    p/f\n',...
         [blanks(offset) 'gamma' blanks(ndigits-offset)])
      if GMAX0<Inf || GMIN0==Inf
         % Do not display result for GMAX0=Inf unless GMINX0=Inf as well
         rctutil.hinfDisplayKC(GINFO)
      end
   end
      
   % GAMMA iteration
   while GMAX>(1+RelTol)*GMIN+AbsTol
      if GMIN>0
         GTRY = sqrt(GMIN*GMAX);
      else
         GTRY = GMAX/10;
      end
      [KTRY,GINFO] = rctutil.hinfKC(GTRY,A,B1r,B2,C1r,C2,D11r,D12r,D21r,Ts,Sx,OPTS);
      if OPTS.ShowProgress
         rctutil.hinfDisplayKC(GINFO)
      end
      if isempty(KTRY)
         GMIN = GTRY;
      else
         GMAX = GTRY;  KBEST = KTRY;
      end
   end
   
   % Validate controller
   [K,CL,GAM] = ValidateFcn(KBEST);
   %fprintf('norm(DK) = %.3g, norm(DK w/ D22) = %.3g\n\n',norm(KBEST.D),norm(K.D))
   if OPTS.Instrument
      fprintf('\nRELTOL=%.3g: GMAX = %.7g vs GAM = %.7g, gap = %.3g\n',RelTol,GMAX,GAM,abs(1-GMAX/GAM))
   end
   if GAM > rctutil.hinfRepairThreshold(GMAX,RelTol,AbsTol)
      % Repair inconsistencies between original and scaled state coordinates.
      % Such inconsistencies may arise when
      %   1. The plant regularization is too small and results in a mix of
      %      very slow/very fast poles
      %   2. The original state coordinates are poorly scaled
      [KBEST,dC1,dD12,dB1,dD21,K,CL,GAM,GMAX,OPTS] = rctutil.hinfRepairKC(...
         dC1,dD12,dB1,dD21,A,B1,B2,C1,C2,D11,D12,D21,Ts,Sx,ValidateFcn,GMAX,GSTOP,OPTS);
      if isempty(KBEST)
         % Failed to stabilize or match performance of H2 controller in original
         % state coordinates
         clear hw, warning(message('Robust:design:hinfsyn20'))
         return
      end
   end
end

% Reduce gains if requested
if Ts==0 && nX>0 && strcmp(opt.LimitGain,'on')
   % Create some room consistent with RELTOL but not exceeding GMAX0
   GMAX = max(GMAX,min(GMAX0,(1+min(0.01,OPTS.RelTol))*GMIN+AbsTol));
   [KBEST,dC1,dD12,dB1,dD21,K,CL,GAM] = ...
      rctutil.hinfLimitGain(KBEST,dC1,dD12,dB1,dD21,K,CL,GAM,...
      A,B1,B2,C1,C2,D11,D12,D21,Sx,ValidateFcn,GMAX,OPTS);
   if OPTS.Instrument
      fprintf('\nLIMITING GAINS...\n')
      fprintf('FINAL: GMAX = %.7g vs GAM = %.7g, gap = %.3g\n',GMAX,GAM,abs(1-GMAX/GAM))
   end
end

if OPTS.ShowProgress
   fprintf('\n  ')
   disp(getString(message('Robust:design:hinfsyn16',sprintf(GAMFORM,GAM))))
end

% Construct INFO output
if nargout>3
   INFO = rctutil.hinfMakeINFO(P,nY,nU,KBEST,dC1,dD12,dB1,dD21,Su,Sy);
end

  
%---------------------------------
function [K,CL,GAM] = localValidate(KSTRUCT,P,Ts,TU,D22,Su,Sy,RelTol)
% Validates central controller
K = ss(KSTRUCT.A,KSTRUCT.B.*Sy',Su.*KSTRUCT.C,Su.*KSTRUCT.D.*Sy',...
   Ts,'TimeUnit',TU);
if any(D22(:)~=0)
   % REVISIT: Can we solve Parrott pb subject I+DK*D22 invertible?
   K = feedback(K,ss(D22,'Ts',Ts,'TimeUnit',TU));
end
CL = lft(P,K);
GAM = hinfnorm(CL,min(RelTol,1e-6));
