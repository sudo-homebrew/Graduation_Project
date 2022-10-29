function [K,muPerf,IterArray] = musynEngine(P0,BLK,BLKnames,nY,nU,Kinit,opt)
% Engine for mu synthesis based on either HINFSYN or HINFSTRUCT.
%
% The uncertain plant P is:
%    HINFSYN      open-loop USS mapping [d; u] to [e; y]
%    HINFSTRUCT   closed-loop GENSS mapping d to e.
%
% Inputs
%    P0     Nominal part of the LFT such that P=lft(Delta,P0).
%           P0 is SS for HINFSYN and GENSS for HINFSTRUCT.
%    BLK    Standard N-by-2 description of the uncertainty blocks. This
%           contains the performance block [nD, nE]
%    nY     Number of measurements. This is empty for HINFSTRUCT.
%    nU     Number of controls. This is empty for HINFSTRUCT.
%    Kinit  Initial controller. Set as empty if no initialization.
%    opt    Option set created with musynOptions.
%
% Outputs
%    K      Controller in SS form for HINFSYN or as a structure of blocks
%           values for HINFSTRUCT.

%   Copyright 2019 The MathWorks, Inc.

% Mixed-MU Information
synBLK = BLK;
if strcmp(opt.MixedMU,'on') && all(BLK(:,1)>0)
   % No real parameters. Turn off MixedMU synthesis
   opt.MixedMU = 'off';
elseif strcmp(opt.MixedMU,'off') && any(BLK(:,1)<0)
   % There are real parameters but MixedMu=='off'. The block structure
   % used for synthesis (synBLK) is different than the actual block
   % structure (BLK) with all [-n 0] replaced by [n 0].
   synBLK = abs(BLK);
end

% Tolerances
TolMU = rctoptions.musyn.TolMU;  % accuracy of MU computation
TolNORM = TolMU/10;              % for peak gain computation
TolHINF = TolMU;                 % for synthesis

% Domain
Ts = P0.Ts;  % Note: assumed positive at this point
if Ts==0
   FRANGE = [1e-8,1e8];
else
   % Note: Fitting D scalings over huge range leads to poles clustering 
   % near z=1, which is often fatal for HINFSYN. So FRANGE(1) can't be too
   % small, and pi/Ts can't be much larger than the bandwidth (MUSYN fails
   % when Ts is too small)
   nf = pi/Ts;
   FRANGE = [1e-8*nf,nf];
end

% HINFSYN/HINFSTRUCT setup
useHINFSTRUCT = isa(P0,'genss');
if useHINFSTRUCT
   CL = P0;  % tunable closed-loop model d->e
   DGCL = CL;    % scaled tunable closed-loop model
   hOpt = opt.hinfstructOPT;
   hOpt.TolGain = TolHINF;
else
   IC = P0;  % unscaled open-loop model [d;u] -> [e;y] (not modified)
   DGIC = IC;    % scaled open-loop model
   hOpt = opt.hinfsynOPT;
   hOpt.RelTol = TolHINF;
end
   
% D/G-K Iteration
hw = ctrlMsgUtils.SuspendWarnings; %#ok<NASGU>
MAXITER = opt.MaxIter;
IterArray = struct('gamma',cell(MAXITER,1),'K',[],'KInfo',[],...
   'PeakMu',[],'DG',[],...
   'PeakMuFit',[],'FitOrder',[],'dr',[],'dc',[],'PSI',[]);
FULLDISP = strcmp(opt.Display,'full');
MixedMU =  strcmp(opt.MixedMU,'on');
BestPerf = Inf;
BestPerfHist = Inf(MAXITER,1);
NSETTLE = 2;  % window for assessing lack of progress
PLOTS = struct('MU',[],'DG',[],'BlockNames',{BLKnames});  % full display plots
for iter=1:MAXITER
   if FULLDISP
      fprintf('\n\n======================= ITERATION %d ========================\n',iter)
   end
   
   % ------------------- K-Step
   if iter==1 && ~isempty(Kinit)
      % Use initial controller/parameters provided as input
      K = Kinit;  
      if ~useHINFSTRUCT
         CL = lft(IC,K);  DGCL = CL;
      end
      gAch = hinfnorm(DGCL,TolNORM);
      KInfo = [];
   else
      % Controller Synthesis
      if FULLDISP
         fprintf('\nCONTROLLER SYNTHESIS (K-STEP):\n')
      end
      if useHINFSTRUCT
         [DGCL,gAch,KInfo] = hinfstruct(DGCL,hOpt);
         K = DGCL.Blocks;
         CL = setBlockValue(CL,K);
      else
         % Note: Don't constraint GMAX, regularization and other factors may break it
         [K,DGCL,gAch,KInfo] = hinfsyn(DGIC,nY,nU,hOpt);
         CL = lft(IC,K);
      end
      if iter>1
         gAch = gAch*PeakMuSmooth;
%          if gAch>(1+TolHINF)*PeakMuFit
%             fprintf('**WARNING: Hinf performance not improving!\n')
%          end
      end
   end
   
   % ------------------- D-Step
   if isfinite(gAch)
      % Compute MU bounds
      if FULLDISP
         if MixedMU
            fprintf('\n\nMU ANALYSIS (DG-STEP):\n\n')
         else
            fprintf('\n\nMU ANALYSIS (D-STEP):\n\n')
         end
      end
      % NOTE: Use mussv(CLss,...) to always capture peak when various
      % issues fixed (g1978688)
      CLss = ss(CL);  % closed-loop system
      if isempty(opt.FrequencyGrid)
         OPTS = struct('Focus',FRANGE,'Negative',false,'Decades',false);
         OMEGA = freqgrid({tzero(CLss)},{pole(CLss)},Ts,4,OPTS);
      else
         OMEGA = opt.FrequencyGrid;
         OMEGA = OMEGA(0<=OMEGA & OMEGA<=0.9*pi/Ts);
      end
      CLg = frd(CLss,OMEGA);
      [uppermu,~,DGInfo] = mussvSmoothDG(CLg,synBLK,opt.FullDG);
      PeakMu = max(uppermu);
      PeakMuSmooth = max(DGInfo.ub);
      BestPerf = min(BestPerf,PeakMu); % Best performance so far
      
      % Fit D,G data
      [PSI,dr,dc,FitOrder,PLOTS] = fitDG(CLg,synBLK,DGInfo,opt,PLOTS);

      % Update scaled open- or closed-loop model for next K step
      if useHINFSTRUCT
         DGCL = dr * CL * inv(dc,'min'); %#ok<MINV>
         DGCL = sector2gain(DGCL,PSI);
      else
         DGIC = blkdiag(dr,eye(nY)) * IC * blkdiag(inv(dc,'min'),eye(nU));
         DGIC = sector2gain(DGIC,PSI,nY,nU);
         DGCL = lft(DGIC,K);
      end
      % Note: Use fixed tolerance here to avoid dependence of GMAX,K on TolPerf
      PeakMuFit = PeakMuSmooth * getPeakGain(DGCL,TolNORM);
      
      if FULLDISP
         % Show scaled closed-loop norm vs mu upper bound
         PLOTS = musynShowMU(uppermu,DGInfo,DGCL,CLg,MixedMU,PLOTS);
      end
   else
      PeakMu = inf; PeakMuFit = inf;
   end
   
   % ------------------- Store and display iteration results
   BestPerfHist(iter) = BestPerf;
   IterArray(iter).gamma = gAch;
   IterArray(iter).PeakMu = PeakMu;
   IterArray(iter).PeakMuFit = PeakMuFit;
   if PeakMu<Inf
      IterArray(iter).K = K;
      IterArray(iter).KInfo = KInfo;
      IterArray(iter).DG = rmfield(DGInfo,'FitRange');
      IterArray(iter).dr = dr;
      IterArray(iter).dc = dc;
      IterArray(iter).PSI = PSI;
      IterArray(iter).FitOrder = FitOrder;
   end
   musynDispSummary( IterArray, iter, opt )
   
   % ------------------- Check stopping conditions
   %  1) Performance  <= opt.TargetPerf
   %  2) Iteration count  >= opt.MaxIter
   %  3) Insufficient progress over last 2 steps
   % Conditions 3 is not checked if opt.TolPerf=0 (complete MaxIter iterations)
   if PeakMu<=opt.TargetPerf || gAch==Inf
      % Unconditional exits
      break
   elseif iter>NSETTLE && opt.TolPerf>0 && ...
         (1+opt.TolPerf)*BestPerf > BestPerfHist(iter-NSETTLE)
      % Terminate when best peak MU settles down (progress over last two
      % iterations falls below TolPerf). This covers cases when PeakMu 
      % slowly increases or oscillates between two values.
      break
   end
            
   % Prompt users to continue
   if FULLDISP
      yns = input('\n\nAnother D-K iteration ([y]/n)?','s');
      if strcmp(yns,'n')
         break
      end
   end

end

% Select best controller
IterArray = IterArray(1:iter);
[muPerf,idx] = min(BestPerfHist);
K = IterArray(idx).K;

% Closing report
if ~strcmp(opt.Display,'off')
   fprintf('\nBest achieved robust performance: %.3g\n\n',BestPerf)
end