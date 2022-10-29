function [K,gAch,IterArray] = musyn(P,nY,nU,varargin)
%MUSYN  Robust controller design via mu-synthesis.
%
%   MUSYN implements the D-K or D-G-K iteration approach to mu-synthesis.
%   It combines H-infinity synthesis (K-step) with mu-analysis (D-step)
%   to mimimize the peak mu upper bound, a measure of robust closed-loop
%   performance (see MUSYNPERF). You can use either HINFSYN or HINFSTRUCT
%   to perform the K-step.
%
% HINFSYN-based syntax (unstructured K):
%
%   [K,CLPERF] = MUSYN(P,NMEAS,NCONT) optimizes the robust performance of
%   the uncertain closed-loop system CL=lft(P,K). The uncertain plant P is
%   a USS model, the last NCON inputs of P are the controls U, and the last
%   NMEAS outputs of P are the measurements Y. MUSYN returns the robust
%   controller K and best achieved robust performance CLPERF (see MUSYNPERF).
%   These variables are related by:
%
%      CL = lft(P,K);
%      BND = musynperf(CL);
%      CLPERF = BND.UpperBound;
%
%   [K,CLPERF,INFO] = MUSYN(P,NMEAS,NCONT) also returns a struct array
%   INFO where INFO(j) contains the results of the j-th D-K iteration. The
%   fields of INFO are:
%      K,gamma     Controller and its scaled H-inf performance
%      KInfo       Synthesis data (see HINFSYN)
%      PeakMu      Robust performance of LFT(P,K)
%      DG          D,G scalings from robust performance analysis
%      dr,dc,PSI   Rational fit of D,G data (see reference pages)
%      FitOrder    D,G fit orders
%      PeakMuFit   Scaled H-inf performance with fitted D,G.
%
%   [...] = MUSYN(P,NMEAS,NCONT,KINIT) initializes the D-K iteration with
%   the controller KINIT. Use KINIT=INFO(j).K to restart with the result of
%   the j-th iteration from a previous run.
%
%   [...] = MUSYN(...,OPT) specifies additional options. Use MUSYNOPTIONS
%   to create the option set OPT.
%
% HINFSTRUCT-based syntax (fixed-structure K):
%
%   [CL,CLPERF] = MUSYN(CL0) optimizes the robust performance by tuning
%   the free parameters in the tunable closed-loop model CL0 (see GENSS).
%   MUSYN returns the tuned version CL of CL0 and the best achieved robust
%   performance CLPERF.
%
%   [CL,CLPERF,INFO,RUNS] = MUSYN(CL0) also returns details about each D-K
%   iteration (same as above except that K field now contains tuned block
%   values) and each run when using the RandomStart option.
%
%   [...] = MUSYN(CL0,BLOCKVALS) initializes the D-K iteration with the
%   tuned block values in BLOCKVALS. Use BLOCKVALS=INFO(j).K to restart
%   with the result of the j-th iteration from a previous run.
%
%   MUSYN(CL0,CL) is the same as MUSYN(CL0,CL.Blocks) and restarts with the
%   output CL of a previous run.
%
%   [...] = MUSYN(...,OPT) specifies additional options. Use MUSYNOPTIONS
%   to create the option set OPT.
%
%   See also MUSYNOPTIONS, MUSYNPERF, USS, GENSS, HINFSYN, HINFSTRUCT, WCGAIN, ROBGAIN.

%   Copyright 2003-2019 The MathWorks, Inc.
narginchk(3,5);

% Validate P
[nPr,nPc,Nmod] = size(P);
nE = nPr-nY;
nD = nPc-nU;
if Nmod>1
   error(message('Robust:design:musyn9'))
elseif nE<=0 || nD<=0
   error(message('Robust:design:musyn10',nU+1,nY+1))
elseif ~all(structfun(@(x) isa(x,'ureal') || isa(x,'ucomplex') || ...
      isa(x,'ultidyn') || isa(x,'umargin'),P.Uncertainty))
   error(message('Robust:design:musyn11'))
end
[P,BLK,BKnames] = musynData(P,nY,nU);
if ~isreal(P)
   error(message('Robust:design:musyn16'))
end

% Process varargin
opt = musynOptions;
Kinit = [];
for ct=1:numel(varargin)
   if isnumeric(varargin{ct}) || isa(varargin{ct},'InputOutputModel')
      Kinit = varargin{ct};
      % Validate
      try
         Kinit = ss(Kinit,'exp');
      catch
         error(message('Robust:design:musyn7'))
      end
      if ~isequal(size(Kinit),[nU nY])
         error(message('Robust:design:musyn8',nY,nU))
      elseif ~isstable(lft(P,Kinit))
         warning(message('Robust:design:musyn12'))
         Kinit = [];
      end
   elseif isa(varargin{ct},'rctoptions.musyn')
      opt = varargin{ct};
   else
      error(message('Robust:design:musyn6',ct+3))
   end
end

% Align HINFSYN verbosity with main Display option
if strcmp(opt.Display,'full')
   opt.hinfsynOPT.Display = 'on';
else
   opt.hinfsynOPT.Display = 'off';
end

% Call MUSYN Engine
% Note: P is of class @ss
Ts = getTs(P);
if Ts<0
   % musynEngine requires P.Ts>=0
   P.Ts = 1;
end
[K,gAch,IterArray] = musynEngine(P,BLK,BKnames,nY,nU,Kinit,opt);
if Ts<0
   K.Ts = -1;
   for ct=1:numel(IterArray)
      IterArray(ct).K.Ts = -1;
   end
end