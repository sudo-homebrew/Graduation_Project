function [CL,gAch,IterArray,RUNS] = musyn(CL0,varargin)
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
narginchk(1,3);

% Validate CL0
[nE,nD,Nmod] = size(CL0);
if Nmod>1
    error(message('Robust:design:musyn9'))
elseif nE==0 || nD==0
    error(message('Robust:design:musyn13'))
elseif ~all(structfun(@(x) ~isUncertain(x) || isa(x,'ureal') || ...
        isa(x,'ucomplex') || isa(x,'ultidyn') || isa(x,'umargin'),CL0.Blocks))
    error(message('Robust:design:musyn11'))
end
[P0,BLK,BKnames] = musynData(CL0,0,0);
if ~isreal(P0)
    error(message('Robust:design:musyn16'))
end

% Process varargin
opt = musynOptions;
Kinit = [];
for ct=1:numel(varargin)
    if isstruct(varargin{ct}) && isscalar(varargin{ct})
        Kinit = varargin{ct};
    elseif isa(varargin{ct},'genss')
        Kinit = varargin{ct}.Blocks;
    elseif isa(varargin{ct},'rctoptions.musyn')
        opt = varargin{ct};
    else
        error(message('Robust:design:musyn6',ct+1))
    end
end

% Apply KINIT settings to tunable model
if ~isempty(Kinit)
    % Eliminate fields unrelated to tuned blocks
    Blocks = CL0.Blocks;
    F = fieldnames(Blocks);
    TBN = F(structfun(@isParametric,Blocks)); % tuned block names
    Kinit = rmfield(Kinit,setdiff(fieldnames(Kinit),TBN));
    try
        Pinit = setBlockValue(P0,Kinit);
    catch
        error(message('Robust:design:musyn14'))
    end
    if isstable(Pinit)
        P0 = Pinit;
    else
        warning(message('Robust:design:musyn12'))
        Kinit = [];
    end
end

% Finalize Options
NRS = opt.RandomStart;
UseParallel = opt.UseParallel && NRS>1;
UseParallel = ltipack.util.checkParallel(UseParallel);
if UseParallel
    % make sure the pool is not a ThreadPool (not currently supported)
    if isa(gcp,"parallel.ThreadPool")
        error(message("Robust:design:musyn17"));
    end
    % Disable display or DK iteration results will come in intertwined
    opt.Display = 'off';
end
opt.UseParallel = UseParallel;
% Align HINFSTRUCT display with main Display option
if strcmp(opt.Display,'full')
    opt.hinfstructOPT.Display = 'iter';
else
    opt.hinfstructOPT.Display = 'off';
end
opt.RandomStart = 0;  % single run insoide D-K iterations

% Perform synthesis
if NRS==0
    % Single run
    % Note: P0 is of class @genss and P0.Ts>=0 (can't create tunable blocks
    % with Ts=-1)
    [K,gAch,IterArray] = musynEngine(P0,BLK,BKnames,[],[],Kinit,opt);
    CL = setBlockValue(CL0,K);
    RUNS = struct('K',K,'muPerf',gAch,'Info',IterArray);
else
    % First run starts from current value of P0, subsequent runs start from
    % randomly generated block values
    TB = cell(NRS+1,1);  INFO = cell(NRS+1,1);  gam = zeros(NRS+1,1);
    Kstart = randomStart(P0,NRS);
    if UseParallel
        % Run NRS+1 synthesis
        for j=NRS+1:-1:1
            if j>1
                Pj = setBlockValue(P0,Kstart(j-1));
                PF(j) = parfeval(@musynEngine,3,Pj,BLK,BKnames,[],[],[],opt);
            else
                PF(j) = parfeval(@musynEngine,3,P0,BLK,BKnames,[],[],Kinit,opt);
            end
        end
        % Fetch results
        for j=1:NRS+1
            [cIdx,Y1,Y2,Y3] = fetchNext(PF);
            TB{cIdx} = Y1;  gam(cIdx) = Y2;  INFO{cIdx} = Y3;
        end
    else
        ANYDISP = ~strcmp(opt.Display,'off');
        if ANYDISP
            fprintf('\n=== Synthesis 1 of %d ============================================\n',NRS+1)
        end
        [TB{1},gam(1),INFO{1}] = musynEngine(P0,BLK,BKnames,[],[],Kinit,opt);
        for j=1:NRS
            if ANYDISP
                fprintf('\n=== Synthesis %d of %d ============================================\n',j+1,NRS+1)
            end
            Pj = setBlockValue(P0,Kstart(j));
            [TB{j+1},gam(j+1),INFO{j+1}] = musynEngine(Pj,BLK,BKnames,[],[],[],opt);
        end
    end
    % Return best result
    RUNS = struct('K',TB,'muPerf',num2cell(gam),'Info',INFO);
    [gAch,ibest] = min(gam);
    CL = setBlockValue(CL0,TB{ibest});
    IterArray = INFO{ibest};
end

