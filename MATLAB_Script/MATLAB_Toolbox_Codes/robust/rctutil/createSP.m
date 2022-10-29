function Data = createSP(M,blk,bidx,opts,SkipComputationFlag,WorstCurrentLB)
% Creates/Initializes a single instance of a matrix, worst-case norm problem.
% Upper and lower bounds are computed, and all information about the
% problem is returned in Data.   Data is a struct, with fields
%   Data.blk, double array describing uncertainty block structure
%   Data.bidx, struct of various row/column indices and other block info
%   Data.nReal, number of real parameters
%   Data.ub = overall upper bound for this problem (derived)
%   Data.lb = overall lower bound for this problem (derived)
%   Data.cubeidx, struct of indices for storage/retrieval into CUBELIST
%   Data.cubelist = [left_ends right_ends LOWER UPPER ACTIVE DGbetaIOScalings]
%      The bounds in cubelist are scaled bounds. Divide by Data.IOScalings
%      to get the actual bounds.
%   Data.Scalednomgain = 0;
%   Data.ScaledLb = cubelist(1,LOWER);
%   Data.ScaledUb = cubelist(1,UPPER);
%   Data.SkipComputationFlag = true;
%   Data.agap = 0;
%   Data.mgap
%   Data.IOScalings = 1;
%   Data.AllWsreal = [];
%   Data.AllWrepreal = [];
%   Data.AllWlvec = {[]};
%   Data.AllWrvec = {[]};
%   Data.AllWrepcomp = [];

%   Copyright 2011-2012 The MathWorks, Inc.
nin = nargin;
if nin<5
   SkipComputationFlag = false;
end
if nin<6
   WorstCurrentLB = 0;
end
% Initialize output
Data = struct('blk',blk,'bidx',bidx,'nReal',[],'cubeidx',[],'M',[],...
   'IOScalings',[],'AllWsreal',[],'AllWrepreal',[],'AllWlvec',[],...
   'AllWrvec',[],'AllWrepcomp',[],'cubelist',[],'Scalednomgain',[],...
   'ScaledLb',[],'ScaledUb',[],'SkipComputationFlag',[],'agap',[],'mgap',[]);

% Number of real parameters
nblk = bidx.sreal.num + bidx.repreal.num;
Data.nReal = nblk;
% Total number of elements in DG-scalings
[ftdidx,varcnt] = uball(M,bidx,[],[],1);

ALLCOR = 1:(2*nblk);
LEFTCOR = 1:nblk;
RIGHTCOR = (nblk+1):(2*nblk);
LOWER = 2*nblk + 1;
UPPER = 2*nblk + 2;
ACTIVE = 2*nblk + 3;
DGbeta = 1 + varcnt; % beta
SCALINGS = (2*nblk+4):(2*nblk+3+DGbeta); % ACTIVE+(1:DGbeta);
cols = 2*nblk + 3 + DGbeta;
SCALARS = 1:bidx.sreal.num;
REPS = bidx.sreal.num+1:nblk;

cubeidx.ALLCOR = ALLCOR;
cubeidx.LEFTCOR = LEFTCOR;
cubeidx.RIGHTCOR = RIGHTCOR;
cubeidx.LOWER = LOWER;
cubeidx.UPPER = UPPER;
cubeidx.ACTIVE = ACTIVE;
cubeidx.DGbeta = DGbeta;
cubeidx.SCALINGS = SCALINGS;
cubeidx.cols = cols;
cubeidx.SCALARS = SCALARS;
cubeidx.REPS = REPS;
Data.cubeidx = cubeidx;


row = bidx.rdimm+1:size(M,1);
col = bidx.cdimm+1:size(M,2);

if SkipComputationFlag
   % Fill outputs but don't compute any bounds
   Data.M = M;
   Data.IOScalings = 1;
   Data.AllWlvec = {[]};
   Data.AllWrvec = {[]};
   
   rowone = ones(1,nblk);
   lowerbnd = 0;
   upperbnd = Inf;
   dgbvec = zeros(1,DGbeta);
   cubelist(1,[ALLCOR LOWER UPPER ACTIVE SCALINGS]) = ...
      [-rowone rowone lowerbnd upperbnd 1 dgbvec];
   Data.cubelist = cubelist;
   Data.Scalednomgain = 0;

   Data.ScaledLb = cubelist(1,LOWER);
   Data.ScaledUb = cubelist(1,UPPER);
   
   Data.SkipComputationFlag = true;
   
   Data.agap = 0;
   Data.mgap = 1;
   return
end

if nblk~=0
   rnorm = norm(M(row,1:bidx.cdimm));
   cnorm = norm(M(1:bidx.rdimm,col));
   if rnorm<0.01
      rnorm = 0.01;
   elseif rnorm>100
      rnorm = 100;
   end
   if cnorm<0.01
      cnorm = 0.01;
   elseif cnorm>100
      cnorm = 100;
   end
   Sr = 1/rnorm;
   Sc = 1/cnorm;
else
   nrM = norm(M);
   Sr = 1/sqrt(nrM);
   Sc = 1/sqrt(nrM);
end

M(row,:) = Sr*M(row,:);
M(:,col) = M(:,col)*Sc;
Data.M = M;
Data.IOScalings = Sr*Sc;
WCLB = WorstCurrentLB*Data.IOScalings;

predim = 5;%50;
CNTMAX = 4;
RNG = RandStream('mt19937ar');

% create BBLists, and compute first set of bounds
NTIMES = opts.NSearch;
MAXCNT = opts.MaxCnt;
MBad = opts.RelMax;
SABad = Data.IOScalings*opts.AbsMax;
rowone = ones(1,nblk);

cubelist = zeros(predim,cols);
%Mcube = rcnrs(M,bidx,-ones(1,nblk),ones(1,nblk));
% [lowerbnd,pertsreal,pertrepreal,pertLvec,pertRvec,pertrepcomp,Scalednomgain] = ...
%    wclowc(Mcube,bidx,NTIMES,MAXCNT,MBad,SABad);
[lowerbnd,pertsreal,pertrepreal,pertLvec,pertRvec,pertrepcomp,Scalednomgain] = ...
   wclowc(M,bidx,NTIMES,MAXCNT,MBad,SABad,RNG);
Data.AllWsreal = reshape(pertsreal,[1 numel(pertsreal)]);
Data.AllWrepreal = reshape(pertrepreal,[1 numel(pertrepreal)]);
Data.AllWlvec = pertLvec;
Data.AllWrvec = pertRvec;
Data.AllWrepcomp = pertrepcomp;

if strcmp(opts.LowerBoundOnly,'off') && ( lowerbnd <= MBad*Scalednomgain+SABad )
   xfeas = [];
   cnt = 1;
   beta = lowerbnd;
   % lbstr = [', LB: ' num2str(beta)];
   % Jan 13, 2011.
   % We are still debating the merits of using cheap mu-like scalings to
   % establish an initial upper bound, versus the LMI function call.
   % On small problems, it's definitely faster
   % to skip the MUSSV/CHEAPBNDS call.  On large problems, with FreqPtWise=0,
   % it's less clear.  The LMI-call could be huge, and there's no reason to
   % do it for every frequency point (even if it has to be done for a few).
   % Of course, the real costly steps are B&B.
   % Tentatively, we will skip the cheap scaling, but the code is left in
   % for future modification.
   % UseCheap = bidx.sreal.num+bidx.repreal.num>0;
   % April 2, 2011.
   % Base decision to use cheap (repeated scaled MU calcs) on the total
   % number of variables in the D/G scalings.  The variable VARCNT,
   % returned from a "setup" call to UBALL above contains this.  For now,
   % we set the limit at 80.
   UseCheap = varcnt>80;
   % disp(['useLMI= ' int2str(~UseCheap) ', Var#=' int2str(varcnt)])
   if UseCheap
      while isempty(xfeas) && cnt<=CNTMAX
         if cnt==1
            low = lowerbnd;
            if 0.995*WCLB>low
               beta = 0.995*WCLB;
            else
               beta = (1.25*low+0.0001);
            end
         else
            low = beta;
            beta = (1.25*low+0.0001);
         end
         [xfeas,ubout] = LOCALcheapbnds(M,blk,bidx,low,beta,ftdidx,WCLB);
         cnt = cnt + 1;
      end
      if isempty(xfeas)
         upperbnd = inf;
         dgbvec = zeros(1,DGbeta);
      else
         upperbnd = ubout;
         dgbvec = xfeas(:)';
      end
      cubelist(1,[ALLCOR LOWER UPPER ACTIVE SCALINGS]) = ...
         [-rowone rowone lowerbnd upperbnd 1 dgbvec];
   else
      % [upper1,di1,dbs1,gzr1,gzl1,psdami1,dgb1] = uball(M,bidx);
      % TODO: enhance UBALL to only check feasibility at a given bound
      [upper1,~,~,~,~,~,dgb1] = uball(M,bidx);
      if isinf(upper1)
         upperbnd = upper1;
         cubelist(1,[ALLCOR LOWER UPPER ACTIVE]) = ...
            [-rowone rowone lowerbnd upperbnd 1];
      else
         upperbnd = upper1;
         cubelist(1,[ALLCOR LOWER UPPER ACTIVE SCALINGS]) = ...
            [-rowone rowone lowerbnd upperbnd 1 dgb1(:)'];
      end
   end
else
   upperbnd = inf;
   dgbvec = zeros(1,DGbeta);
   cubelist(1,[ALLCOR LOWER UPPER ACTIVE SCALINGS]) = ...
      [-rowone rowone lowerbnd upperbnd 1 dgbvec];
end
Data.cubelist = cubelist;
Data.Scalednomgain = Scalednomgain;

Data.ScaledLb = cubelist(1,LOWER);
Data.ScaledUb = cubelist(1,UPPER);

Data.SkipComputationFlag = ( lowerbnd > MBad*Scalednomgain+SABad );

% Compute an estimate of the "gap" that cannot be reduced through B&B on
% the real uncertainties.  This "gap" is due to the complex uncertainties.
% and is estimated by computing the gap when the real uncertainties are set
% to nominal.
if all( blk(:,3) > 0 )
   % Pure complex block
   gap = Data.ScaledUb - Data.ScaledLb;
   Data.agap = gap;
   Data.mgap = gap/Data.ScaledUb;
elseif all( blk(:,3) < 0 )
   % Pure real block
   Data.agap = 0;
   Data.mgap = 1;
else
   % Mixed block: Solve for bounds with reals at nominal
   ridx =[bidx.repreal.rows{:}, bidx.sreal.rows{:}];
   cidx =[bidx.repreal.cols{:}, bidx.sreal.cols{:}];
   cM = M;
   cM(ridx,:) = [];
   cM(:,cidx) = [];
   
   cblk = blk;
   cblk(blk(:,3) <0,:) = [];
   cbidx = blk2idx(cblk);
   
   clb = wclowc(cM,cbidx,NTIMES,MAXCNT,MBad,SABad,RNG);
   cub = uball(cM,cbidx);
   
   Data.agap = (cub-clb);
   Data.mgap = Data.agap/cub;
end


%-------------------------------------------------------
% LOCAL
function [xfeas,ubout] = LOCALcheapbnds(m,blk,bidx,lbnd,beta,ftdidx,WCLB)
% Use in conjunction with UBALL, where the ordering of the variables
% is as done below.  typically, BETA should be a bound that easily
% passes as a worst-case performance bound.

if isinf(lbnd)
   xfeas = [];
   ubout = inf;
   return;
end

% if WCLB>lbnd
%    beta = WCLB;
% end
opt = 'UZ';

mu3blk = [];
nblk = size(blk,1);
for i=1:nblk
   if blk(i,1)==1 && blk(i,2)==1
      % repeated real or complex scalar
      mu3blk = [mu3blk;blk(i,3) 0]; %#ok<*AGROW>
   elseif blk(i,3)>0
      % full block, make it not repeated
      mu3blk = [mu3blk;blk(i,3)*blk(i,[1 2])];
   end
end
DGbeta = max([ftdidx.grealidx ftdidx.gimagidx ftdidx.drealidx ftdidx.dimagidx]) + 1;

%dims = ndims(m);
szm = size(m);
ny = szm(1);
%nu = szm(2);
ne = szm(1) - bidx.rdimm;
nd = szm(2) - bidx.cdimm;
mublk = [mu3blk;nd ne];
cnt = 0;
fnd = 0;
CNTMAX = 5;
lower = lbnd;
upper = 2*beta - lower;
go = 1;
xydata = [];

m11 = m(1:bidx.rdimm,1:bidx.cdimm);
optnowarning = opt(opt=='d');
bnds11 = mussv(m11,mu3blk,optnowarning);
if bnds11(1) < 1
   % OK
else
   % bound will never work
   if bnds11(2)>=1
      % really have a problem
      % XXXMAJOR - will get INF later
      xfeas = [];
      ubout = inf;
      return;
      
   end
end

while cnt<CNTMAX && go==1
   % Simple bisection
   % CNTMAX needs to be at least 4 for this to activate
   % betaval = LOCALestibeta(xydata,lbnd,beta);
   betaval = (lower+upper)/2;
   fac = 1/sqrt(betaval);
   
   sclm = m;
   sclm(bidx.rdimm+1:bidx.rdimm+ne,:) = fac*sclm(bidx.rdimm+1:bidx.rdimm+ne,:);
   sclm(:,bidx.cdimm+1:bidx.cdimm+nd) = fac*sclm(:,bidx.cdimm+1:bidx.cdimm+nd);
   
   [bnds,info] = mussv(sclm,mublk,opt);
   xydata = [xydata;betaval bnds(1)];
   
   if bnds(1)<1
      fnd = 1;
      gbnds = bnds;
      gdvec = info.dvec;
      ggvec = info.gvec;
      %gfac = fac;
      gbetaval = betaval;
      upper = betaval;
      if bnds(1)>0.9995 || betaval<=WCLB 
         go = 0;
      end
   else
      if cnt==0
         xfeas = [];
         ubout = inf;
         go = 0;
      end
      lower = betaval;
   end
   cnt = cnt + 1;
end

if fnd ==1
   ubout = gbetaval;
   xfeas = zeros(DGbeta,1);
   [Dr,~,Grc,~] = ynftdam2(gdvec,ggvec,mublk,gbnds(1));
   %tmp = sclm'*Dr*sclm - (1.0000001*bnds(1))^2*Dc + sqrt(-1)*(Gcr*sclm-sclm'*Grc);
   %eig(tmp)
   % Dr is NY x NY, Dc is NU x NU, Gcr is NU x NY, Grc is NY x NU
   alpha = Dr(ny,ny);
   fix = gbetaval/alpha;
   xfeas(ftdidx.grealidx) = fix*real(Grc(ftdidx.grealselect));
   xfeas(ftdidx.gimagidx) = fix*imag(Grc(ftdidx.gimagselect));
   xfeas(ftdidx.drealidx) = fix*real(Dr(ftdidx.drealselect));
   xfeas(ftdidx.dimagidx) = fix*imag(Dr(ftdidx.dimagselect));
   xfeas(end) = gbetaval^2;
   %Dr = fix*Dr;
   %Grc = fix*Grc;
   %eig(m'*DD*m - DDD + sqrt(-1)*(Grc*m-m'*Grc))
end

function [bguess] = LOCALestibeta(bfdata,blow,bhigh)

szd = size(bfdata);
targetvalue = .999999;

if szd(1)==2
   mI = [bfdata(:,1) [1;1]]\bfdata(:,2);
   bguess = max([eps (targetvalue-mI(2))/mI(1)]);
else
   [~,idx] = sort(bfdata(:,2));
   b = bfdata(idx,1);
   f = bfdata(idx,2);
   gt = find(f>1);
   lt = find(f<1);
   if isempty(gt)
      newdata = [b(lt(end-1:end)) f(lt(end-1:end))];
      bguess = LOCALestibeta(newdata,blow,bhigh);
   else      
      newdata = [b(lt(end)) f(lt(end));b(gt(1)) f(gt(1))];
      bguess = LOCALestibeta(newdata,blow,bhigh);
   end
end
