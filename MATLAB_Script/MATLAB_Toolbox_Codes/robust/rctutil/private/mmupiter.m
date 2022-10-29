function [lb,Delta,bout,wout] = mmupiter(Min,allBlkData,bstart,wstart,opt)
% function  [a,b,z,w] = mmupiter(M,allBlkData,bstart,wstart,opt)
%   Power method for all forms of skewed-Mixed-Mu.
%   Fixed blocks are delineated within allBlkData.FVidx, which is
%   populated with the command rctutil.FixedVaryIndex

%   Copyright 1986-2020 The MathWorks, Inc.
[nR,nC] = size(Min);
if ~all(isfinite(Min(:)))
   % Protect against frequency response evaluating to Inf/NaN
   lb = Inf;
   Delta = zeros(nC,nR);
   bout = [];  wout = [];
   return
end
   
% Argument check
if nargin<4
   bstart = [];
   wstart = [];
end
if nargin<5
   opt = struct('CNTMAX',[],'diagScale',true);
end

FV = allBlkData.FVidx;
VaryRows = FV.VaryRows;
VaryCols = FV.VaryCols;
fixedBlkIdx = FV.fixedBlkIdx;
FixedRows = FV.FixedRows;
FixedCols = FV.FixedCols;
FixedUnionRealCols = FV.FixedUnionRealCols;
FixedUnionRealRows = FV.FixedUnionRealRows;
VaryComplexCols = FV.VaryComplexCols;
VaryComplexRows = FV.VaryComplexRows;
%blk = allBlkData.simpleblk;
allVary = isempty(fixedBlkIdx);
nVR = numel(VaryRows);
nFR = numel(FixedRows);
nComplexVary = numel(FV.VaryCIdx);
nComplexFixed = numel(FV.FixedCIdx);

problemType = allBlkData.problemType;

if opt.diagScale
   M = osbal(Min,allBlkData);
else
   M = Min;
end

% Iteration options
% XXX, depend on blocks in a more complicated way...
if allBlkData.allcomp.num==0
   % Pure real mu
   CNTMAX = 250;
   stol = 1e-13;
elseif nComplexVary==0
   % There are some complex fixed blocks but varying portion is pure real
   CNTMAX = 250;
   stol = 1e-7;
elseif nComplexFixed==0 && ~allVary
   % There are complex varying blocks and all fixed blocks are real
   CNTMAX = 250;
   stol = 1e-7;
   % Reduces UB/LB gap from 4.8% to 3.2% for SafariShadEtAlEngineEx, 
   % no difference otherwise
%    CNTMAX = 500;
%    stol = 1e-8;
else
   CNTMAX = 150;
   stol = 1e-7;
end
realParmStep = length(M)/4;   % XXX MINOR: FROM RUBLOW.M
if ~isempty(opt.CNTMAX)
   CNTMAX = opt.CNTMAX;
end


% Initialize vectors
[Mr,Mc]=size(M);
if isempty(bstart)
   [~,~,v] = svd(M);
   % Always slightly perturb singular vectors to avoid getting stuck along
   % invariant directions.
   RNG = RandStream('mt19937ar');
   b = v(:,1) + 0.01*complex(RNG.rand(Mc,1)-0.5,RNG.rand(Mc,1)-0.5);
   bstart = b/norm(b);
   if allVary || Mc==1
      wstart = bstart;
   else
      w = v(:,2) + 0.01*complex(RNG.rand(Mc,1)-0.5,RNG.rand(Mc,1)-0.5);
      wstart = w/norm(w);
   end
end
newz = zeros(Mr,1);
newb = zeros(Mc,1);
a = zeros(Mr,1);
z = zeros(Mr,1);
b = bstart;
w = wstart;

% Get indices for real scalars, complex scalars, and complex full blocks
nreal = allBlkData.allreal.num;
realr = allBlkData.allreal.allrows;
realc = allBlkData.allreal.allcols;

nrepc = allBlkData.repcomp.num;
repcr = allBlkData.repcomp.allrows;
repcc = allBlkData.repcomp.allcols;

nfull = allBlkData.full.num;
fullr = allBlkData.full.allrows;
fullc = allBlkData.full.allcols;

% Get masks used to vectorize code
real_mask = allBlkData.masks.real_mask;
repc_mask = allBlkData.masks.repc_mask;
Dfull_maskr = allBlkData.masks.Dfull_maskr;
Dfull_maskc = allBlkData.masks.Dfull_maskc;

% Transpose Masks once (rather than in loops)
real_maskT = real_mask';
repc_maskT = repc_mask';
Dfull_maskrT = Dfull_maskr';
Dfull_maskcT = Dfull_maskc';

if allVary
   MT = M';
   M11 = zeros(0,0);
   M12 = zeros(0,nC);
   M21 = zeros(nR,0);
   M22 = M;
else
   M11 = M(FixedRows,FixedCols);
   M12 = M(FixedRows,VaryCols);
   M21 = M(VaryRows,FixedCols);
   M22 = M(VaryRows,VaryCols);
   M11s = M11';
   M12s = M12';
   M21s = M21';
   M22s = M22';
end
ALLREALSARESCALAR = isequal(real_mask, eye(size(real_mask)));
ALLFULLSARESCALAR = isequal(Dfull_maskc, eye(size(Dfull_maskc))) & ...
   isequal(Dfull_maskr, eye(size(Dfull_maskr)));

% Set up for loop
converged = false;
cnt = 0;
qb = ones(nreal,1);
lb = 0;

ascale = 1;
wscale = 1;
while ~converged && cnt<CNTMAX
   
   cnt = cnt + 1;
   if allVary
      newa = M*b;
      beta1 = norm(newa);
      newa = newa/beta1;
   else
      b1 = b(FixedCols);
      b2 = b(VaryCols);
      M11b = M11*b1;
      M12b = M12*b2;
      M21b = M21*b1;
      M22b = M22*b2;
      
      [~,alpha1,newa] = s4vecp(M11b,M12b,M21b,M22b);
      beta1 = 1/alpha1;
      if isinf(beta1)
         %b(VaryCols) = 0;
         ascale = norm(newa);
         if ascale==0
            ascale = 1;
         end
      else
         ascale = 1;
      end
      newa([FixedRows,VaryRows]) = newa/ascale;
   end
   if beta1 < 100*eps
      % a = newa; % XXX revisit if needed
      break;
   end
   
   if cnt==1
      qb=real(real_mask*( conj(b(realc)).*newa(realr) ));
   end
   
   % real blocks
   if nreal>0
      wreal = w(realc);
      areal = newa(realr);
      breal = b(realc);
      if ALLREALSARESCALAR
         norma = abs(areal);
         normb = abs(breal);
         wa = conj(wreal).*areal;
      else
         norma = sqrt(real_mask*( conj(areal).*areal ));
         normb = sqrt(real_mask*( conj(breal).*breal ));
         wa = real_mask*( conj(wreal).*areal );
      end
      norma = norma+(norma<=100*eps*normb).*(normb+100*eps);
      qz = sign(qb).*normb./norma+realParmStep*real(wa);
      qz = qz./max(abs(qz),1);
      if ALLREALSARESCALAR
         newz(realr) = qz.*wreal;
      else
         newz(realr) = (real_maskT*qz).*wreal;
      end
   end
   
   % repeated complex scalar blocks
   if nrepc>0
      wrepc = w(repcc);
      arepc = newa(repcr);
      wa = repc_mask*( conj(wrepc).*arepc );
      abs_wa = abs(wa);
      wa = (abs_wa>100*eps).*( wa./ max(abs_wa,100*eps) );
      newz(repcr) = (repc_maskT*wa).*wrepc;
   end
   
   % full blocks
   if nfull>0
      wfull = w(fullc);
      afull = newa(fullr);
      if ALLFULLSARESCALAR
         norma = abs(afull);
         normw = abs(wfull);
      else
         norma = sqrt(Dfull_maskr*( conj(afull).*afull ));
         normw = sqrt(Dfull_maskc*( conj(wfull).*wfull ));
      end
      tmp=zeros(nfull,1);
      idx = find(norma>100*eps*normw);
      tmp(idx) = normw(idx)./norma(idx);
      if ALLFULLSARESCALAR
         newz(fullr) = tmp.*afull;
      else
         newz(fullr) = (Dfull_maskrT*tmp).*afull;
      end
   end
   
   % neww = M'*newz;
   if allVary
      neww = MT*newz;
      beta2 = norm(neww);
      neww = neww/beta2;
   else
      z1 = newz(FixedRows);
      z2 = newz(VaryRows);
      M11sz = M11s*z1;
      M21sz = M21s*z2;
      M12sz = M12s*z1;
      M22sz = M22s*z2;
      
      [~,alpha2,neww] = s4vecp(M11sz,M21sz,M12sz,M22sz);
      beta2 = 1/alpha2;
      if isinf(beta2)
         %z(VaryRows) = 0;
         wscale = norm(neww);
         if wscale==0
            wscale = 1;
         end
      else
         wscale = 1;
      end
      neww([FixedCols,VaryCols]) = neww/wscale;
   end
   if beta2 < 100*eps
      break;
   end
   
   % real blocks
   if nreal>0
      wreal = neww(realc);
      areal = newa(realr);
      breal = b(realc);
      if ALLREALSARESCALAR
         norma = abs(areal);
         normb = abs(breal);
         wa = conj(wreal).*areal;
      else
         norma = sqrt(real_mask*( conj(areal).*areal ));
         normb = sqrt(real_mask*( conj(breal).*breal ));
         wa = real_mask*( conj(wreal).*areal );
      end
      norma = norma+(norma<=100*eps*normb).*(normb+100*eps);
      qb = sign(qz).*normb./norma+realParmStep*real(wa);
      qb = qb./max(abs(qb),1);
      if ALLREALSARESCALAR
         newb(realc) = qb.*areal;
      else
         newb(realc) = (real_maskT*qb).*areal;
      end
   end
   
   % repeated complex scalar blocks
   if nrepc>0
      wrepc = neww(repcc);
      arepc = newa(repcr);
      aw = repc_mask*( conj(arepc).*wrepc );
      abs_aw = abs(aw);
      aw = (abs_aw>100*eps).*( aw./ max(abs_aw,100*eps) );
      newb(repcc) = (repc_maskT*aw).*arepc;
   end
   
   % full blocks
   if nfull>0
      wfull = neww(fullc);
      afull = newa(fullr);
      if ALLFULLSARESCALAR
         norma = abs(afull);
         normw = abs(wfull);
      else
         norma = sqrt(Dfull_maskr*( conj(afull).*afull ));
         normw = sqrt(Dfull_maskc*( conj(wfull).*wfull ));
      end
      tmp=zeros(nfull,1);
      idx = find(normw>100*eps*norma);
      tmp(idx) = norma(idx)./normw(idx);
      if ALLFULLSARESCALAR
         newb(fullc) = tmp.*wfull;
      else
         newb(fullc) = (Dfull_maskcT*tmp).*wfull;
      end
   end
   
   newlb = max(beta1,beta2);
   %[newlb lb ( newlb-lb ) cnt]
   if abs( newlb-lb ) < stol || (isinf(newlb) && isinf(lb))
      chng = [newb; newa; newz; neww]-[b; a; z; w];
      if max(abs(chng)) < stol
         converged = true;
      end
      %     else
      %         chng = NaN;
   end
   
   % Update
   lb = newlb;
   a = newa;
   b = newb;
   z = newz;
   w = neww;
end

bout = b;
wout = w;
lb_iter = lb;

% Delta = mkcpert(b,a,allBlkData,'unitary');
Delta = mkcpert(b,a,allBlkData,'dyad');
Delta(realc,realr) =  diag(real_maskT*qb);
Delta(VaryCols,VaryRows) = (1/lb_iter)*Delta(VaryCols,VaryRows);

% FixedBlocks can be <1 but can't be >1--> Don't allow fixScale to be <1.
fixScale = max([1 ascale wscale]);
Delta(FixedCols,FixedRows) = Delta(FixedCols,FixedRows)/fixScale;

switch problemType
   case {'robstab' 'robgain'}
      if lb>100*eps && nComplexVary==0 % Varying portion is pure real
         if strcmp(problemType,'robgain')
            % one fixed block, it is complex, full-block
            DeltaVary = Delta(VaryCols, VaryRows);
            Mloop99 = M11 + M12*(0.9999*DeltaVary)/(eye(nVR)-M22*(0.9999*DeltaVary))*M21;
            Mloop101 = M11 + M12*(1.0001*DeltaVary)/(eye(nVR)-M22*(1.0001*DeltaVary))*M21;
            n99 = norm(Mloop99);
            n101 = norm(Mloop101);
            if n99>=1
               lb = 1/norm(0.9999*DeltaVary);
               [U,S,V] = svd(Mloop99);
               DeltaFixed = (V(:,1)*U(:,1)')/S(1,1);
               Delta(FixedCols, FixedRows) = DeltaFixed;
               Delta(VaryCols, VaryRows) = 0.9999*DeltaVary;
            elseif n101<1
               lb = 0;
            else
               % n99*(1-t) + n101*t = 1
               t = (1-n99)/(n101-n99);
               vscale = 0.9999*(1-t) + 1.0001*t + 1e-5;
               Mloop = M11 + M12*(vscale*DeltaVary)/(eye(nVR)-M22*(vscale*DeltaVary))*M21;
               nrmMloop = norm(Mloop);
               if nrmMloop>=1
                  lb = 1/norm(vscale*DeltaVary);
                  [U,S,V] = svd(Mloop);
                  DeltaFixed = (V(:,1)*U(:,1)')/S(1,1);
                  Delta(FixedCols, FixedRows) = DeltaFixed;
                  Delta(VaryCols, VaryRows) = vscale*DeltaVary;
               else
                  vscale = 1.0001;
                  lb = 1/norm(vscale*DeltaVary);
                  [U,S,V] = svd(Mloop101);
                  DeltaFixed = (V(:,1)*U(:,1)')/S(1,1);
                  Delta(FixedCols, FixedRows) = DeltaFixed;
                  Delta(VaryCols, VaryRows) = vscale*DeltaVary;
               end
            end
         else
            DeltaFixed = Delta(FixedCols, FixedRows);
            Mloop = M22 + M21*DeltaFixed/(eye(nFR)-M11*DeltaFixed)*M12;
            DeltaVary = Delta(VaryCols, VaryRows);
            evl = eig(Mloop*DeltaVary);
            minD1 = min(abs(1-evl));  % XXX what if EVL is 0.9999999 - then fails to produce nonzero Delta
            if minD1>1e-10
               lb = 0;
            end
         end
      elseif lb>100*eps % at least one complex Varying block
         % wrap in all real, and all Fixed complexes
         DeltaFixedUnionReal = Delta(FixedUnionRealCols, FixedUnionRealRows);
         M11 = M(FixedUnionRealRows,FixedUnionRealCols);
         evl = eig(M11*DeltaFixedUnionReal);
         if min(abs(1-evl))<1e-10
            Delta(VaryComplexCols, VaryComplexRows) = 0;
            lb = 1/norm(Delta(VaryCols,VaryRows));
         else
            R11 = numel(FixedUnionRealRows);
            M12 = M(FixedUnionRealRows,VaryComplexCols);
            M21 = M(VaryComplexRows,FixedUnionRealCols);
            M22 = M(VaryComplexRows,VaryComplexCols);
            Mloop = M22 + M21*DeltaFixedUnionReal/(eye(R11)-M11*DeltaFixedUnionReal)*M12;
            DeltaVaryComplex = Delta(VaryComplexCols, VaryComplexRows);
            [~,evl] = eig(DeltaVaryComplex*Mloop);
            [~,idx] = max(abs(diag(evl)));
            DeltaVaryComplex = DeltaVaryComplex/evl(idx,idx);
            Delta = zeros(Mc,Mr);
            Delta(VaryComplexCols, VaryComplexRows) = DeltaVaryComplex;
            Delta(FixedUnionRealCols, FixedUnionRealRows) = DeltaFixedUnionReal;
            lb = 1/norm(Delta(VaryCols,VaryRows));
         end
        elseif nComplexVary>0
            % (3/1/2019, PJS): Lower bound is < 100*eps
            % This can happen due to failed power iteration, e.g.
            %    G = [0 -1; 1 0]; blk = [1 0; 1 0];
            % Power iteration exits early if bstart=wstart = [-1; 0];
            % Use varying complex blocks to construct a lb>0 with:
            %  a) Fixed or Varying Real blocks = 0
            %  b) Fixed complex blocks = 0
            %  c) Varying repeated complex scalar blocks =  eye(n)
            %  d) Varying full complex blocks = ones(n,1)*ones(1,m)/sqrt(n*m)
            blk = allBlkData.simpleblk;
            newloc =allBlkData.newloc;
            sb.azidx = allBlkData.repcomp.rows;
            sb.bwidx = allBlkData.repcomp.cols;            
            fb.azidx = allBlkData.full.rows;
            fb.bwidx = allBlkData.full.cols;
            
            DeltaComplex = zeros(Mc,Mr);
            for i=1:size(blk,1)
                idx = newloc(i);
                if  blk(i,2)==0 && blk(i,1)>0
                    DeltaComplex(sb.bwidx{idx},sb.azidx{idx}) = eye(blk(i,1));
                elseif blk(i,2)>0
                    lvec{i} = ones(blk(i,1),1);
                    rvec{i} = ones(1,blk(i,2));
                    Deltai = lvec{i}* rvec{i} / norm( lvec{i}* rvec{i} );
                    DeltaComplex(fb.bwidx{idx},fb.azidx{idx}) =  Deltai;
                end
            end

            % Fill in only the varying complex blocks of Delta
            Delta = zeros(Mc,Mr);
            Delta(VaryComplexCols, VaryComplexRows) = ...
                DeltaComplex(VaryComplexCols, VaryComplexRows);
            
            evl = eig(Delta*M);
            [lb,idx] = max(abs(evl));
            Delta = Delta/evl(idx);
      end
   case {'wcgain' 'general'}
      DeltaFix = Delta(FixedCols,FixedRows);
      eMDF = eig(M11*DeltaFix);
      if min(abs(eMDF-1))<1e-10
         lb = inf;
         Delta(VaryCols,VaryRows) = 0;
      else
         F11 = numel(FixedRows);
         Mloop = M22 + M21*DeltaFix/(eye(F11)-M11*DeltaFix)*M12;
         if strcmp(problemType,'wcgain')
            [U,S,V] = svd(Mloop);
            lb = S(1,1);
            Delta(VaryCols,VaryRows) = (V(:,1)*U(:,1)')/lb;
         else
            Vblk = allBlkData.simpleblk;
            Vblk(allBlkData.FVidx.fixedBlkIdx,:) = [];
            VblkD = rctutil.mkBlkData(Vblk);
            [lb,Delta(VaryCols,VaryRows)] = mmupiter(Mloop,VblkD);
         end
      end
end


if lb <= 100*eps || isnan(lb)
   % New policy is to return Delta=0 when
   % lb=0.   lb=0 indicates no perturbation causing singularity was found.
   Delta = rctutil.dummyDelta(allBlkData.simpleblk);
   lb = 0;
end

