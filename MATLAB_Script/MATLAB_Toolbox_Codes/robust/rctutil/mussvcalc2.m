function [bnds,info] = mussvcalc2(matin,blk,opt,fixedBlkIdx,SETUP)
% Helper function for MUSSV.

%   Copyright 2010-2012 The MathWorks, Inc.

% Issues
%  1) gflag is not used in mussvcalc.  It can be deleted
%     from the input argument list.
%  2) New option:
%       'g6'  Attempt lower bound based on wcgain multiple times
%              (in this case 10+6*10 times), larger number typically
%              gives better lower bound. This alternative lower bound
%              code is run in addition to the power iteration.
%     Note: I modified goptvl.m so that the number following any option
%     can be more than one digit. Also, this new lower bound code
%     reverts to the standard power iteration for purely complex
%     block structures and hence is not run in this case.
%  3) Use mdata input argument to set options for mu upper LMI bound.
%     This is undocumented and only for internal function calls.


%--------------------------------------------------
% Options Set-up
%--------------------------------------------------
nin = nargin;
nout = nargout;
if nin<3
   opt='';
end
if nin<4
   fixedBlkIdx = zeros(0,1);
end
if nin<5
   SETUP = [];
end

if isempty(opt)
   opt = '';
end

if nout==2
   info = struct('bnds',[],'dvec',[],'pvec',[],'gvec',[],'sens',[],'blk',[]);
end

% Map old options into new options
opt(opt=='C') = 'a';
opt(opt=='r') = 'i';
opt(opt=='R') = 'm';
% Strip off 'c' and 'w'
opt(opt=='c') = [];
opt(opt=='w') = [];
ostruc.displaywarnings = any(opt=='d');
ostruc.fastupper = any(opt=='f');
ostruc.bestupper = any(opt=='a');
ostruc.bestuppernoscaling = any(opt=='n');
ostruc.initialize = any(opt=='i');
ostruc.multiplelower = any(opt=='m');
if ostruc.multiplelower
   ostruc.multiplelowertimes = goptvl(opt,'m',1,1);
else
   ostruc.multiplelowertimes = 0;
end
ostruc.silent = any(opt=='s');
ostruc.decreaselower = any(opt=='x');
ostruc.decreaselowerto2 = any(opt=='U');
ostruc.increaselower = any(opt=='t');
%ostruc.gui = any(opt=='g');
ostruc.backup = any(opt=='b');
ostruc.runmuwcglb = any(opt=='g');
ostruc.runspi = any(opt=='p') || ostruc.multiplelower ||...
    ostruc.decreaselower || ostruc.decreaselowerto2 || ostruc.increaselower;
ostruc.ForceOnlyGradientLikeUB = any(opt=='G');
if ~ostruc.displaywarnings
   hw = ctrlMsgUtils.SuspendWarnings; %#ok<NASGU>
end

% Set max # of iterations in lower bound
if ostruc.decreaselowerto2
   mulbopt.CNTMAX = 2;
elseif ostruc.decreaselower
   mulbopt.CNTMAX = 50;
else
   % XXX mussvcalc uses 200 iterations
   mulbopt.CNTMAX = 100;
end
mulbopt.diagScale = false;

%--------------------------------------------------
% Get matrix dimensions and reshape data into 3-D
%--------------------------------------------------
if isa(matin,'double')
   szm = size(matin);
   if length(szm)==2
      ostruc.silent = 1;
   end
   exd = szm(3:end);
   npts = prod(exd);
   % make N-D into 3-D
   mat = reshape(matin,[szm(1) szm(2) npts]);
elseif isa(matin,'frd')
   szm = size(matin.ResponseData);
   exd = szm(3:end);
   npts = prod(exd);
   Ts = matin.Ts;
   % make N-D into 3-D, also absorb any delays
   mat = reshape(freqresp(matin,matin.Frequency,matin.FrequencyUnit),[szm(1) szm(2) npts]);
else
   error(message('Robust:analysis:Mussvcalc21'))
end
Mr = szm(1);
Mc = szm(2);

if isempty(mat) && isempty(blk)
   bnds = zeros([1,2,exd]);
   sens = zeros([1,0,exd]);
   rowd = zeros([1,0,exd]);
   rowp = zeros([1,0,exd]);
   rowg = zeros([1,0,exd]);
   
   % Store Outputs
   if isa(matin,'frd')
      bnds = frd(bnds,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
      sens = frd(sens,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
      rowd = frd(rowd,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
      rowp = frd(rowp,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
      rowg = frd(rowg,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
   end
   
   info.dvec = rowd;
   info.pvec = rowp;
   info.gvec = rowg;
   info.sens = sens;
   info.blk = blk;
   info.bnds = bnds;
   
   return;
end

%--------------------------------------------------
% Check blk structure and get block indices
%--------------------------------------------------

% blk must be Nx2 matrix of integers
if size(blk,2)~=2 || any(floor(blk(:))~=ceil(blk(:)))
   ctrlMsgUtils.error('Robust:analysis:Mussvcalc22')
end

nblk = size(blk,1);
% Automatically use LMI code if:
%  1) user specifies 'a' option (code above sets ostruc.bestupper = true)
%  OR
%  2) problem contains fixed blocks 
%  OR
%  3) number of variables is less than nDVLimit, AND 
%   user has not explicitly specified fast ('f') or gradient ('G') options
nDV = rctutil.nDVmussv(blk);
nDVLimit = 60;
if ~isempty(fixedBlkIdx) || ...
    (nDV<nDVLimit && ~ostruc.fastupper && ~ostruc.ForceOnlyGradientLikeUB)
   ostruc.bestupper = true;
end


% Rough count of decision variable.  Automatically use LMI code ('a') if
% number of variables is less than nDVLimit, and user has not
% explicitly specified 'fastoption' ('f').  When we implement mussvOptions,
% we will restore a manner in which user can force the original default
% (not LMI) behavior.
nDV = rctutil.nDVmussv(blk);
nDVLimit = 60;
if ~ostruc.fastupper && nDV<nDVLimit && ~ostruc.ForceOnlyGradientLikeUB
   ostruc.bestupper = true;
end

[index,blk] = rctutil.mkBlkData(blk,fixedBlkIdx);
if ostruc.bestupper
   dtol = 1e-4;
   DGLMI =  rctutil.DGLMIsys(index,fixedBlkIdx);
   DGLMI.dtol = dtol;
   DGLMI.LMIopt = [1e-2 0 1/dtol 5 1];
   index.allDGlmi = DGLMI;
end
index.allreal.realidx = rctutil.mkBlkData(index.allreal.realblk, []);
index.allcomp.compidx = rctutil.mkBlkData(index.allcomp.compblk, []);

% Update settings for power iteration & muwcgainlb
if index.allcomp.num==0
    % Set lower bound opt structure for pure real problems
    % Use to increase # of power iters if matrix data is real
    mulbopt_r = mulbopt;
    mulbopt_r.CNTMAX = 300;
    if ~ostruc.runspi
        % force GBLB when all real blocks and no option specified for LB
        ostruc.runmuwcglb = true;
    end
else
    % Run power iteration for complex/mixed mu problems
    ostruc.runspi = true;
end

% Get indices from blk2idx plus a few other indices/masks needed for mu
% Also grab indices for real blocks only and for complex blocks only
ridxm = index.ridxm; % col2 of tblkp
cidxm = index.cidxm; % col1 of tblkp

% Check compatibility of block and matrix dimensions
if (index.rdimm~=Mr || index.cdimm~=Mc) && isempty(SETUP)
   ctrlMsgUtils.error('Robust:analysis:Mussvcalc2IncomBlk')
end


% Revisit this logic if/when we implement FV version of muwcglb.
hasFixedBlock = ~isempty(fixedBlkIdx);
if hasFixedBlock
   % At this point, No gain-based for FV problems
   ostruc.runmuwcglb = false;
   ostruc.runspi = true;
   indexWithoutFixedBlock = rctutil.mkBlkData(blk,[]);
end


      
% Set # of tries for muwcglb
if ostruc.runmuwcglb
   % If the 'g' option is not found in OPT, it means that GBLB has been
   % forced since there are no complex blocks.  In that case, chose the
   % number of iterations based on the number of blocks.  This is not a
   % theorem, but simply forces more iterations with the number of blocks,
   % limiting it to some value.
   notfoundval = min([max([1 ceil(nblk/4)]) 5]);
   GBLBoptval = goptvl(opt,'g',1,notfoundval);
   wcglbopt.Ntry = 10+10*GBLBoptval;
end

if index.allcomp.num==0 && ostruc.displaywarnings
   warning(message('Robust:analysis:Mussvcalc23'));
end

%--------------------------------------------------
% Initialize outputs:
%  Scalings/Delta stored as row vectors in
%  the same format as the old mussv code
%--------------------------------------------------
wp=sum(sum(index.masks.DeltaFull_mask))+index.allrep.num;
wd=sum(sum(index.masks.Drep_mask))+index.full.num;
wg=sum(index.allreal.repeated);
bnds = zeros([1 2 exd]);
rowp = zeros([1 wp exd]);
rowd = zeros([1 wd exd]);
rowg = zeros([1 wg exd]);
sens = zeros([1 nblk exd]);
if ~isempty(SETUP)
   if isa(matin,'frd')
      % July 28, 2010.  FrequencyUnit instead of Units, also will
      % we have to specify TimeUnit to make them look the same
      bnds = frd(bnds,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
      sens = frd(sens,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
      rowd = frd(rowd,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
      rowp = frd(rowp,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
      rowg = frd(rowg,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
   end
   info.dvec = rowd;
   info.pvec = rowp;
   info.gvec = rowg;
   info.sens = sens;
   info.blk = blk;
   info.bnds = bnds;
   return
end

%--------------------------------------------------
% Start pointwise computation of bounds
%--------------------------------------------------
if ~ostruc.silent
   strl = 'Points completed: ';
   fprintf(strl)
end
RNG = RandStream('twister','Seed',0);
% density = mdata(2);
% pcnt = 1;
lptxt = '';
deltareal = [];
for ii = 1:npts
   M = mat(:,:,ii);
   %     Dr = eye(Mr);
   %     Dc = eye(Mc);
   %     Gcr = zeros(Mc,Mr);
   
   %--------------------------------------------------
   % Pre-scale: Osborne's
   %--------------------------------------------------
   if ~ostruc.bestuppernoscaling
      [dMd,Dr_os,Dci_os] = osbal(M,index);
      Dc_os = inv(Dci_os);
   else
      dMd = M;
      Dr_os = eye(Mr);
      Dc_os = eye(Mc);
   end
   
   %--------------------------------------------------
   % Scale Data and Compute Initial Upper Bound
   %--------------------------------------------------
   %    scale = max(max(abs(dMd)))/5;
   %    if scale<10*eps || hasFixedBlock
   %       scale = 1;  % should only scale the VARYing channels
   %    else
   %       dMd = dMd/scale;
   %    end
   %    [~,svDMD,vv]=svd(dMd);
   
   scale = max(max(abs(dMd)))/5;
   if scale<100*eps
      scale = 1;  % should only scale the VARYing channels
   end
   dMd(index.FVidx.VaryRows,:)  = dMd(index.FVidx.VaryRows,:)/sqrt(scale);
   dMd(:,index.FVidx.VaryCols)  = dMd(:,index.FVidx.VaryCols)/sqrt(scale);
   [~,svDMD,vv]=svd(dMd);
   
   if hasFixedBlock
      % upper bound must be derived from generalized eval problem
      FixedCols = index.FVidx.FixedCols;
      VaryCols = index.FVidx.VaryCols;
      DcF = zeros(Mc);
      DcV = zeros(Mc);
      DcF(FixedCols,FixedCols) = eye(numel(FixedCols));
      DcV(VaryCols,VaryCols) = eye(numel(VaryCols));
      A = dMd'*dMd - DcF;
      B = DcV;
      evals = eig(A, B);
      finiteIdx = ~isinf(evals);
      ubsq = 1.0001*max(real(evals(finiteIdx)));
      if max(real(eig(A-ubsq*B)))<=0
         ub = sqrt(ubsq);
      else
         ub = inf;
      end
   else
      ub = svDMD(1,1);
   end
   Dr = eye(Mr);
   Dc = eye(Mc);
   Gcr = zeros(Mc,Mr);
   % UB represents an upper-bound on mu(dMd).  Note mu(M)=mu(dMd), so upper
   % bound on mu(M) as well.  This is for all FV cases.  Scalings that
   % certify this bound (if any, ie., if UB is finite) are D=I, G=0.  
   
   %--------------------------------------------------
   % Lower Bound: Power iteration
   %--------------------------------------------------
   if ostruc.runspi
       % Run Power Iteration
       if ostruc.initialize || ii==1
         % initialize lower bound with no previous info
         bstart = vv(:,1);
         %          bstart = crandn(size(bstart));
         %          bstart = bstart/norm(bstart);
         wstart = bstart;
      else
         % initialize lower bound with previous result
         bstart = bprev;
         wstart = wprev;
      end
      if index.allcomp.num==0 && isreal(dMd)
         % Increase # of power iterations if pure real unc/pure real data
         [lb,Delta,b,w]=mmupiter(dMd,index,bstart,wstart,mulbopt_r);
      else
         [lb,Delta,b,w]=mmupiter(dMd,index,bstart,wstart,mulbopt);
      end
   else
       lb = 0;
       Delta = zeros(Mc,Mr);
       repc = index.allrep.allcols;
       repr = index.allrep.allrows;
       Delta(repc,repr) = 1e50*eye(length(repc));
       fullc = index.full.allcols;
       fullr = index.full.allrows;
       Delta(fullc,fullr) =  1e50*index.masks.DeltaFull_mask;
   end
          
   %--------------------------------------------------
   % Fast Upper Bound: XXX (?) should not run if numel(FixedBlkIdx)>0
   %--------------------------------------------------
   % Unfortunately, MUFASTUB needs upper and lower bounds, and seems to
   % rely heavily on these.  We can take the attitude that these should
   % just be for the standard-mu problem, letting MUFASTUB select G that
   % way, and then let MUFASTUB decide (at it's bottom) if there is
   % anything certified with regard to mu_{FV}.
   if hasFixedBlock
      ubValue = svDMD(1);
      lbValue = mmupiter(dMd,indexWithoutFixedBlock);
   else
      ubValue = ub; % also just svDMD(1)
      lbValue = lb;
   end
   if index.allreal.num>0 && ( ubValue > 1.005*max([lbValue 10*eps]) )
      % if there are fixed blocks, we are just trying to get "decent"
      % scalings
      [ubFastUB,DrFastUB,DcFastUB,GcrFastUB] = mufastub(dMd,index,[ubValue lbValue]);
      if ubFastUB<ub
         ub = ubFastUB; Dr = DrFastUB; Dc = DcFastUB; Gcr = GcrFastUB;
      end
   end

   % Call MUDESCENTUB if options imply "not fast" and "not best"
   if (~ostruc.fastupper) && (~ostruc.bestupper) && ( ub > 1.005*max([lb 10*eps]) )
      %--------------------------------------------------
      % Gradient Descent Upper Bound, initialize via OSBAL and/or MUFASTUB
      %--------------------------------------------------
      if isinf(ub)
         DGinit = mkDGinit(dMd,index,'G');
      else
         DGinit.ub=ub; DGinit.Dr=Dr; DGinit.Dc=Dc; DGinit.Gcr=Gcr;
         
      end
      [ub,Dr,~,Dc,Gcr] = mudescentub(dMd,index,DGinit);
      if ub>DGinit.ub
         ub = DGinit.ub; Dr = DGinit.Dr; Dc = DGinit.Dc; Gcr = DGinit.Gcr;
      end
   end
   
   % Call MULMIUB if option implies "best"
   if ostruc.bestupper && ( ub > 1.005*max([lb 10*eps]) )
      %--------------------------------------------------
      % LMI Upper Bound, initialize via OSBAL and/or MUFASTUB
      %--------------------------------------------------
      if isinf(ub)
         DGinit = mkDGinit(dMd,index,'a');
      else
         DGinit.ub=ub; DGinit.Dr=Dr; DGinit.Dc=Dc; DGinit.Gcr=Gcr;
      end
      [ub,Dr,~,~,Gcr] = mulmiub(dMd,index,DGinit);
      if ub>DGinit.ub
         ub = DGinit.ub; Dr = DGinit.Dr; Gcr = DGinit.Gcr;
      end
   end
   
   % power-iteration restarts computed after all upper-bound calculations
   % are complete.  Note that dMd variable does not change after its
   % creation in OSBAL (and scaling afterwards). Hence lower bound can be
   % executed at any location, and coordinates are still the same.
   if ostruc.runspi
      jj = 1;
      while lb<0.995*ub && jj<=ostruc.multiplelowertimes
         bstart = complex(RNG.rand(Mc,1),RNG.rand(Mc,1));
         bstart = bstart/(norm(bstart) + 10*eps);
         wstart = complex(RNG.rand(Mc,1),RNG.rand(Mc,1));
         wstart = wstart/(norm(wstart) + 10*eps);
         if index.allcomp.num==0 && isreal(dMd)
            % Increase # of power iterations if pure real unc/pure real data
            [lbtry,Deltatry,btry,wtry]=mmupiter(dMd,index,bstart,wstart,mulbopt_r);
         else
            [lbtry,Deltatry,btry,wtry]=mmupiter(dMd,index,bstart,wstart,mulbopt);
         end
         if lbtry>lb
            lb = lbtry;
            Delta = Deltatry;
            b = btry;
            w = wtry;
         end
         jj = jj + 1;
      end
      bprev = b;
      wprev = w;
   end
   
   %--------------------------------------------------
   % Gain-Based Lower Bound: WCGAIN (WCLOWC) on real parts and
   %    power iteration on complex parts
   %--------------------------------------------------
   if ostruc.runmuwcglb && index.allreal.num>0 && ub>100*eps
      % XXX 4/14/16 We can create an FV version of muwcglb
      [lbtry,Deltatry,deltareal] = muwcglb(dMd,index,deltareal,[ub lb],RNG,wcglbopt);
      if lbtry>lb
         lb = lbtry;
         Delta = Deltatry;
      end
   end
   
   %--------------------------------------------------
   % Unscale and prepare outputs:
   %--------------------------------------------------
   % Need ub>0 b/c mussv outputs D/G scales in the balanced
   % form (ami2ynrow will cause problems if ub==0)
   % XXX, nov 21, 2013: changed from 10*eps to 1e-3
%    ub = max(ub,1e-3);
%    ub = ub*scale;
%    lb = lb*scale;
%    Delta = Delta/scale;
%    normM=norm(M);
   ub = max(ub,1e-3);
   ub = ub*scale;
   lb = lb*scale;
   Delta(index.FVidx.VaryCols,index.FVidx.VaryRows) = Delta(index.FVidx.VaryCols,index.FVidx.VaryRows)/scale;
   normM=norm(M);
   
   if ub<normM || (hasFixedBlock && isfinite(ub))
      % Dr = Dr_os'*Dr*Dr_os;  % sqrtm(Dr) formed below, and passed
      % directly to ami2ynrow
      Gcr = scale*(Dc_os'*Gcr*Dr_os);
      
      % Store scalings as row vectors
      Dcell = cell(nblk,1);  Gcell = cell(nblk,1);
      for i=1:nblk
         if blk(i,2)==0
            rptr = ridxm(i):(ridxm(i+1)-1);
            cptr = cidxm(i):(cidxm(i+1)-1);
         else
            rptr = ridxm(i);
            cptr = cidxm(i);
         end
         Gcell{i} = Gcr(cptr,rptr);
         % block-by-block SQRTM of Dr_os'*Dr*Dr_os
         if any(i==fixedBlkIdx)
            DrBlk = scale*Dr(rptr,rptr);
         else
            DrBlk = Dr(rptr,rptr);
         end
         Dr_osBlk = Dr_os(rptr,rptr);
         [u,d] = schur(DrBlk);
         d = sqrt(diag(d));
         [~,s,v] = svd(lrscale(u'*Dr_osBlk,d,[]));   % diag(d)*u’* Dr_os
         Dcell{i} = v*s*v';
      end
      sqrtmFlag = true;
      [trowd,trowg] = rctutil.ami2ynrow(Dcell,Gcell,blk,ub,sqrtmFlag);
   else
      if ~hasFixedBlock
         ub = normM;
      end
      % sigmaub stores identity D-scales and a zero G-scales
      [trowd,trowg] = sigmaub(blk);
   end
   bnds(1,:,ii) = [ub lb];
   
   % Store Delta as rep reals, rep comp, complex full
   trowp = zeros(1,wp);
   cnt=1;
   for i=1:nblk
      if blk(i,1)<0 && blk(i,2)==0
         rptr = ridxm(i);
         cptr = cidxm(i);
         trowp(cnt) = Delta(cptr,rptr);
         cnt=cnt+1;
      end
   end
   for i=1:nblk
      if blk(i,1)>0 && blk(i,2)==0
         rptr = ridxm(i);
         cptr = cidxm(i);
         trowp(cnt) = Delta(cptr,rptr);
         cnt=cnt+1;
      end
   end
   tmp = Delta(index.full.allcols,index.full.allrows);
   trowp(index.allrep.num+1:end) = tmp(index.masks.DeltaFull_mask~=0);
   
   if wg,  rowg(1,:,ii) = trowg;  end
   rowd(1,:,ii) = trowd;
   if lb>0
      % Leave ROWP equal to 0 for LowerBound=0. We have no Delta that
      % causes singularity, so 0 is fine.
      rowp(1,:,ii) = trowp;
   end
   
   %--------------------------------------------------
   % Sensitivity calculation
   %--------------------------------------------------
   [tdl,tdr] = mussvunwrap(trowd,blk);
   sm = tdl*M/tdr;
   if nblk == 1
      sens(1,1,ii) = 1;
   else
      for ib=1:nblk
         sensr = sm(ridxm(ib):ridxm(ib+1)-1,[1:cidxm(ib)-1 cidxm(ib+1):Mc]);
         sensc = sm([1:ridxm(ib)-1 ridxm(ib+1):Mr],cidxm(ib):cidxm(ib+1)-1);
         sens(1,ib,ii) = norm(sensr) + norm(sensc);
      end
   end
   
   %--------------------------------------------------
   % Display progress information
   %--------------------------------------------------
   %     if ostruc.gui && (ii/density >= pcnt)
   %         set(mdata(1),'string',[int2str(ii) '/' int2str(npts)]);
   %         drawnow;
   %         pcnt = pcnt+1;
   %     end
   
   if ~ostruc.silent
      fprintf(repmat('\b',[1 length(lptxt)]))
      lptxt = sprintf('%d/%d',ii,npts);
      fprintf(lptxt);
   end
   
end % for ii

if ~ostruc.silent && ostruc.backup
   nonewline = repmat('\b',1,length(strl)+length(lptxt));
   fprintf(nonewline)
elseif ~ostruc.silent
   fprintf('\n')
end

% Scale the upper bound by a small amount due to numerical rounding
bnds(1,1,:) = (1+5e-7)*bnds(1,1,:);
info.bnds = bnds;

% Store Outputs
if isa(matin,'frd')
   bnds = frd(bnds,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
   sens = frd(sens,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
   rowd = frd(rowd,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
   rowp = frd(rowp,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
   rowg = frd(rowg,matin.Frequency,Ts,'FrequencyUnit',matin.FrequencyUnit);
end

info.dvec = rowd;
info.pvec = rowp;
info.gvec = rowg;
info.sens = sens;
info.blk = blk;
info.bnds = bnds;
