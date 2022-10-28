function [bnds,info] = mussvcalc(matin,blk,opt,mdata,gflag,SETUP)
%

%   Copyright 2004-2011 The MathWorks, Inc.

nin = nargin;
nout = nargout;
if nin == 2
    opt = '';
    gflag = [];
    mdata = [];
    SETUP = [];
 elseif nin == 3
    mdata = [];
    gflag = [];
    SETUP = [];
 elseif nin == 4
    gflag = [];
    SETUP = [];
 elseif nin == 5
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
end
ostruc.silent = any(opt=='s');
ostruc.decreaselower = any(opt=='x');
ostruc.decreaselowerto2 = any(opt=='U');
ostruc.increaselower = any(opt=='t');
ostruc.gui = any(opt=='g');
ostruc.backup = any(opt=='b');



if ~ostruc.displaywarnings
   hw = ctrlMsgUtils.SuspendWarnings; %#ok<NASGU>
end

% BLKIN = blk;
% OPTIN = opt;

if isempty(mdata)
   mdata = rand(1,2);
end
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
   % make N-D into 3-D
   mat = reshape(freqresp(matin,matin.Frequency,matin.FrequencyUnit),[szm(1) szm(2) npts]); % note RESPONSEDATA is DOUBLE
else
   error('Only DOUBLE and FRD data are allowed')
end
if isempty(gflag)
   gflag = npts; %#ok<NASGU>
end
mnum = npts;
mrows = szm(1);
mcols = szm(2);

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
   
   return
end

if (isempty(mat) || isempty(blk)) && isempty(SETUP)
   error('   MATIN dimensions incompatible with BLK dimensions')
end
if size(blk,2)~=2 || norm(round(real(blk))-blk) > 1e-6
    error('   BLK is invalid')
elseif any(round(real(blk(:,1)))) == 0 || any(isnan(blk(:)))
    error('   BLK is invalid')
else
    blk = round(real(blk));
end
for ii=1:length(blk(:,1))
   if all( blk(ii,:) == [ 1 0] )
      blk(ii,:)  = [ 1 1] ;
   end
   if all( blk(ii,:) == [-1 1] )
      blk(ii,:)  = [-1 0] ;
   end
end
for ii=1:length(blk(:,1))
   if all(blk(ii,:) == [-1 -1])
      blk(ii,:) = [-1 0];
   end
end
if any(blk(:,2)<0)
   error('BLK is invalid');
end
if any(blk(:,2)~=0&blk(:,1)<0),
   error('Real FULL blocks not allowed');
end
if any(abs(blk)>500)
   error('No blocks larger than 500, please')
end

%
%  Insert at line 111, uses (mat, blk), changes mat
%
if ostruc.bestupper && ~ostruc.bestuppernoscaling
   [~,initinfo] = mussv(mat,abs(blk),'sx');
   [~,initSig] = mussvextract(initinfo);
   mat = LOCALdmdi(initSig.DLeft,mat,initSig.DRight);
end
%
%
%%%%%%%%%%%%%%%%%

numits = [30; 200];      contol = 1.001;
% numits(1)  = 10;  contol = 1.01;
%if any(opt=='c'), numits(1)  = 30;  contol = 1.001;     end
if ostruc.decreaselower, numits(2)  = 50;             end
if ostruc.decreaselowerto2, numits(2)  = 2;             end

nblk = size(blk,1);

[Or,Oc,~,~,K,I,J] = reindex(blk);
if (any([length(Oc),length(Or)]~=[mrows,mcols])) && isempty(SETUP)
    error('   MATIN dimensions incompatible with BLK dimensions')
end
[Kc,Kr,Ic,Ir,Jc,Jr,Fc,Fr,kc,kr,ic,ir,jc,jr,fc,fr,sc,sr,csc,csr,...
        csF,nK,nI,nJ,nF,nc,nr,kcK,icI,jcJ,jrJ,fcF] = rubind(K,I,J);

LFr = logical(Fr); LFc = logical(Fc);
LJr = logical(Jr); LJc = logical(Jc);

if ~sum(Ic+Jc)
    if ostruc.displaywarnings
       warning('Pure-real MUSSV problem, LowerBound convergence difficulties expected.');
    end
    numits(2) = 1.5*numits(2);
end

pmask = jrJ'*jcJ;
ps = [];
wp = sum(sum(pmask)) + nF;
dmask = fcF'*fcF;
ds = [];
wd = sum(sum(dmask)) + nJ;

Lpmask = logical(pmask);
Ldmask = logical(dmask);

wg = sum(K);

% initialize N-D output matrices
bnds =  zeros([1 2 exd]);
rowp =  zeros([1 wp exd]);
rowd =  zeros([1 wd exd]);
sens =  zeros([1 nblk exd]);
if wg
   rowg = zeros([1 wg exd]);
else
   rowg = zeros([1 0 exd]);
end % if wg
if ~isempty(SETUP)
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
	info.blk = blk; %BLKIN;
   info.bnds = bnds;
   return
end

commonptr = zeros(size(blk,1)+1,1);
commonptr(1) = 1;
pgb = cell(size(blk,1),1);
for pg=1:size(blk,1)
  if blk(pg,1)<0 && blk(pg,2)==0
     pgb{pg} = ublock(-blk(pg,1),1,'ltisr');
     commonptr(pg+1) = commonptr(pg) + (-blk(pg,1));
  elseif blk(pg,1)>0 && blk(pg,2)==0
     pgb{pg} = ublock(blk(pg,1),1,'ltisc');
     commonptr(pg+1) = commonptr(pg) + (blk(pg,1));
  else
     pgb{pg} = ublock(blk(pg,:),1,'ltifc');
     % LMI/MUBND makes everything square internally
     commonptr(pg+1) = commonptr(pg) + max(blk(pg,:));
  end
  if pg==1
     pgDelta = pgb{1};
  else
     pgDelta = udiag(pgDelta,pgb{pg});
  end
end
%pgDelta = udiag(pgb{:});  % UDIAG only works with 10 or less
Dcell = cell(size(blk,1),1);
Gcell = cell(size(blk,1),1);


if ~ostruc.silent  
   strl = ['points completed (of ' int2str(mnum) ') ... ' ];
   fprintf(strl)
end
density = mdata(2);
pcnt = 1;
lptxt = '';
for ii = 1:mnum
   M = mat(:,:,ii);
   mtmp = M;
   M = M(Oc,Or);
   if ostruc.initialize || ii==1    % call with no previous INFO
         [tbnds,prt,dc,dri,dcb,drib,dcd,~,g,y,b] = ...
          rub(M,K,I,J,ostruc,numits,contol,Kc,Kr,Ic,Ir,Jc,Jr,...
          Fc,Fr,kc,kr,ic,ir,jc,jr,fc,fr,sc,sr,csc,csr,csF,...
          nK,nI,nJ,nF,nc,nr,kcK,icI,jcJ,jrJ,fcF);
   elseif ostruc.fastupper
         [tbnds,prt,dc,dri,dcb,drib,dcd,~,g,y,b] = ...
          rub(M,K,I,J,ostruc,numits,contol,Kc,Kr,Ic,Ir,Jc,Jr,...
          Fc,Fr,kc,kr,ic,ir,jc,jr,fc,fr,sc,sr,csc,csr,csF,...
          nK,nI,nJ,nF,nc,nr,kcK,icI,jcJ,jrJ,fcF,dcb,drib,y,b);
   else
         [tbnds,prt,dc,dri,dcb,drib,dcd,~,g,y,b] = ...
          rub(M,K,I,J,ostruc,numits,contol,Kc,Kr,Ic,Ir,Jc,Jr,...
          Fc,Fr,kc,kr,ic,ir,jc,jr,fc,fr,sc,sr,csc,csr,csF,...
          nK,nI,nJ,nF,nc,nr,kcK,icI,jcJ,jrJ,fcF,dcb,drib,y,b,dc,dri,g,tbnds(1));
   end
   trowd = []; trowp = [];
   if sum(Fc)
         ps = sum(fcF'.*(diag(prt(LFr,LFc))*ones(1,nF)))./sum(fcF');
         df = dcd(LFc,LFc); trowd = df(Ldmask);
   end
   if sum(Jc)
         ds = max(jcJ'.*(diag(dcd(LJc,LJc))*ones(1,nJ)));
         pj = prt(LJr,LJc); trowp = pj(Lpmask);
   end
   trowp = [ps(:); trowp(:)].';
   trowd = [trowd(:); ds(:)].';
   trowg = g.';

   nrm = norm(mtmp);
   nogo = 0;
   if tbnds(1,1)<1e-5 && nrm>1e-3
         nogo = 1;
   end
   if ostruc.bestupper && ~nogo
      [newu,pgD,pgG] = mubnd(mtmp,pgDelta,[],'off');
      for i=1:size(blk,1)
         if blk(i,2)==0
            loc = commonptr(i):commonptr(i+1)-1;
         else
            loc = commonptr(i);
         end
         Dcell{i} = pgD(loc,loc);
         Gcell{i} = pgG(loc,loc);
      end
      [trowd,trowg] = rctutil.ami2ynrow(Dcell,Gcell,blk,newu);
      tbnds(1,1) = newu;
   end
   nrmbigm = norm(M);
   if tbnds(1,1) > nrmbigm
        [trowd,trowg] = sigmaub(blk);
        tbnds(1,1) = nrmbigm;
   end

   bnds(1,:,ii) = tbnds;
   rowp(1,:,ii) = trowp;
   rowd(1,:,ii) = trowd;
   if wg,  rowg(1,:,ii) = trowg;  end

   if ostruc.gui
        if ii/density >= pcnt
            set(mdata(1),'string',[int2str(ii) '/' int2str(mnum)]);
            drawnow;
            pcnt = pcnt+1;
        end
   end

   if ~ostruc.silent
      if 10*floor(ii/10)==ii
         ptxt = int2str(ii);
         llptxt = length(lptxt);
         for count=1:llptxt
            fprintf('\b');
         end
         fprintf(ptxt);
         lptxt = ptxt;
      end
   end

   [tdl,tdr] = mussvunwrap(trowd,blk);
   %
   sm = tdl*mtmp/tdr;
   tblkp = ptrs(abs(blk));
   if nblk == 1
      sens(1,1,ii) = 1;
   else
      for ib=1:nblk
         sensr = sm(tblkp(ib,2):tblkp(ib+1,2)-1,[1:tblkp(ib,1)-1 tblkp(ib+1,1):mcols]);
         sensc = sm([1:tblkp(ib,2)-1 tblkp(ib+1,2):mrows],tblkp(ib,1):tblkp(ib+1,1)-1);
         sens(1,ib,ii) = norm(sensr) + norm(sensc);
      end
   end
end  % for ii
if ~ostruc.silent && ostruc.backup
   nonewline = repmat('\b',1,length(strl)+length(lptxt));
   fprintf(nonewline)
elseif ~ostruc.silent
   fprintf('\n')
end

% Scale the upper bound by 1+1e-8 due to numerical rounding
bnds(1,1,:) = (1+5e-7)*bnds(1,1,:);

%
%  Insertion, uses rowp, rowd, rowg, blk, bnds
%  updates rowp, rowd, rowg, sens, bnds
%
if ostruc.bestupper && ~ostruc.bestuppernoscaling
   % need to update rowd and rowg
   infolayer2.pvec = rowp;
   infolayer2.dvec = rowd;
   infolayer2.gvec = rowg;
   infolayer2.sens = sens;
   infolayer2.blk = blk;
   infolayer2.bnds = bnds;
   [~,~,Lm2] = mussvextract(infolayer2);
   Lm.Dr = LOCALactbc(initSig.DLeft,Lm2.Dr,initSig.DLeft);
   Lm.Dc = LOCALactbc(initSig.DRight,Lm2.Dc,initSig.DRight);
   Lm.Gcr = LOCALactbc(initSig.DRight,Lm2.Gcr,initSig.DLeft);
   Lm.Grc = LOCALactbc(initSig.DLeft,Lm2.Grc,initSig.DRight);
   [rowd,rowg] = LOCALami2row(Lm.Dc,Lm.Gcr,blk,bnds(1,1,:));
   % Sens will be original sens from simple DMDI scaling
   sens = initinfo.sens;
end
%
% Continue at line 316 
%



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
info.blk = blk; %BLKIN;
info.bnds = bnds;


% LOCAL functions, belong at bottom
function [rowd,rowg] = LOCALami2row(Dc,Gcr,blk,ubnd)
% Dc is col(M) dimensional, use rows of Delta to access
% Gcr is shaped like Delta, use Delta to access
szD = size(Dc);
exd = szD(3:end);
nblk = size(blk,1);
Dcell = cell(nblk,1);
Gcell = cell(nblk,1);
for k=1:prod(exd)
   UpperBound = ubnd(1,1,k);
   rp = 1; % in Delta
   cp = 1; % in Delta
   for i=1:size(blk,1)
      if blk(i,2)==0
         rend = rp+abs(blk(i,1))-1;
         cend = cp+abs(blk(i,1))-1;
         Dcell{i} = Dc(rp:rend,rp:rend,k);
         Gcell{i} = Gcr(rp:rend,cp:cend,k);
         rp = rend + 1;
         cp = cend + 1;
      else
         rend = rp+blk(i,1)-1;
         cend = cp+blk(i,2)-1;
         Dcell{i} = Dc(rp,rp,k);
         Gcell{i} = Gcr(rp,cp,k);
         rp = rend + 1;
         cp = cend + 1;
      end
   end
   [trowd,trowg] = rctutil.ami2ynrow(Dcell,Gcell,blk,UpperBound);
   if isempty(trowg)  % See MUSSV - empty G are handled special
      trowg = zeros(1,0);
   end
   if k==1
      rowd = zeros([size(trowd) exd]);
      rowg = zeros([size(trowg) exd]);
   end
   rowd(:,:,k) = trowd;
   rowg(:,:,k) = trowg;
end

function m = LOCALactbc(A,B,C)
% M = A'*B*C, assumes all array dims line up.
szA = size(A);
szC = size(C);
exd = szA(3:end);
m = zeros([szA(2) szC(2) exd]);
at = conj(permute(A,[2 1 3:length(szA)]));
for k=1:prod(exd)
   m(:,:,k) = at(:,:,k)*B(:,:,k)*C(:,:,k);
end

function X = LOCALdmdi(DL,M,DR)
% X = DL*M/DR, assumes all array dims line up.
szM = size(M);
exd = szM(3:end);
X = zeros(szM);
for k=1:prod(exd)
   X(:,:,k) = DL(:,:,k)*M(:,:,k)/DR(:,:,k);
end

