%MUSSVUNWRAP  Unwraps the INFO structure returned by MUSSV.
%   Not intended to be used by users.
%
%   The number of output arguments specifies what should be
%   returned.  The various alternatives are:
%
%	                  [delta] = MUSSVUNWRAP(MUINFO)       or
%	                  [dl,dr] = MUSSVUNWRAP(MUINFO)       or
%	               [gl,gm,gr] = MUSSVUNWRAP(MUINFO)       or
%	            [Dr,Dc,Gr,Gc] = MUSSVUNWRAP(MUINFO) (LMI) or
%	         [dl,dr,gl,gm,gr] = MUSSVUNWRAP(MUINFO)       or
%	   [dl,dr,gl,gm,gr,delta] = MUSSVUNWRAP(MUINFO)
%
%	 See also: MUSSV

% Copyright 2003-2005 The MathWorks, Inc.

% For us, we can also call the old way, as below...
%	               [delta] = MUSSVUNWRAP(PVEC,BLK)            or
%	               [dl,dr] = MUSSVUNWRAP(DVEC,BLK)            or
%	            [gl,gm,gr] = MUSSVUNWRAP(GVEC,BLK)            or
%	         [Dr,Dc,Gr,Gc] = MUSSVUNWRAP(DVEC,GVEC,BLK,BNDS) (LMI) or
%	      [dl,dr,gl,gm,gr] = MUSSVUNWRAP(DVEC,GVEC,BLK)       or
%	[dl,dr,gl,gm,gr,delta] = MUSSVUNWRAP(DVEC,GVEC,PVEC,BLK)
%
%
% fields in MUINFO are (dvec,pvec,gvec,sens,blk,bnds)

function [varargout] = mussvunwrap(varargin)

if nargout == 1
   if nargin==1
      varargout{1} = LOCALunwrapp(varargin{1}.pvec,varargin{1}.blk);
   elseif nargin==2
      varargout{1} = LOCALunwrapp(varargin{1:2});
   end
elseif nargout == 2
   varargout = cell(2,1);
   if nargin==1
      [varargout{:}] = LOCALunwrapd(varargin{1}.dvec,varargin{1}.blk);
   elseif nargin==2
      [varargout{:}] = LOCALunwrapd(varargin{1:2});
   end
elseif nargout == 3
   varargout = cell(3,1);
   if nargin==1
      [varargout{:}] = LOCALunwrapg(varargin{1}.gvec,varargin{1}.blk);
   elseif nargin==2
      [varargout{:}] = LOCALunwrapg(varargin{1:2});
   end
elseif nargout == 4
   varargout = cell(4,1);
   if nargin==1
      [varargout{:}] = LOCALLMIunwrapdg(varargin{1}.dvec,...
         varargin{1}.gvec,varargin{1}.blk,varargin{1}.bnds);
   elseif nargin==4
      [varargout{:}] = LOCALLMIunwrapdg,(varargin{1:4});
   else
      error('Invalid Input Arguments')
   end
elseif nargout == 5
   varargout = cell(5,1);
   if nargin==1
      [varargout{1},varargout{2}] = LOCALunwrapd(varargin{1}.dvec,varargin{1}.blk);
      [varargout{3},varargout{4},varargout{5}] = LOCALunwrapg(varargin{1}.gvec,varargin{1}.blk);
   elseif nargin==3
      [varargout{1},varargout{2}] = LOCALunwrapd(varargin{[1 3]});
      [varargout{3},varargout{4},varargout{5}] = LOCALunwrapg(varargin{[2 3]});
   else
      error('Invalid Input Arguments')
   end
elseif nargout == 6
   varargout = cell(6,1);
   if nargin==1
      [varargout{1},varargout{2}] = LOCALunwrapd(varargin{1}.dvec,varargin{1}.blk);
      [varargout{3},varargout{4},varargout{5}] = LOCALunwrapg(varargin{1}.gvec,varargin{1}.blk);
      varargout{6} = LOCALunwrapp(varargin{1}.pvec,varargin{1}.blk);
   elseif nargin==4
      [varargout{1},varargout{2}] = LOCALunwrapd(varargin{[1 4]});
      [varargout{3},varargout{4},varargout{5}] = LOCALunwrapg(varargin{[2 4]});
      varargout{6} = LOCALunwrapp(varargin{[3 4]});
   else
      error('Invalid Input Arguments')
   end
      
else
   error('Invalid input argument')
end

function [dl,dr] = LOCALunwrapd(rowd,blk)

if isa(rowd,'double')
   szm = size(rowd);
   exd = szm(3:end);
   npts = prod(exd);
   mat = reshape(rowd,[szm(1) szm(2) npts]);
elseif isa(rowd,'frd')
   szm = size(rowd.ResponseData);
   exd = szm(3:end);
   npts = prod(exd);
   mat = reshape(rowd.ResponseData,[szm(1) szm(2) npts]);
else
   % XXX better error message
   error('Only DOUBLE, PMAT and FRD')
end
mnum = npts;
drows = szm(1);
dcols = szm(2);

if drows~=1
   error('Invalid ROWD, incorrect number of rows')
end

if length(blk(1,:))~=2 | any(abs(round(real(blk))-blk) > 1e-6)
   error('   BLK is invalid')
elseif any(round(real(blk(:,1)))) == 0 | any(isnan(abs(blk)))
   error('   BLK is invalid')
else
   blk = round(real(blk));
end
for ii=1:length(blk(:,1))
   if all( blk(ii,:) == [ 1 0] )
      blk(ii,:)  = [ 1 1];
   end
   if all( blk(ii,:) == [-1 1] )
      blk(ii,:) = [-1 0];
   end
   if all( blk(ii,:) == [-1 -1] )
      blk(ii,:)  = [-1 0] ;
   end
end
if any(blk(:,2)<0)
	error('   BLK is invalid');
end
if any(blk(:,2)~=0&blk(:,1)<0),
	error('   Real FULL blocks not allowed');
end
if any(abs(blk)>500)
   error('   No blocks larger than 500, please')
end

[Or,Oc,Ur,Uc,K,I,J] = reindex(blk);

[Kc,Kr,Ic,Ir,Jc,Jr,Fc,Fr,kc,kr,ic,ir,jc,jr,fc,fr,sc,sr,csc,csr,...
 csF,nK,nI,nJ,nF,nc,nr,kcK,icI,jcJ,jrJ,fcF] = rubind(K,I,J);

dmask = fcF'*fcF;
wd = sum(sum(dmask)) + nJ;
wdf = sum(sum(dmask));
wg  = sum(K);
pmask = jrJ'*jcJ;
wp = sum(sum(pmask)) + nF;
wps = nF;

LKc = logical(Kc); LKr = logical(Kr);
LFc = logical(Fc); LFr = logical(Fr);
LJc = logical(Jc); LJr = logical(Jr);

Ldmask = logical(dmask); Lpmask = logical(pmask);

if dcols~=wd
   error('   ROWD is the wrong size')
end

dl = zeros([csc(4) csc(4) exd]);
dr = zeros([csr(4) csr(4) exd]);
tdl = zeros(csc(4));
tdr = zeros(csr(4));
tdf = zeros(csc(3),csc(3));
for i=1:npts
   trowd = mat(:,:,i);
   if sum(Fc)
	   tdf(Ldmask) = trowd(1:wdf);
	   tdl(LFc,LFc) = tdf;
	   tdr(LFr,LFr) = tdf;
   end
   if sum(Jc)
      tdl(LJc,LJc) = diag(diag(jcJ'*diag(trowd(wdf+1:wd))*jcJ));
      tdr(LJr,LJr) = diag(diag(jrJ'*diag(trowd(wdf+1:wd))*jrJ));
   end
   dl(:,:,i) = tdl(Uc,Uc);
   dr(:,:,i) = tdr(Ur,Ur);
end
if isa(rowd,'frd')
   dl = frd(dl,rowd.Frequency,rowd.Ts,'Units',rowd.Units);
   dr = frd(dr,rowd.Frequency,rowd.Ts,'Units',rowd.Units);
%    dl.Ts = rowd.Ts;
%    dr.Ts = rowd.Ts;
end

function [gl,gm,gr] = LOCALunwrapg(rowg,blk)

if isa(rowg,'double')
   szm = size(rowg);
   exd = szm(3:end);
   npts = prod(exd);
   mat = reshape(rowg,[szm(1) szm(2) npts]);
elseif isa(rowg,'frd')
   szm = size(rowg.ResponseData);
   exd = szm(3:end);
   npts = prod(exd);
   mat = reshape(rowg.ResponseData,[szm(1) szm(2) npts]);
else
   % XXX better error message
   error('Only DOUBLE, PMAT and FRD')
end
mnum = npts;
grows = szm(1);
gcols = szm(2);

if grows==0
   idx = find(blk(:,2)==0);
   blk(idx,2) = blk(idx,1);
   nr = sum(blk(:,2));
   nc = sum(blk(:,1));
   gl = zeros(nr);
   gm = zeros(nr,nc);
   gr = zeros(nc);
   return
elseif grows~=1
   error('Invalid ROWG, incorrect number of rows')
end

if length(blk(1,:))~=2 | any(abs(round(real(blk))-blk) > 1e-6)
   error('   BLK is invalid')
elseif any(round(real(blk(:,1)))) == 0 | any(isnan(abs(blk)))
   error('   BLK is invalid')
else
   blk = round(real(blk));
end
for ii=1:length(blk(:,1))
   if all( blk(ii,:) == [ 1 0] )
      blk(ii,:)  = [ 1 1];
   end
   if all( blk(ii,:) == [-1 1] )
      blk(ii,:) = [-1 0];
   end
   if all( blk(ii,:) == [-1 -1] )
      blk(ii,:)  = [-1 0] ;
   end
end
if any(blk(:,2)<0)
	error('   BLK is invalid');
end
if any(blk(:,2)~=0&blk(:,1)<0),
	error('   Real FULL blocks not allowed');
end
if any(abs(blk)>500)
   error('   No blocks larger than 500, please')
end

[Or,Oc,Ur,Uc,K,I,J] = reindex(blk);

[Kc,Kr,Ic,Ir,Jc,Jr,Fc,Fr,kc,kr,ic,ir,jc,jr,fc,fr,sc,sr,csc,csr,...
 csF,nK,nI,nJ,nF,nc,nr,kcK,icI,jcJ,jrJ,fcF] = rubind(K,I,J);

dmask = fcF'*fcF;
wd = sum(sum(dmask)) + nJ;
wdf = sum(sum(dmask));
wg  = sum(K);
pmask = jrJ'*jcJ;
wp = sum(sum(pmask)) + nF;
wps = nF;

LKc = logical(Kc); LKr = logical(Kr);
LFc = logical(Fc); LFr = logical(Fr);
LJc = logical(Jc); LJr = logical(Jr);

Ldmask = logical(dmask); Lpmask = logical(pmask);

if gcols~=wg
   error('  ROWG is the wrong size')
end

gl = zeros([csc(4) csc(4) exd]);
gm = zeros([csc(4) csr(4) exd]);
gr = zeros([csr(4) csr(4) exd]);
tgl = zeros(csc(4));
tgm = zeros(csc(4),csr(4));
tgr = zeros(csr(4));
for i=1:npts
   trowg = mat(:,:,i);
   tgk = diag(trowg);
   tgl(LKc,LKc) = tgk;
   tgm(LKc,LKr) = tgk;
   tgr(LKr,LKr) = tgk;
   gl(:,:,i) = tgl(Uc,Uc);
   gm(:,:,i) = tgm(Uc,Ur);
   gr(:,:,i) = tgr(Ur,Ur);
end
if isa(rowg,'frd')
   gl = frd(gl,rowg.Frequency,rowg.Ts,'Units',rowg.Units);
   gm = frd(gm,rowg.Frequency,rowg.Ts,'Units',rowg.Units);
   gr = frd(gr,rowg.Frequency,rowg.Ts,'Units',rowg.Units);
%    gl.Ts = rowg.Ts;
%    gm.Ts = rowg.Ts;
%    gr.Ts = rowg.Ts;
end

function [pert] = LOCALunwrapp(rowp,blk)

if isa(rowp,'double')
   szm = size(rowp);
   exd = szm(3:end);
   npts = prod(exd);
   mat = reshape(rowp,[szm(1) szm(2) npts]);
elseif isa(rowp,'frd')
   szm = size(rowp.ResponseData);
   exd = szm(3:end);
   npts = prod(exd);
   mat = reshape(rowp.ResponseData,[szm(1) szm(2) npts]);
else
   % XXX better error message
   error('Only DOUBLE, PMAT and FRD')
end
mnum = npts;
prows = szm(1);
pcols = szm(2);

if prows~=1
   error('Invalid ROWP, incorrect number of rows')
end

if length(blk(1,:))~=2 | any(abs(round(real(blk))-blk) > 1e-6)
   error('   BLK is invalid')
elseif any(round(real(blk(:,1)))) == 0 | any(isnan(abs(blk)))
   error('   BLK is invalid')
else
   blk = round(real(blk));
end
for ii=1:length(blk(:,1))
   if all( blk(ii,:) == [ 1 0] )
      blk(ii,:)  = [ 1 1];
   end
   if all( blk(ii,:) == [-1 1] )
      blk(ii,:) = [-1 0];
   end
   if all( blk(ii,:) == [-1 -1] )
      blk(ii,:)  = [-1 0] ;
   end
end
if any(blk(:,2)<0)
	error('   BLK is invalid');
end
if any(blk(:,2)~=0&blk(:,1)<0),
	error('   Real FULL blocks not allowed');
end
if any(abs(blk)>500)
   error('   No blocks larger than 500, please')
end

[Or,Oc,Ur,Uc,K,I,J] = reindex(blk);

[Kc,Kr,Ic,Ir,Jc,Jr,Fc,Fr,kc,kr,ic,ir,jc,jr,fc,fr,sc,sr,csc,csr,...
 csF,nK,nI,nJ,nF,nc,nr,kcK,icI,jcJ,jrJ,fcF] = rubind(K,I,J);

dmask = fcF'*fcF;
wd = sum(sum(dmask)) + nJ;
wdf = sum(sum(dmask));
wg  = sum(K);
pmask = jrJ'*jcJ;
wp = sum(sum(pmask)) + nF;
wps = nF;

LKc = logical(Kc); LKr = logical(Kr);
LFc = logical(Fc); LFr = logical(Fr);
LJc = logical(Jc); LJr = logical(Jr);

Ldmask = logical(dmask); Lpmask = logical(pmask);

if pcols~=wp
   error('ROWP is the wrong size')
end

pert =	zeros([csr(4) csc(4) exd]);
tprt = zeros(csr(4),csc(4));
tpj  = zeros(sum(Jr),sum(Jc));
for i=1:npts
   trowp = mat(:,:,i);
   if sum(Jc),
      tpj(Lpmask) = trowp(wps+1:wp);
      tprt(LJr,LJc) = tpj;
   end
   if sum(Fc)
      tprt(LFr,LFc) = diag(diag(fcF'*diag(trowp(1:wps))*fcF));
   end
   pert(:,:,i) = tprt(Ur,Uc);
end
if isa(rowp,'frd')
   pert = frd(pert,rowp.Frequency,rowp.Ts,'Units',rowp.Units);
%    pert.Ts = rowp.Ts;
end

function [Dr,Dc,Grc,Gcr] = LOCALLMIunwrapdg(rowd,rowg,blk,bnds)

tblk = abs(blk);
idx = find(tblk(:,2)==0);
tblk(idx,2) = tblk(idx,1);
rcdim = sum(tblk,1);
Nr = rcdim(2);
Nc = rcdim(1);


if isa(rowd,'double') && isa(rowg,'double') && isa(bnds,'double')
   szmD = size(rowd);
   exdD = szmD(3:end);
   szmG = size(rowg);
   exdG = szmG(3:end);
   szmB = size(bnds);
   exdB = szmB(3:end);
   if isequal(exdD,exdG) && isequal(exdD,exdB)
      npts = prod(exdD);
      exd = exdD;
      matD = reshape(rowd,[szmD(1) szmD(2) npts]);
      matG = reshape(rowg,[szmG(1) szmG(2) npts]);
      matB = reshape(bnds(1,1,:),[1 1 npts]);
   else
      error('Corrupt D/G/BNDS data.');
   end
elseif isa(rowd,'frd') && isa(rowg,'frd') && isa(bnds,'frd')
   szmD = size(rowd.ResponseData);
   exdD = szmD(3:end);
   szmG = size(rowg.ResponseData);
   exdG = szmG(3:end);
   szmB = size(bnds.ResponseData);
   exdB = szmB(3:end);
   if isequal(exdD,exdG) && isequal(exdD,exdB)
      npts = prod(exdD);
      exd = exdD;
      matD = reshape(rowd.ResponseData,[szmD(1) szmD(2) npts]);
      matG = reshape(rowg.ResponseData,[szmG(1) szmG(2) npts]);
      matB = reshape(bnds.ResponseData(1,1,:),[1 1 npts]);
   else
      error('Corrupt D/G/BNDS data.');
   end
else
   error('D, G and BNDS data must be compatible DOUBLE or FRD')
end

Dr = zeros([Nr Nr exd]);
Dc = zeros([Nc Nc exd]);
Grc = zeros([Nr Nc exd]);
Gcr = zeros([Nc Nr exd]);
for i=1:npts
   [Dr(:,:,i),Dc(:,:,i),Grc(:,:,i),Gcr(:,:,i)] = ...
      ynftdam2(matD(1,:,i),matG(1,:,i),blk,matB(1,:,i));
end
if isa(rowd,'frd')
   Dr = frd(Dr,rowd.Frequency,rowd.Ts,'Units',rowd.Units);
   Dc = frd(Dc,rowd.Frequency,rowd.Ts,'Units',rowd.Units);
   Grc = frd(Grc,rowd.Frequency,rowd.Ts,'Units',rowd.Units);
   Gcr = frd(Gcr,rowd.Frequency,rowd.Ts,'Units',rowd.Units);
end

