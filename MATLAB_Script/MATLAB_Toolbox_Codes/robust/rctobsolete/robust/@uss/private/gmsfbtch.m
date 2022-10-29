function [icnew,chekbest,dsysl,dsysr,gsysm,gsysr,gsysfc] =  ...
   gmsfbtch(icprev,bnds,muinfo,sens,blk,clpg,maxord,visflag)
% function [icnew,chekbest,dsysl,dsysr,gsysm,gsysr,gsysfc] =  ...
%     gmsfbtch(icprev,bnds,muinfo,sens,blk,clpg,maxord,visflag)
%
% Fits frequency response data with system matrices.
% All computations are done in non-interactive batch mode.
% Assumes the mixed mu upper bound from MUN has been 
% run to generate the data ROWD, ROWG, BNDS, and SENS.
% Returns fits to the D scaling in DL and DR, and fits 
% to the G scaling in GM, GR and GFC.  In addition ICN 
% contains the new interconnection formed from the above 
% systems for the next iteration of H_infinity design in 
% a mixed mu synthesis procedure.
%
% See Also MSFN, DKITN, FITSYSN, FITGDAT

%   Copyright (c) 1991-96 by MUSYN Inc. and The MathWorks

%   P.M. Young - February 1997
%   GJB 28aug08 - modified to use RCT commands
if nargin < 7
 disp('usage: [icn,tbnd,dl,dr,gm,gr,gfc] = msfbtchv4(ic,bnds,muinfo,sens,blk,clpg,maxord,visflag)')
 return
end

if nargin==6
	maxord = [];
    visflag = [];
elseif nargin==7
    visflag = [];
end

dflag = 0;
if icprev.Ts~=0
    dflag = 1;
end
ddata = muinfo.dvec;
gdata = muinfo.gvec;

if isempty(maxord)
    maxordd = 5;
    maxordg = 6;
elseif size(maxord(:),1)==1
    maxordd = maxord(1);
    maxordg = maxord(1);
else
    maxordd = maxord(1);
    maxordg = maxord(2);
end

if isempty(visflag)
    visflag = 0;
end
if isempty(dflag)
    dflag = 0;
end

% Note we form mixed and associated complex block structure.
% Then perform a few error checks on BLK and also obtain nmeas 
% and ncont from ICPREV and BLK

if isempty(blk)
	error('BLK is empty - nothing to fit!')
end
if length(blk(1,:))~=2 || any(any(abs(round(real(blk))-blk) > 1e-6))
	error('   BLK is invalid')
elseif any(round(real(blk(:,1)))) == 0 || any(any(isnan(blk)))
	error('   BLK is invalid')
else
	blk = round(real(blk));
end

blkr = blk;  blk = abs(blk);

for ii=1:length(blkr(:,1))
    if all( blkr(ii,:) == [ 1 0] )
        blkr(ii,:)  = [ 1 1] ;
    end
    if all( blkr(ii,:) == [-1 1] )
        blkr(ii,:)  = [-1 0] ;
    end
    if all( blk(ii,:) == [ 1 0] )
        blk(ii,:)  = [ 1 1] ;
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

if isa(icprev,'lti') 
    if isa(icprev,'frd')
        error('IC must be a SS object')
    else
        [plantrow,plantcol] = size(icprev);
    end
else
    error('IC must be a SS object')   
end
    
nblk = size(blk,1);
bdim = blk;
rp = find(blk(:,2)==0);
dsize = ones(nblk,1);
if ~isempty(rp)
    bdim(rp,2) = blk(rp,1);
    dsize(rp) = blk(rp,1).*blk(rp,1); 
end
bp = cumsum([1 1;bdim]);
pimp = cumsum([1;dsize]);
pertrow = norm(bdim(:,1),1);
pertcol = norm(bdim(:,2),1);
nmeas = round(plantrow - pertcol);
ncntrl = round(plantcol - pertrow);

if nmeas <0 || ncntrl < 0
	error('IC and BLK are incompatible')
end

% Set up some pointers

sdmax = 0;
for i=nblk:-1:1
   if (sdmax<(blk(i,1)+blk(i,2)))&&(blkr(i,2)~=0)
      sdmaxp = i;
      sdmax = blk(i,1)+blk(i,2);
   end
end

% Now get the old data for D and G.
[drows,dcols] = size(ddata);
omega = ddata.Frequency;
dnum = size(omega,1);
if ~isa(ddata,'frd')
   error('ROWD must be a FRD row vector')
end

% Note we cannot allow omega = zero in the frequency response
% for mixed problems - but we'll fix it later in fitgdat

% omega = getiv(ddata);
% zerotol = 1e-12;
%if any(abs(omega)<=zerotol)
%   if any(blkr(:,1) < 0)
%      error('frequency range too low (cannot go to DC)')
%      return
%   end
%end

blkp = ptrs(blk);
leftdim  = blkp(nblk+1,2) - 1;  
rightdim = blkp(nblk+1,1) - 1; 

% SKIPD will normalize all of the D-matrices by the last complex scalar block 
% that has the largest dimensions, or simply by the last element of 
% the last complex block (if all are full Ds or reals) or the last element 
% of the the last block (if all are reals)

if sdmax == 0
   index = find(blkr(:,1)>0);siz=length(index);
   if siz==0
      skipd=nblk;
   else
      skipd=index(siz);
   end
else
   skipd = sdmaxp;
end

% Now a few more error checks

deesiz = length(find(blk(:,2)~=0));
if any(blk(:,2)==0)
    deesiz = deesiz + round((norm(blk(find(blk(:,2)==0),1)))^2);
end
geesiz = 0;
if any(blkr(:,1)<0)
    geesiz = geesiz + round(norm(blk(find(blkr(:,1)<0),1),1));
end
sensiz = length(blk(:,1));

if (drows~=1) || (dcols~=deesiz)
   error('ROWD is incompatible with BLK')
end
[grows,gcols] = size(gdata);

if ~isa(gdata,'frd')
   error('ROWG must be a FRD row vector')
end
if geesiz == 0
   if ~isempty(gdata)
      error('ROWG is incompatible with BLK')
   end
else
   if (grows~=1) || (gcols~=geesiz)
      error('ROWG is incompatible with BLK')
   end
end
[srows,scols] = size(sens);
if ~isa(sens,'frd')
   error('SENS must be a FRD row vector')
end
if (srows~=1) || (scols~=sensiz)
   error('SENS is incompatible with BLK')
end
[brows,bcols] = size(bnds);
if ~isa(bnds,'frd')
   error('BNDS must be a FRD object')
end
if (brows~=1) || (bcols~=2)
   error('BNDS must be a 1x2 FRD row vector')
end
if ~all(sens.Frequency==ddata.Frequency)
   error('SENS & ROWD have different frequency data')
end
if any(blkr(:,1) < 0)
%   if indvcmp(sens,gdata) ~= 1
   if ~all(sens.Frequency==ddata.Frequency)
      error('SENS & ROWG have different frequency data')
   end
end
if ~all(sens.Frequency==bnds.Frequency)
   error('SENS & BNDS have different frequency data')
end

bmax = norm(bnds(1,1),'inf');
if bmax <= 0
   if any(blkr(:,1)<0)
   	error('Global Upper Bound is not strictly positive')
   end
end

%  Recompute D,G for the synthesis problem

if any(blkr(:,1) < 0)
   if visflag==1
      disp(' ')
      disp(' Mixed problem - need to recompute D,G Scaling:')
   end
   [chekbest,ddata,gdata,sens] = ...
         grmures(clpg,ddata,gdata,sens,blkr,bmax,visflag);
   sens = gsord(sens,blkr);   
else
   chekbest = bnds;
end
[dmatl,dmatr,gmatl,gmatm,gmatr] = mussvunwrap(ddata,gdata,blkr);

% dmatl is now unwrapped, and looks like a LEFT D scaling.  BLKP(i,2) is 
% the correct pointer to the i'th block gmatl is also unwrapped, and also 
% looks like a LEFT G scaling.

%  Now adjust D scaling by (I+G*G)^(-1/4) so scalings use synthesis bound 
dmatldata = dmatl.ResponseData;
dmatrdata = dmatr.ResponseData;

for i=1:dnum  % Number of frequency points
   if (any(blkr(:,1) < 0))
         gvec = diag(gmatl.ResponseData(:,:,i));
         gscal = (ones(leftdim,1)+gvec.*gvec).^(-0.25)*ones(1,leftdim);
         dmatldata(:,:,i) = gscal.*dmatl.ResponseData(:,:,i);
         gvec = diag(gmatr.ResponseData(:,:,i));
         gscal = (ones(rightdim,1)+gvec.*gvec).^(-0.25)*ones(1,rightdim);
         dmatrdata(:,:,i) = gscal.*dmatrdata(:,:,i);
   end
end
dmatl = frd(dmatldata,omega,dmatl.Ts);
dmatr = frd(dmatrdata,omega,dmatr.Ts);

% Convert invertible D to Hermtitan, positive-definite D
% by polar decomposition:  D = U_P*D_P.  Also correct for
% corresponding G scaling as G_NEW = (U_P^*)*G*U_P.
% Polar decomposition from svd: D = U*sig*(V^*)
% and so D_P = V*sig*(V^*) and U_P = U*(V^*).

for i=1:nblk
    lind = bp(i,2):bp(i+1,2)-1;
    rind = bp(i,1):bp(i+1,1)-1;
    if blk(i,2) == 0
        invd = dmatl(lind,lind);
        [uinvd,sinvd,vinvd] = svd(invd);
        hpdd = vinvd*sinvd*vinvd';
        dmatl(lind,lind) = hpdd;
        dmatr(rind,rind) = hpdd;
	    if blkr(i,1) < 0
            invg = gmatl(lind,lind);
            ungg = uinvd*vinvd';
            hpgg = ungg'*invg*ungg;
            gmatl(lind,lind) = hpgg;
            gmatm(lind,rind) = hpgg;
            gmatr(rind,rind) = hpgg;
	    end
    end
end

%normalize through one entry of D
dmatldata = dmatl.ResponseData;
dmatrdata = dmatr.ResponseData;
for i=1:dnum
   fac = 1/dmatldata(blkp(skipd+1,2)-1,blkp(skipd+1,2)-1,i);
   dmatldata(1:leftdim,1:leftdim,i)   = dmatldata(1:leftdim,1:leftdim,i)*fac;
   dmatrdata(1:rightdim,1:rightdim,i) = dmatrdata(1:rightdim,1:rightdim,i)*fac;
end
dmatl = frd(dmatldata,omega,dmatl.Ts);
dmatr = frd(dmatrdata,omega,dmatr.Ts);

% block-by-block GENPHASE - again include correction for G
% as D_NEW - U_DIAG*D_OLD and G_NEW = U_DIAG*G_OLD*(U_DIAG^*)

for i=1:nblk
    lind = bp(i,2):bp(i+1,2)-1;
    rind = bp(i,1):bp(i+1,1)-1;
    if blk(i,2) == 0
        mrows = bdim(i,1);
        hpdd = dmatl(lind,lind);
        if blkr(i,1) < 0
           hpgg = gmatl(lind,lind);
        end
        for ii=1:mrows
            sd = genphase(hpdd(ii,ii));         % pos -> complex
            sdphz = sd/abs(sd); 		        % complex, mag=1
            hpdd(ii,:) = sdphz*hpdd(ii,:);      % scale row
            if blkr(i,1) < 0
               hpgg(ii,:) = sdphz*hpgg(ii,:);	% scale row
               hpgg(:,ii) = hpgg(:,ii)*sdphz';	% scale col
            end
        end
        dmatl(lind,lind) = hpdd;
        dmatr(rind,rind) = hpdd;
        if blkr(i,1) < 0
            gmatl(lind,lind) = hpgg;
            gmatm(lind,rind) = hpgg;
            gmatr(rind,rind) = hpgg;
        end
    else
        hpdd = dmatl(lind(1),lind(1));
        try
            scald = genphase(abs(hpdd));
        catch
            keyboard
        end
        ll = scald*diag(ones(1,blk(i,2)));
        rr = scald*diag(ones(1,blk(i,1)));
        dmatl(lind,lind) = ll;
        dmatr(rind,rind) = rr;
    end
end
dmatlorig = dmatl;
dmatrorig = dmatr;
dconl = dmatlorig;
dconr = dmatrorig;
if any(blkr(:,1)<0)
	gmatlorig = gmatl;
	gmatmorig = gmatm;
	gmatrorig = gmatr;
end 

perctold = 1.03;
fitstatd = zeros(pimp(nblk+1)-1,1);
fitvecd = (-1)*ones(pimp(nblk+1)-1,1);

perctolg = 1.04;
fitstatg = zeros(pimp(nblk+1)-1,1);
fitvecg = (-1)*ones(pimp(nblk+1)-1,1);

% Need to correctly initialize syscelld  with ALL identity blocks
% and syscellg (which contains the fit to jG) with all zero blocks.
% Might be able to do without loops but it's not a big deal.
% Also fix fitstatg and fitvecg to mark complex blocks
% as already fit with a zero order G scale.

syscelld = {};
syscellg = {};
for i=1:nblk
    if blk(i,2)==0		  	  % know it's repeated because no [1 0] in blk
        for j=1:blk(i,1)      % icol
            for k=1:blk(i,1)  % irow
				pp = pimp(i) + (j-1)*blk(i,1) + (k-1);
                if j==k
                    syscelld{pp} = 1;
                else
                    syscelld{pp} = 0;
                end
                syscellg{pp} = 0;
                if blkr(i,1)>0
                    fitstatg(pp) = 1;		% mark the G scales for complex
                    fitvecg(pp) = 0;		% blocks as already fit
                end
            end
        end
	else
		j = 1;
		k = 1;
		pp = pimp(i) + (j-1)*blk(i,1) + (k-1);
		syscelld{pp} = 1;
		syscellg{pp} = ss(0);
        if blkr(i,1)>0
            fitstatg(pp) = 1;		% mark the G scales for complex
            fitvecg(pp) = 0;		% blocks as already fit
        end
    end
end

if any(blkr(:,1)<0)
    [syscelld,fitstatd,fitvecd,currentdl,currentdr] = ...
        gmudkafd(clpg,chekbest,dmatl,dmatr,gmatlorig,gmatmorig,gmatrorig,...
                 bmax,sens,blkr,maxordd,perctold,visflag);
else
   [syscelld,fitstatd,fitvecd,currentdl,currentdr] = ...
        gmudkafd(clpg,bnds,dmatl,dmatr,[],[],[],bmax,sens,blk,maxordd,...
                 perctold,visflag);
end

% Now reconstruct D scalings and factor them IF they have changed
% by looking at the values in fitvecd (same as fitstatd but stores order).
% Also need to change both G scaling data AND any existing fits for any
% repeated real parameter blocks where the full D scales have changed.

dsysl = [];
dsysr = [];
for i=1:nblk
       lind = bp(i,2):bp(i+1,2)-1;
       rind = bp(i,1):bp(i+1,1)-1;
       if blk(i,2) == 0
           dblk = [];
           for j=1:blk(i,1)    % col
               syscol = [];
               for k=1:blk(i,1)
                   pp = pimp(i) + (j-1)*blk(i,1) + (k-1);
                   syscol = [syscol;syscelld{pp}]; 
%                    if j==k
%                       sysdiag = blkdiag(sysdiag,syscell{pp});
%                    end
               end
               dblk = [dblk,syscol];
           end
           if size(dblk.a,1) > 0
                %smpdblkg = frd(smpdblk,currentdl.Frequency);
                %dsysl = blkdiag(dsysl,smpdblk);
                %dsysr = blkdiag(dsysr,smpdblk);
                if dflag~=0
                   dblktmp = d2cbil(dblk);
                   [smpdblktmp,ublktmp] = gfactord(dblktmp);
                   smpdblk = c2dbil(smpdblktmp);
                   ublk = c2dbil(ublktmp);
                else
                   [smpdblk,ublk] = gfactord(dblk);
                end
           else
               smpdblk = dblk;
               ublk = eye(size(dblk,1));
           end
           dsysl = blkdiag(dsysl,smpdblk);
           dsysr = blkdiag(dsysr,smpdblk);
           if blkr(i,1) < 0
              % All [1 0] blocks in blk changed to [1 1] so if 
              % here the SCALAR block MUST be REPEATED.  This is
              % assumed by this part of the code
              hpgg = gmatlorig(lind,lind);
              hpdd = dmatlorig(lind,lind);
              if isa(ublk,'lti')
                  %ublkg = frd(ublk,omega,dflag);
                  ublkg = frd(ublk,omega);
              else
                 ublkg = ublk;
              end
              hpgg = ublkg*hpgg*ublkg';  	% fix the G data
              gmatl(lind,lind) = hpgg;
              gmatm(lind,rind) = hpgg;
              gmatr(rind,rind) = hpgg;
              hpdd = ublkg*hpdd;  		    % fix the D data
              dconl(lind,lind) = hpdd;
              dconr(rind,rind) = hpdd;
           end
       else
            dblk = [];
            sys = syscelld{pimp(i)};
            if ~isempty(sys.a)
               if dflag~=0
                   smpsys = c2dbil(gfactord(d2cbil(sys)));
               else
                   smpsys = gfactord(sys);
               end
            else
               smpsys = sys;
            end
            for l=1:min(blk(i,:))
                dblk = blkdiag(dblk,smpsys);
            end
            if blk(i,1)<blk(i,2)
                dsysr = blkdiag(dsysr,dblk);
                for l=1:blk(i,2)-blk(i,1)
                    dblk = blkdiag(dblk,smpsys);
                end
                dsysl = blkdiag(dsysl,dblk);
            elseif blk(i,1)>blk(i,2)
                dsysl = blkdiag(dsysl,dblk);
                for l=1:blk(i,1)-blk(i,2)
                    dblk = blkdiag(dblk,smpsys);
                end
                dsysr = blkdiag(dsysr,dblk);
            else
                dsysl = blkdiag(dsysl,dblk);
                dsysr = blkdiag(dsysr,dblk);
            end
       end
end % for i = 1:nblk
pertrow=round(pertrow);  %sajjad
gsysm = zeros(pertcol,pertrow);
gsysr = eye(pertrow);
gsysfc = zeros(pertcol,pertrow);

if any(blkr(:,1)<0)
    [syscellg,fitstatg,fitvecg,currentgl,currentgm,currentgr] = ...
        gmudkafg(clpg,chekbest,dconl,dconr,gmatl,gmatm,gmatr,bmax,sens,blkr,...
                maxordg,perctolg,syscellg,fitstatg,fitvecg,visflag);
    % Now reconstruct G scalings and factor them. 
    gsysm = [];		% fit to jG				    (OLD - gcen)
    gsysr = [];		% fit to (I+G^2)^(-1/2)		(OLD - gright)
    gsysfc = [];    % fit to jG(I+G^2)^(-1/2)	(OLD - gmid)
    for i=1:nblk
       if blk(i,2) == 0
           gmblk = [];
           for j=1:blk(i,1)    % col
               syscol = [];
               for k=1:blk(i,1)
                   pp = pimp(i) + (j-1)*blk(i,1) + (k-1);
                   syscol = [syscol;syscellg{pp}];
               end
               gmblk = [gmblk syscol];
           end
           if dflag~=0
               gmblktmp = d2cbil(gmblk);
               [grblktmp,gfcblktmp] = gfactorg(gmblktmp);
               grblk = c2dbil(grblktmp);
               gfcblk = c2dbil(gfcblktmp);
           else
             [grblk,gfcblk] = gfactorg(ss(gmblk));
           end
           gsysm  = blkdiag(gsysm,gmblk);
           gsysr  = blkdiag(gsysr,grblk);
           gsysfc = blkdiag(gsysfc,gfcblk);
       else
           sysgm = syscellg{pimp(i)}; % should be a scalar
           if dflag~=0
%                sysgmtmp = d2cbil(sysgm);
%                [sysgrtmp,sysgfctmp] = gfactorg(sysgmtmp);
%                sysgr = c2dbil(sysgrtmp);
%                sysgfc = c2dbil(sysgfctmp);
               sysgmtmp = d2cbil(sysgm);
               [sysgrtmp,sysgfctmp] = gfactorg(sysgmtmp);
               sysgr = c2dbil(sysgrtmp);
               sysgr.Ts = sysgm.Ts;
               sysgfc = c2dbil(sysgfctmp);
               sysgfc.Ts = sysgm.Ts;
           else
               [sysgr,sysgfc] = gfactorg(sysgm);
           end
           if blkr(i,1) < 0
              % We do NOT allow REAL FULL blocks.  If you are here
              % then the full block is real, so it MUST be a SCALAR.  
              % This is assumed by this part of the code
               gsysm = blkdiag(gsysm,sysgm);
               gsysr = blkdiag(gsysr,sysgr);
               gsysfc = blkdiag(gsysfc,sysgfc);
           else
              % This is a NON scalar full block so it MUST be complex
              % This is assumed by this part of the code
               gsysm = blkdiag(gsysm,zeros(blk(i,2),blk(i,1)));
               gsysr = blkdiag(gsysr,eye(blk(i,1)));
               gsysfc = blkdiag(gsysfc,zeros(blk(i,2),blk(i,1)));
           end
       end
    end % for i = 1:nblk
end

% ALL scaling fitting/building is done. It remains to form the interconnection.
% First add IDENTITIES for measurements and controls

dsysl = blkdiag(dsysl,eye(nmeas));
dsysr = blkdiag(dsysr,eye(ncntrl));

%if any(blkr(:,1)<0)
%   gsysm = blkdiag(gsysm,zeros(nmeas,ncntrl));
%   gsysr = blkdiag(gsysr,eye(ncntrl));
%   gsysfc = blkdiag(gsysfc,zeros(nmeas,ncntrl));
%end
gsysm = blkdiag(gsysm,zeros(nmeas,ncntrl));
gsysr = blkdiag(gsysr,eye(ncntrl));
gsysfc = blkdiag(gsysfc,zeros(nmeas,ncntrl));

% Now form the interconnection - using stable factorizations
% and employing state cancellations

icd = dsysl*icprev*inv(dsysr);   

if any(blkr(:,1) < 0)
%   levin = 1.02*bmax;
%   icnew = icd*gsysr - levin*gsysfc;
%   incnew.Ts = icprev.Ts;
    [ag,bg,cg,dg] = ssdata(gsysfc);
    [dum1,dum2,ch,dh] = ssdata(gsysr);
    [ad,bd,cd,dd] = ssdata(icd);
    if isempty(ag)
        [rowdg,coldg] = size(dg);
        ag = [];
        bg = zeros(0,coldg);
        cg = zeros(rowdg,0);
        rowdh = size(dh,1);
        ch = zeros(rowdh,0);
    end
    if isempty(ad)
        [rowdd,coldd] = size(dd);
        ad = [];
        bd = zeros(0,coldd);
        cd = zeros(rowdd,0);
    end
   zz = zeros(length(ag),length(ad));
   anew = [ad bd*ch;zz ag];
   bnew = [bd*dh;bg];
   levin = 1.02*bmax;
   cgt = levin*(-1)*cg;
   dgt = levin*(-1)*dg;
   cnew = [cd dd*ch+cgt];
   dnew = dd*dh+dgt;
   if isempty(anew)
      icnew = ss(dnew,icd.Ts);
   else
      icnew = ss(anew,bnew,cnew,dnew);
      icnew.Ts = icd.Ts;
   end
else
   icnew = icd;  % complex mu problem
end
