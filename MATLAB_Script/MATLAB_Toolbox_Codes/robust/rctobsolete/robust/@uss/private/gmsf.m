function [icnew,chekbest,dsysl,dsysr,gsysm,gsysr,gsysfc] = ...
            gmsf(icprev,bnds,muinfo,sens,blk,clpg,maxord,diagnos)
% function [icn,tbnd,dl,dr,gm,gr,gfc] = gmsf(ic,bnds,rowd,rowg,sens,blk,clpg,maxord,diagnos)
%
% Fits frequency response data with system matrices.
% Assumes the mixed mu upper bound from MUN has been
% run to generate the data ROWD, ROWG, BNDS, and SENS.
% Returns fits to the D scaling in DL and DR, and fits
% to the G scaling in GM, GR and GFC.  In addition ICN
% contains the new interconnection formed from the above
% systems for the next iteration of H_infinity design in
% a mixed mu synthesis procedure.
%
% See Also DKITN, FITSYSN, FITGDAT

% P. M. Young, December, 1996.
% GJB Modifed 11 Sept 08

%   Copyright 2009-2018 The MathWorks, Inc.
if nargin < 7
   disp('usage: [icn,tbnd,dl,dr,gm,gr,gfc] = gmsf(ic,bnds,muinfo,sens,blk,clpg,maxord,diagnos)')
   return
end
if nargin == 7
    diagnos = [];
end
 
if isempty(diagnos)
    diagnos = 0;
end

dflag = 0;
if icprev.Ts~=0
    dflag = 1;
end
perctold = 1.03;
ddata = muinfo.dvec;
gdata = muinfo.gvec;
if size(maxord(:),2)==1
    maxordd = maxord(1);
    maxordg = maxord(1);
else
    maxordd = maxord(1);
    maxordg = maxord(2);
end

warnstate = warning;  % added to get rid of axis warnings - pmy
warning off

displogflag = 0;  % edit to enable log scaling in display for discrete-time

lmagax = 'liv,lm';
magax = 'liv,m';
dax = 'liv,d';
phax = 'liv,p';
if dflag~=0
    if displogflag==0
      lmagax = 'iv,lm';
      magax = 'iv,m';
      dax = 'iv,d';
      phax = 'iv,p';
    else
      lmagax = 'liv,lm';
      magax = 'liv,m';
      dax = 'liv,d';
      phax = 'liv,p'; 
    end
end

% Note we form mixed and associated complex block structure.
% Then perform a few error checks on BLK and also obtain nmeas 
% and ncont from ICPREV and BLK

if isempty(blk)
	error('BLK is empty - nothing to fit!')
end
if ~(isreal(blk) && size(blk,2)==2 && norm(rem(blk,1),1)==0 && all(blk(:,1)>0))
   error('   BLK is invalid')
end

blkr = blk;  
blk = abs(blk);

for ii=1:length(blkr(:,1))
    if all( blkr(ii,:) == [ 1 0] )
        blkr(ii,:)  = [ 1 1];
    end
    if all( blkr(ii,:) == [-1 1] )
        blkr(ii,:)  = [-1 0];
    end
    if all( blk(ii,:) == [ 1 0] )
        blk(ii,:)  = [ 1 1] ;
    end
end

if any(blk(:,2)<0)
	error('   BLK is invalid');
end
if any(blk(:,2)~=0&blk(:,1)<0)
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
   if (sdmax<(blk(i,1)+blk(i,2))) && (blkr(i,2)~=0)
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
% for mixed problems - but we'll fix it later in gfitgdat

%omega = getiv(ddata);
%zerotol = 1e-12;
%if any(abs(omega)<=zerotol)
%   if any(blkr(:,1) < 0)
%      error('frequency range too low (cannot go to DC)')
%      return
%   end
%end

blkp = ptrs(blk);
leftdim = blkp(nblk+1,2) - 1;  % check this ??
rightdim = blkp(nblk+1,1) - 1;  % check this ??

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
    deesiz = deesiz + round((norm(blk(blk(:,2)==0,1)))^2);
end
geesiz = 0;
if any(blkr(:,1)<0)
    geesiz = geesiz + round(norm(blk(blkr(:,1)<0,1),1));
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
   disp(' ')
   disp(' Mixed problem - need to recompute D,G Scaling:')
   [chekbest,ddata,gdata,sens] = ...
         grmures(clpg,ddata,gdata,sens,blkr,bmax);
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

%  Now adjust D scaling by (I+G*G)^(-1/4)
%  so scalings use synthesis bound 

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

% normalize D scales using SKIPD
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
        scald = genphase(abs(hpdd));
        ll = scald*diag(ones(1,blk(i,2)));
        rr = scald*diag(ones(1,blk(i,1)));
        dmatl(lind,lind) = ll;
        dmatr(rind,rind) = rr;
    end
end
dmatlorig = dmatl;
dmatrorig = dmatr;
currentdl = dmatl;
currentdr = dmatr;
dconl = dmatlorig;
dconr = dmatrorig;
if any(blkr(:,1)<0)
	gmatlorig = gmatl;
	gmatmorig = gmatm;
	gmatrorig = gmatr;
	currentgl = gmatl;
	currentgm = gmatm;
	currentgr = gmatr;
end 

tempdmd = currentdl*clpg/currentdr;
if any(blkr(:,1)<0)            % repeated, real scalars
    tempdmdjg = tempdmd - bmax*sqrt(-1)*currentgm;
    tempgfac = eye(pertrow) + currentgr*currentgr;
    cursclmat = tempdmdjg/sqrtm(tempgfac);
else
    cursclmat = tempdmd;
end
cursclbnd = fnorm(cursclmat);

if diagnos == 1
    curfig = gcf;
    diagfig = curfig+1;
    figure(diagfig);
    clf reset
    for i=1:nblk
        si = int2str(i);
        rp = bp(i,2);
        cp = bp(i,1);
        if blk(i,2) == 0 && blk(i,1) > 1
           for j=1:blk(i,1)
              sj = int2str(j);
              for k=1:blk(i,1)
            	sk = int2str(k);
                leftrow = rp + j - 1;
            	leftcol = rp + k - 1;
            	rightrow = cp + j - 1;
            	rightcol = cp + k - 1;
            	lo = dmatlorig(leftrow,leftcol);
            	ln = dmatl(leftrow,leftcol);
            	ro = dmatrorig(rightrow,rightcol);
            	rn = dmatr(rightrow,rightcol);
            	uplot('bode',lo,ln,ro,rn);
            	title(['D Block ' si '[' sj ' ' sk ']']);
            	disp('Paused...');pause;disp('Running...')
                if blkr(i,1) < 0
                    lo = gmatlorig(leftrow,leftcol);
                    ln = gmatl(leftrow,leftcol);
                    mo = gmatmorig(leftrow,rightcol);
                    mn = gmatm(leftrow,rightcol);
                    ro = gmatrorig(rightrow,rightcol);
                    rn = gmatr(rightrow,rightcol);
                    uplot('bode',lo,ln,mo,mn,ro,rn);
                    title(['G Block ' si '[' sj ' ' sk ']']);
                    disp('Paused...');pause;disp('Running...')
                end
              end
           end
        end
    end
    clf
    if any(blkr(:,1)<0)
       uplot(magax,chekbest,cursclbnd)
       title('DIAGNOSIS: Modified Mu Upper Bound and Scaled(GenPhase) Bound')
    else
       uplot(magax,bnds(1,1),cursclbnd)
       title('DIAGNOSIS: Mu Upper Bound and Scaled(GenPhase) Bound')
    end
    disp('Paused...');pause;disp('Running...')
    figure(curfig);
end

clf reset % somehow messes up without this after subplot commands??
if isempty(rp)
    comparebnds = axes('position',[.1 .72 .85 .23]);
    plotmag = axes('position',[.1 .41 .85 .23]);
    plotsens = axes('position',[.1 .1 .85 .23]); 
else 
    comparebnds = axes('position',[.1 .6 .35 .35]);
    plotmag = axes('position',[.6 .6 .35 .35]);
    plotphase = axes('position',[.6 .1 .35 .35]);
%     plotphasehidden = axes('position',[.6 .1 .35 .35],'visible','off',...
%        'xlim',[0 1],'ylim',[0 1]);
    hiddentext = text(.5,.5,'No Phase');
    set(hiddentext,'visible','off','horizontalalignment','center');
    plotsens = axes('position',[.1 .1 .35 .35]);
end

iblkd = 1;
irowd = 1;
icold = 1;
% ord = -1;
% gostopd = 1;
skipchoiced = 0;
firsttimed = 1;
fitstatd = zeros(pimp(nblk+1)-1,1);
fitvecd = (-1)*ones(pimp(nblk+1)-1,1);
fitvecdold = fitvecd;
m1d = [];
m2d = [];
opd = 'onlydata';

skipg = 0;

perctolg = 1.04;
if any(blkr(:,1)<0)
	firstg = find(blkr(:,1)<0);
	iblkg = firstg(1);
else
	iblkg = 0;
end
irowg = 1;
icolg = 1;
skipchoiceg = 0;
firsttimeg = 1;
fitstatg = zeros(pimp(nblk+1)-1,1);
fitvecg = (-1)*ones(pimp(nblk+1)-1,1);
m1g = [];
m2g = [];
opg = 'onlydata';

% Need to correctly initialize syscelld with ALL identity blocks
% and syscellg (which contains the fit to jG) with all zero blocks.
% Might be able to do without loops but it's not a big deal.
% Also fix fitstatg, fitvecg and fitvecgold to mark complex blocks
% as already fit with a zero order G scale.

syscelld = {};
syscellg = {};
for i=1:nblk
    if blk(i,2)==0		  	  % know it's repeated because no [1 0] in blk
        for j=1:blk(i,1)      % icol
            for k=1:blk(i,1)  % irow
		pp = pimp(i) + (j-1)*blk(i,1) + (k-1);
                if j==k
                    syscelld{pp} = ss(1);
                else
                    syscelld{pp} = ss(0);
                end
                syscellg{pp} = ss(0);
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
		syscelld{pp} = ss(1);
		syscellg{pp} = ss(0);
        if blkr(i,1)>0
            fitstatg(pp) = 1;		% mark the G scales for complex
            fitvecg(pp) = 0;		% blocks as already fit
        end
    end
end
%fitvecgold = fitvecg;

axes(comparebnds);
uplot(magax,cursclbnd,'--');

% MAIN LOOP STARTS HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

gostopmain = 1;
while gostopmain
  gostopd = 1;
  while gostopd 		% this the the main D fitting loop
    pimpointer = pimp(iblkd) + (icold-1)*blk(iblkd,1) + (irowd-1);
    lfr = bp(iblkd,2) + irowd - 1;
    lfc = bp(iblkd,2) + icold - 1;
    rir = bp(iblkd,1) + irowd - 1;
    ric = bp(iblkd,1) + icold - 1;
    data = dmatl(lfr,lfc);
    wt = sens(1,iblkd);
    if strcmp(opd,'changedisplay')
        disp(m1d);
        sys = syscelld{pimpointer};
        curord = size(sys.a,1);
        sysg = frd(sys,omega);
        if curord >= 0
            tt = ['D Block[' int2str(iblkd) '], Element[' int2str(irowd) ' ' int2str(icold) ...
                    '], Order[' int2str(curord) ']'];
        else
            tt = ['(' int2str(iblkd) ';' int2str(irowd) ' ' int2str(icold) ...
                    ':' 'd)'];
        end
        axes(plotmag)
            uplot(lmagax,data,'-',sysg,'--');
            title(['Magnitude Data and Fit: ' tt])
        if blk(iblkd,2)==0 && blk(iblkd,1) > 1 && ~isempty(rp)
            axes(plotphase)
            uplot(phax,data,'-',sysg,'--');
            title('Phase Data and Fit')
            set(plotphase,'visible','on');
            set(hiddentext,'visible','off')
        elseif ~isempty(rp)
            set(plotphase,'visible','off');
            delete(get(plotphase,'children'))
            set(hiddentext,'visible','on')
        end
        axes(plotsens)
        uplot(lmagax,wt);
        title('Sensitivity')
        disp(m2d);
    elseif strcmp(opd,'newfit')
        disp(m1d);
        curord = ord;
        sys = fitfrd(data,curord,[],wt,1);
        syscelld{pimpointer} = sys;
        fitstatd(pimpointer) = 1;
        fitvecd(pimpointer) = curord;
        sysg = frd(sys,omega);
        if blk(iblkd,2) == 0
            currentdl(lfr,lfc) = sysg;
            currentdr(rir,ric) = sysg;
        else
            lind = bp(iblkd,2):bp(iblkd+1,2)-1;
            rind = bp(iblkd,1):bp(iblkd+1,1)-1;
            ll = diag(sysg*ones(1,blk(iblkd,2)));
            rr = diag(sysg*ones(1,blk(iblkd,1)));
            currentdl(lind,lind) = ll;
            currentdr(rind,rind) = rr;
        end
        tempdmd = currentdl*clpg/currentdr;
        if any(blkr(:,1)<0)            % repeated, real scalars
            tempdmdjg = tempdmd - bmax*sqrt(-1)*currentgm;
            tempgfac = eye(pertrow) + currentgr*currentgr;
            cursclmat = tempdmdjg/sqrtm(tempgfac);
        else
            cursclmat = tempdmd;
        end
        cursclbnd = fnorm(cursclmat);
        axes(comparebnds);
        if any(blkr(:,1)<0)
            uplot(magax,chekbest,'-',cursclbnd,'--');
            title('Scaled Bound and Modified MU')
        else
            uplot(magax,bnds(1,1),'-',cursclbnd,'--');
            title('Scaled Bound and MU')
        end
        axes(plotmag)
        uplot(lmagax,data,'-',sysg,'--');
        tt = ['(' int2str(iblkd) ';' int2str(irowd) ' ' int2str(icold) ...
                    ':' int2str(curord) ')'];
        title(['Mag Data and Fit: ' tt])
        if blk(iblkd,2)==0 && blk(iblkd,1) > 1 && ~isempty(rp)
            axes(plotphase)
            uplot(phax,data,'-',sysg,'--');
            title('Phase Data and Fit')
            set(plotphase,'visible','on');
            set(hiddentext,'visible','off')
        elseif ~isempty(rp)
            set(plotphase,'visible','off');
            delete(get(plotphase,'children'))
            set(hiddentext,'visible','on')
        end
        disp(m2d);
    elseif strcmp(opd,'status')
        disp(m1d)
        for i=1:nblk
            si = int2str(i);
            if blk(i,2)==0
                disp(['D Block ' si ':']);
                for j=1:blk(i,1)
                    str = [];
                    sj = int2str(j);
                    for k=1:blk(i,1)
                        sk = int2str(k);
                        pp=pimp(i)+(j-1)*blk(i,1)+(k-1);
                        ind_d = syscelld{pp};
                        if fitstatd(pp)==0
                            os = 'data';
                        else
                            os=int2str(size(ind_d.a,1));
                        end
                        str=[str '  (' sj ',' sk ')=' os ';'];
                    end
                    disp(['      ' str]);
                end
            else
                pp=pimp(i);
                ind_d = syscelld{pp};
                if fitstatd(pp)==0
                    os = 'data';
                else
                    os = int2str(size(ind_d.a,1));
                end
                disp(['D Block ' si ': ' os]);
            end
        end
        disp(['AutoPrefit Fit Tolerance: ' num2str(perctold)]);
        disp(['AutoPrefit Maximum Order: ' int2str(maxordd)]);
        disp(m2d)
    elseif strcmp(opd,'onlydata')
        disp(m1d);
        curord = -1;
        tt = ['(' int2str(iblkd) ';' int2str(irowd) ' ' int2str(icold) ...
            ':' 'd)'];
        if firsttimed == 1
            firsttimed = 0;
            axes(comparebnds);
            if any(blkr(:,1)<0)
            	uplot(magax,chekbest,'-');
                title('Modified MU Upper Bound')
            else
            	uplot(magax,bnds(1,1),'-');
                title('MU Upper Bound')
            end
        end
        axes(plotmag)
        uplot(lmagax,data,'-');
        title(['Magnitude Data: ' tt])
        if blk(iblkd,2)==0 && blk(iblkd,1) > 1 && ~isempty(rp)
            axes(plotphase)
            uplot(phax,data,'-');
            title('Phase Data')
            set(plotphase,'visible','on');
            set(hiddentext,'visible','off')
        elseif ~isempty(rp)
            set(plotphase,'visible','off');
            delete(get(plotphase,'children'))
            set(hiddentext,'visible','on')
        end
        axes(plotsens)
        uplot(lmagax,wt);
        title('Sensitivity')
        disp(m2d);
    elseif strcmp(opd,'autofit')
        disp(m1d)
        if any(blkr(:,1)<0)
            [syscelld,fitstatd,fitvecd,currentdl,currentdr] = ...
             gmudkafd(clpg,chekbest,dmatl,dmatr,gmatlorig,gmatmorig,gmatrorig,...
                     bmax,sens,blkr,maxordd,perctold,1);
        else
            [syscelld,fitstatd,fitvecd,currentdl,currentdr] = ...
             gmudkafd(clpg,bnds,dmatl,dmatr,[],[],[],bmax,sens,blk,maxordd,...
                     perctold,1);
        end
        disp(m2d)
        skipchoiced = 1;
    elseif strcmp(opd,'hold')
        disp(m1d)
        disp(m2d)
    elseif strcmp(opd,'tryagain')
        disp(m1d);
    else
        error('Message not found')
    end
    if ~skipchoiced
        c = input('Enter Choice (return for list): ','s');
        if isempty(c)
            c = 'ch';
        end
        [gostopd,skipg,opd,iblkd,ord,irowd,icold,perctold,maxordd,m1d,m2d,m1g,m2g] = ...
                    gfitchosd(c,blk,blkr,iblkd,irowd,icold,fitstatd,fitstatg,pimp,curord,...
                perctold,maxordd); 
    else
        skipchoiced = 0;
        opd = 'changedisplay';
        gostopd = 1;
        m1d = [];
        m2d = [];
        tempdmd = currentdl*clpg/currentdr;
        if any(blkr(:,1)<0)            % repeated, real scalars
            tempdmdjg = tempdmd - bmax*sqrt(-1)*currentgm;
            tempgfac = eye(pertrow) + currentgr*currentgr;
            tempgscal = inv(sqrtm(tempgfac));
            cursclmat = tempdmdjg*tempgscal;
        else
            cursclmat = tempdmd;
        end
        cursclbnd = fnorm(cursclmat);
        axes(comparebnds);
        if any(blkr(:,1)<0)
           uplot(magax,chekbest,'-',cursclbnd,'--');
           title('Scaled Bound and Modified MU')
        else
           uplot(magax,bnds(1,1),'-',cursclbnd,'--');
           title('Scaled Bound and MU')
        end
    end
  end % while gostopd

% Now reconstruct D scalings and factor them IF they have changed
% by looking at the values in fitvecd (same as fitstatd but stores order).
% Also need to change both G scaling data AND any existing fits for any
% repeated real parameter blocks where the full D scales have changed.

  dsysl = [];
  dsysr = [];
  anychangeflag = 0;
  for i=1:nblk
       lind = bp(i,2):bp(i+1,2)-1;
       rind = bp(i,1):bp(i+1,1)-1;
       ls = bp(i,2);
       rs = bp(i,1);
       if blk(i,2) == 0
           le = bp(i+1,2)-1;
           re = bp(i+1,1)-1;
           dblk = [];
           changeflag = 0;
           for j=1:blk(i,1)    % col
               syscol = [];
               for k=1:blk(i,1)
                   pp = pimp(i) + (j-1)*blk(i,1) + (k-1);
                   if fitvecd(pp) ~= fitvecdold(pp)
                       changeflag = 1;
                       fitvecdold(pp) = fitvecd(pp);
                   end
                   syscol = [syscol;syscelld{pp}];
               end
               dblk = [dblk syscol];
           end
           if size(dblk.a,1) > 0
               if dflag
                   dblktmp = d2cbil(dblk);
                   [smpdblktmp,ublktmp] = gfactord(dblktmp);
                   smpdblk = c2dbil(smpdblktmp);
                   ublk = c2dbil(ublktmp);
                else
                   [smpdblk,ublk] = gfactord(dblk);
               end
           	   if diagnos == 1
                   disp('D~D-Ds~Ds')
                   dd1 = smpdblk'*smpdblk;
                   dd2 = dblk'*dblk;
                   if size(dblk.a,1) < 12  % XXX
                       norm(dd1-dd2,'inf',.00001)
                   else
                       dd1g = frd(dd1,omega);
                       dd2g = frd(dd2,omega);
                       norm(dd1g-dd2g,'inf')
                   end
                   [~,hsv] = balreal(smpdblk);
                   disp('Hankel Singular Values')
                   hsv
                   disp('Paused...');pause;disp('Running...')
               end
           else
               smpdblk = dblk;
               [deerowd,~] = size(dblk);
               ublk = eye(deerowd);
           end
           if diagnos == 1
           	   smpdblkg = frd(smpdblk,omega);
               figure(diagfig);
               clf reset
               dll = currentdl(ls:le,ls:le);
               uplot(magax,svd(dll/smpdblkg))
               title(['DIAGNOSIS: Left SV: D Block ' int2str(i)])
               disp('Paused...')
               pause
               disp('Running...')
               drr = currentdr(rs:re,rs:re);
               uplot(magax,svd(drr/smpdblkg))
               title(['DIAGNOSIS: Right SV: D Block ' int2str(i)])
               disp('Paused...');pause;disp('Running...')
               figure(curfig);
           end
           dsysl = blkdiag(dsysl,smpdblk);
           dsysr = blkdiag(dsysr,smpdblk);
           if changeflag == 1 
               if blkr(i,1) < 0
                  % All [1 0] blocks in blk changed to [1 1] so if 
                  % here the SCALAR block MUST be REPEATED.  This is
                  % assumed by this part of the code
                  hpgg = gmatlorig(lind,lind);
                  hpdd = dmatlorig(lind,lind);
                  if size(ublk.a,1) > 0
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

                  for j=1:blk(i,1)    % col
                     for k=1:j	     % row has to be <= col
                        pp  = pimp(i) + (j-1)*blk(i,1) + (k-1);
                        if fitstatg(pp)==1 % fit exists so we do need to redo it
                           anychangeflag = 1;
                           ppc = pimp(i) + (k-1)*blk(i,1) + (j-1); % points to transpose 
                           lfr = bp(i,2) + k - 1;
                           lfc = bp(i,2) + j - 1;
                           rir = bp(i,1) + k - 1;
                           ric = bp(i,1) + j - 1;
                           lfrc = bp(i,2) + j - 1;  % pointers to transpose for easy reference
                           lfcc = bp(i,2) + k - 1;
                           rirc = bp(i,1) + j - 1;
                           ricc = bp(i,1) + k - 1;
                           data = gmatl(lfr,lfc);
                           wt = sens(1,i);
                           ord = fitvecg(pp);
                           if k~=j
                               sys = fitfrd(sqrt(-1)*data,ord,wt); % pass jG to fit
                           else
                               %%% XXX need to modify FITGDAT
                               sys = gfitgdat(data,ord,wt);	% pass G (real) but returns fit to jG
                           end
                           syscellg{pp} = sys;
                           if k~=j  % form gji as gji = -gij~ (since sys fits jG)
                               [atmp,btmp,ctmp,dtmp] = ssdata(sys);
                               if isempty(atmp)
                                   sysc = ss(-dtmp',sys.Ts);
                               else
                                   sysc = ss(-atmp',ctmp',btmp',-dtmp',sys.Ts);
                               end
                               syscellg{ppc} = sysc;
                           end
                           sysg = (-1)*sqrt(-1)*frd(sys,omega); % note sys is a fit to jG
                           if k~=j
                               currentgl(lfr,lfc) = sysg;
                               currentgm(lfr,ric) = sysg;
                               currentgr(rir,ric) = sysg;
                               currentgl(lfrc,lfcc) = sysg';  % assign hermitian conjugate
                               currentgm(lfrc,ricc) = sysg';
                               currentgr(rirc,ricc) = sysg';
                           else
                               currentgl(lfr,lfc) = real(sysg);  % diagonal
                               currentgm(lfr,ric) = real(sysg);
                               currentgr(rir,ric) = real(sysg);
                           end
                        end
                     end
                  end
               end
           end
       else
           dblk = [];
           sys = syscelld{pimp(i)};
           if ~isempty(sys.a)
                if dflag
                   dblktmp = d2cbil(sys);
                   [smpdblktmp] = gfactord(dblktmp);
                   smpsys = c2dbil(smpdblktmp);
                else
                   smpsys = gfactord(sys);
                end
               if diagnos == 1
                   disp('d~d-ds~ds')
                   norm(smpsys'*smpsys - sys'*sys,'inf',.000001)
                   disp('Paused...');pause;disp('Running...')
               end
           else
               smpsys = sys;
           end
           for l=1:min(blk(i,:))
               dblk = blkdiag(dblk,smpsys);
           end
           if diagnos == 1
               smpsysg = frd(smpsys,omega);
               figure(diagfig);
               clf reset
               dll = currentdl(ls,ls);
               uplot(magax,abs(dll/smpsysg))
               title(['DIAGNOSIS: Left Abs: D Block ' int2str(i)])
               disp('Paused...')
               pause
               disp('Running...')
               drr = currentdr(rs,rs);
               uplot(magax,abs(drr/smpsysg))
               title(['DIAGNOSIS: Right Abs: D Block ' int2str(i)])
               disp('Paused...');pause;disp('Running...')
               figure(curfig);
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
  if diagnos == 1
       dsyslg = frd(dsysl,omega);
       figure(diagfig);
       clf reset
       uplot(magax,svd(currentdl/dsyslg))
       title('DIAGNOSIS: Left SV: Full')
       disp('Paused...');pause;disp('Running...')
       figure(curfig);
       disp('POLES of DSYSL and DSYSR')
       damp(pole(dsysl))
       damp(pole(dsysr))
       disp('Paused...');pause;disp('Running...')
       disp('POLES of minv(DSYSL) and minv(DSYSR)')
       damp(pole(inv(dsysl)))
       damp(pole(inv(dsysr)))
       disp('Paused...');pause;disp('Running and returning...')
  end
  if anychangeflag==1  % G scalings had to be refit so compute and plot new bound
      tempdmd = currentdl*clpg/currentdr;
      if any(blkr(:,1)<0)            % repeated, real scalars
          tempdmdjg = tempdmd - bmax*sqrt(-1)*currentgm;
          tempgfac = eye(pertrow) + currentgr*currentgr;
          cursclmat = tempdmdjg/sqrtm(tempgfac);
      else
          cursclmat = tempdmd;
      end
      cursclbnd = fnorm(cursclmat);
      axes(comparebnds);
      if any(blkr(:,1)<0)
            uplot(magax,chekbest,'-',cursclbnd,'--');
            title('Scaled Bound and Modified MU')
      else
            uplot(magax,bnds(1,1),'-',cursclbnd,'--');
            title('Scaled Bound and MU')
      end
      if skipg==1
         rego_dk = 1;
         while rego_dk
            x_dk = input('    Full G-scales have changed - exit anyway? (y/n) ','s');
            if strcmp(x_dk,'y') || strcmp(x_dk,'''y''')
               rego_dk = 0;
            elseif strcmp(x_dk,'n') || strcmp(x_dk,'''n''')
               skipg = 0;
               rego_dk = 0;
            else
               disp('Please use y or n')
            end
         end
      end
  end
  if (all(blkr(:,1)>0)) || (skipg==1)
    gostopmain = 0;
  else
    gostopg = 1;
    while gostopg 		% this the the main G fitting loop
        pimpointer =  pimp(iblkg) + (icolg-1)*blk(iblkg,1) + (irowg-1);
        pimpointerc = pimp(iblkg) + (irowg-1)*blk(iblkg,1) + (icolg-1); % points to transpose 
        lfr = bp(iblkg,2) + irowg - 1;
        lfc = bp(iblkg,2) + icolg - 1;
        rir = bp(iblkg,1) + irowg - 1;
        ric = bp(iblkg,1) + icolg - 1;
        lfrc = bp(iblkg,2) + icolg - 1;  % pointers to transpose for easy reference
        lfcc = bp(iblkg,2) + irowg - 1;
        rirc = bp(iblkg,1) + icolg - 1;
        ricc = bp(iblkg,1) + irowg - 1;
        data = gmatl(lfr,lfc);
        wt = sens(1,iblkg);
        if strcmp(opg,'changedisplay')
            disp(m1g);
            sys = syscellg{pimpointer};
            curord = size(sys.a,1);
            sysg = (-1)*sqrt(-1)*frd(sys,omega);  % our system is a fit for jG
            if curord >= 0
                tt = ['G Block[' int2str(iblkg) '], Element[' int2str(irowg) ' ' int2str(icolg) ...
                        '], Order[' int2str(curord) ']'];
            else
                tt = ['(' int2str(iblkg) ';' int2str(irowg) ' ' int2str(icolg) ...
                        ':' 'g)'];
            end
            axes(plotmag)
            if irowg~=icolg
                uplot(lmagax,data,'-',sysg,'--');
                title(['Magnitude Data and Fit: ' tt])
            else
                uplot(dax,real(data),'-',real(sysg),'--');
                title(['Data and Fit: ' tt])
            end
            if (irowg~=icolg) && ~isempty(rp)
                axes(plotphase)
                uplot(phax,data,'-',sysg,'--');
                title('Phase Data and Fit')
                set(plotphase,'visible','on');
                set(hiddentext,'visible','off')
            elseif ~isempty(rp)
                set(plotphase,'visible','off');
                delete(get(plotphase,'children'))
                set(hiddentext,'visible','on')
            end
            axes(plotsens)
            uplot(lmagax,wt);
            title('Sensitivity')
            disp(m2g);
        elseif strcmp(opg,'newfit')
            disp(m1g);
            if irowg~=icolg
                curord = ord;
                sys = fitfrd(sqrt(-1)*data,curord,wt); % pass jG to fit
            else
                curord = 2*floor(ord/2);
                %%% XXX FIX FITGDAT
                sys = gfitgdat(data,curord,wt);% pass G (real) but returns fit to jG
            end
            syscellg{pimpointer} = sys;
            fitstatg(pimpointer) = 1;
            fitvecg(pimpointer) = curord;
            if irowg~=icolg  % form gji as gji = -gij~ (since sys fits jG)
            	[atmp,btmp,ctmp,dtmp] = ssdata(sys);
            	if isempty(atmp)
               		sysc = ss(-dtmp,sys.Ts)';
            	else
               		sysc = ss(-atmp',ctmp',btmp',-dtmp',sys.Ts);
            	end
            	syscellg{pimpointerc} = sysc;
            	fitstatg(pimpointerc) = 1;
                fitvecg(pimpointerc) = curord;
            end
            sysg = (-1)*sqrt(-1)*frd(sys,omega); % note sys is a fit to jG
            if blk(iblkg,2) == 0  % has to be repeated because no [1 0] in blk
                if irowg~=icolg
                   currentgl(lfr,lfc) = sysg;
                   currentgm(lfr,ric) = sysg;
                   currentgr(rir,ric) = sysg;
                   currentgl(lfrc,lfcc) = sysg';  % assign hermitian conjugate
                   currentgm(lfrc,ricc) = sysg';
                   currentgr(rirc,ricc) = sysg';
               else
                   currentgl(lfr,lfc) = real(sysg);  % diagonal
                   currentgm(lfr,ric) = real(sysg);
                   currentgr(rir,ric) = real(sysg);
                end
            else
                lind = bp(iblkg,2):bp(iblkg+1,2)-1;  % should only ever be scalar here 
                rind = bp(iblkg,1):bp(iblkg+1,1)-1;  % because no real full blocks allowed
                ll = real(blkdiag(sysg*ones(1,blk(iblkg,2))));
                rr = real(blkdiag(sysg*ones(1,blk(iblkg,1))));
                currentgl(lind,lind) = ll;
                currentgm(lind,rind) = ll;
                currentgr(rind,rind) = rr;
            end
            tempdmd = currentdl*clpg/currentdr;
            if any(blkr(:,1)<0)            % repeated, real scalars
                tempdmdjg = tempdmd - bmax*sqrt(-1)*currentgm;
                tempgfac = eye(pertrow) + currentgr*currentgr;
                tempgscal = inv(sqrtm(tempgfac));
                cursclmat = tempdmdjg*tempgscal;
            else
                cursclmat = tempdmd;
            end
            cursclbnd = fnorm(cursclmat);
     	    axes(comparebnds);
          if any(blkr(:,1)<0)
             uplot(magax,chekbest,'-',cursclbnd,'--');
             title('Scaled Bound and Modified MU')
          else
             uplot(magax,bnds(1,1),'-',cursclbnd,'--');
             title('Scaled Bound and MU')
          end
          axes(plotmag)
          tt = ['(' int2str(iblkg) ';' int2str(irowg) ' ' int2str(icolg) ...
                        ':' int2str(curord) ')'];
		    if irowg~=icolg
                	uplot(lmagax,data,'-',sysg,'--');
                	title(['Mag Data and Fit: ' tt])
		    else
                	uplot(dax,vreal(data),'-',vreal(sysg),'--');
                	title(['Data and Fit: ' tt])
          end
          if (irowg~=icolg) && ~isempty(rp)
             axes(plotphase)
             uplot(phax,data,'-',sysg,'--');
             title('Phase Data and Fit')
             set(plotphase,'visible','on');
             set(hiddentext,'visible','off')
          elseif ~isempty(rp)
             set(plotphase,'visible','off');
             delete(get(plotphase,'children'))
             set(hiddentext,'visible','on')
          end
          disp(m2g);
        elseif strcmp(opg,'status')
            disp(m1g)
            for i=1:nblk
                if blkr(i,1) < 0
                    si = int2str(i);
                    if blk(i,2)==0
                        disp(['G Block ' si ':']);
                        for j=1:blk(i,1)
                            str = [];
                            sj = int2str(j);
                            for k=1:blk(i,1)
                                sk = int2str(k);
                                pp=pimp(i)+(j-1)*blk(i,1)+(k-1);
                                ind_g = syscellg{pp};
                                if fitstatg(pp)==0
                                    os = 'data';
                                else
                                    os=int2str(xnum(ind_g));
                                end
                                str=[str '  (' sj ',' sk ')=' os ';'];
                            end
                            disp(['      ' str]);
                        end
                    else
                        pp=pimp(i);
                        ind_g = syscellg{pp};
                        if fitstatg(pp)==0
                            os = 'data';
                        else
                            os = int2str(xnum(ind_g));
                        end
                        disp(['G Block ' si ': ' os]);
                    end
                end
            end
            disp(['AutoPrefit Fit Tolerance: ' num2str(perctolg)]);
            disp(['AutoPrefit Maximum Order: ' int2str(maxordg)]);
            disp(m2g)
        elseif strcmp(opg,'onlydata')
            disp(m1g);
            curord = -1;
            tt = ['(' int2str(iblkg) ';' int2str(irowg) ' ' int2str(icolg) ...
                ':' 'g)'];
            if firsttimeg == 1
                firsttimeg = 0;  % don't think we need to do anything here - pmy
%                axes(comparebnds);
%	      	if any(blkr(:,1)<0)
%             	    vplot(magax,chekbest,'-');
%            	    title('Modified MU Upper Bound')
%		      else
%            	    vplot(magax,sel(bnds,1,1),'-');
%            	    title('MU Upper Bound')
%		      end
            end
            axes(plotmag)
		    if irowg~=icolg
                	uplot(lmagax,data,'-');
                	title(['Magnitude Data: ' tt])
            else
                uplot(dax,vreal(data))
                title(['Data: ' tt])
		    end
            if (irowg~=icolg) && ~isempty(rp)
                axes(plotphase)
                uplot(phax,data,'-');
                title('Phase Data')
                set(plotphase,'visible','on');
                set(hiddentext,'visible','off')
            elseif ~isempty(rp)
                set(plotphase,'visible','off');
                delete(get(plotphase,'children'))
                set(hiddentext,'visible','on')
            end
            axes(plotsens)
            uplot(lmagax,wt);
            title('Sensitivity')
            disp(m2g);
        elseif strcmp(opg,'autofit')
            disp(m1g)
            [syscellg,fitstatg,fitvecg,currentgl,currentgm,currentgr] = ...
                gmudkafg(clpg,chekbest,dconl,dconr,gmatl,gmatm,gmatr,bmax,...
                    sens,blkr,maxordg,perctolg,syscellg,fitstatg,fitvecg,[]);
            disp(m2g)
            skipchoiceg = 1;
        elseif strcmp(opg,'hold')
            disp(m1g)
            disp(m2g)
        elseif strcmp(opg,'tryagain')
            disp(m1g);
        else
            error('Message not found')
        end
        if ~skipchoiceg
            c = input('Enter Choice (return for list): ','s');
            if isempty(c)
                c = 'ch';
            end
            [gostopg,gostopmain,opg,iblkg,ord,irowg,icolg,perctolg,maxordg,...
                m1g,m2g,m1d,m2d] = ...
                gfitchosg(c,blk,blkr,iblkg,irowg,icolg,fitstatd,fitstatg,...
                            pimp,curord,perctolg,maxordg); 
        else
            skipchoiceg = 0;
            opg = 'changedisplay';
            gostopg = 1;
            m1g = [];
            m2g = [];
            tempdmd = currentdl*clpg/currentdr;
            if any(blkr(:,1)<0)            % repeated, real scalars
                tempdmdjg = tempdmd - bmax*sqrt(-1)*currentgm;
                tempgfac = eye(pertrow) + currentgr*currentgr;
                tempgscal = inv(sqrtm(tempgfac));
                cursclmat = tempdmdjg*tempgscal;
            else
                cursclmat = tempdmd;
            end
            cursclbnd = fnorm(cursclmat);
            axes(comparebnds);
		    if any(blkr(:,1)<0)
            	uplot(magax,chekbest,'-',cursclbnd,'--');
            	title('Scaled Bound and Modified MU')
		    else
            	uplot(magax,bnds(1,1),'-',cursclbnd,'--');
            	title('Scaled Bound and MU')
		    end
        end
    end % while gostopg
  end
end % while gostopmain

% END OF MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Now reconstruct G scalings and factor them.  The D scalings
% are already constructed in the main loop.

gsysm = zeros(pertcol,pertrow);
gsysr = eye(pertrow);
gsysfc = zeros(pertcol,pertrow);

if any(blkr(:,1)<0)
   gsysm = [];		% fit to jG				(OLD - gcen)
   gsysr = [];		% fit to (I+G^2)^(-1/2)		(OLD - gright)
   gsysfc = [];	% fit to jG(I+G^2)^(-1/2)	(OLD - gmid)
   for i=1:nblk
%       lind = bp(i,2):bp(i+1,2)-1;
%       rind = bp(i,1):bp(i+1,1)-1;
      ls = bp(i,2);
      rs = bp(i,1);
      if blk(i,2) == 0
         le = bp(i+1,2)-1;
         re = bp(i+1,1)-1;
         gmblk = [];
         for j=1:blk(i,1)    % col
            syscol = [];
            for k=1:blk(i,1)
               pp = pimp(i) + (j-1)*blk(i,1) + (k-1);
               syscol = [syscol; syscellg{pp}];
            end
            gmblk = [gmblk syscol];
         end
         if dflag
            gmblktmp = d2cbil(gmblk);
            [grblktmp,gfcblktmp] = gfactorg(gmblktmp);
            grblk = d2sbil(grblktmp);
            gfcblk = d2sbil(gfcblktmp);
         else
            [grblk,gfcblk] = gfactorg(gmblk);
         end
         if ~isempty(gmblk.a) && (diagnos == 1) && (blkr(i,1)<0)
            disp('(I+G~G)^(-1) - G_hG_h~')
            dd1 = inv( eye(blk(i,1)) + gmblk'*gmblk );
            dd2 = grblk*grblk';
            if size(gmblk,1) < 12  % XXX
               norm(dd1-dd2,'inf',.00001)
            else
               dd1g = frd(dd1,omega);
               dd2g = frd(dd2,omega);
               norm(dd1g-dd2g,'inf')
            end
            disp('GG_h - (GG_h)_(red)')
            dd1 = gmblk*grblk;
            dd2 = gfcblk;
            if size(gmblk,1)  < 12  % XXX
               norm(dd1-dd2,'inf',.00001)
            else
               dd1g = frd(dd1,omega);
               dd2g = frd(dd2,omega);
               norm(dd1g-dd2g,'inf')
            end
            [~,hsv] = balreal(grblk);  % XXX
            disp('Hankel Singular Values')
            hsv
            disp('Paused...');pause;disp('Running...')
         end
         if (diagnos==1) && (blkr(i,1)<0)
            gmblkg = (-1)*sqrt(-1)*frd(gmblk,omega); % note gmblk is a fit to jG
            figure(diagfig);
            clf reset
            gmm = currentgm(ls:le,rs:re);
            uplot(magax,svd(gmm-gmblkg))
            title(['DIAGNOSIS: SV, SUB: G Block ' int2str(i)])
            disp('Paused...')
            pause
            disp('Running...')
            figure(curfig);
         end
         gsysm = blkdiag(gsysm,gmblk);
         gsysr = blkdiag(gsysr,grblk);
         gsysfc = blkdiag(gsysfc,gfcblk);
      else
         sysgm = syscellg{pimp(i)}; % should be a scalar
         if dflag
            sysgmtmp = d2cbil(sysgm);
            [sysgrtmp,sysgfctmp] = gfactorg(sysgmtmp);
            sysgr = c2dbil(sysgrtmp);
            sysgfc = c2dbil(sysgfctmp);
         else
            [sysgr,sysgfc] = gfactorg(sysgm);
         end
         if ~isempty(sysgm.a) && (diagnos == 1) && (blkr(i,1)<0)
            disp('(1+g~g)^(-1) - g_hg_h~')
            dd1 = inv(1+sysgm'*sysgm);
            dd2 = sysgr*sysgr';
            norm(dd1-dd2,'inf',.000001)
            disp('gg_h - (gg_h)_(red)')
            dd1 = sysgm*sysgr;
            dd2 = sysgfc;
            norm(dd1-dd2,'inf',.000001)
            disp('Paused...');pause;disp('Running...')
         end
         if (diagnos==1) && (blkr(i,1)<0)
            sysgmg = (-1)*sqrt(-1)*frd(sysgm,omega); % note sysgm is a fit to jG
            figure(diagfig);
            clf reset
            gmm = currentgm(ls,rs);
            uplot(magax,abs(gmm-sysgmg))
            title(['DIAGNOSIS: Abs, Sub: G Block ' int2str(i)])
            disp('Paused...')
            pause
            disp('Running...')
            figure(curfig);
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
   if (diagnos==1) && (any(blkr(:,1)<0))
      gsysmg = (-1)*sqrt(-1)*frd(gsysm,omega); % note gsysm is a fit to jG
      figure(diagfig);
      clf reset
      uplot(magax,svd(currentgm-gsysmg))
      title('DIAGNOSIS: SV, SUB: Full')
      disp('Paused...');pause;disp('Running...')
      figure(curfig);
      disp('POLES of GSYSM, GSYSR, and GSYSFC')
      damp(pole(gsysm))
      damp(pole(gsysr))
      damp(pole(gsysfc))
      disp('Paused...');pause;disp('Running and returning...')
   end
end
warning(warnstate);  % put warning back to what it should be
% ALL scaling fitting/building is done.
% It remains to form the interconnection.
% First add IDENTITIES for measurements and controls

dsysl = blkdiag(dsysl,eye(nmeas));
dsysr = blkdiag(dsysr,eye(ncntrl));
gsysm = blkdiag(gsysm,zeros(nmeas,ncntrl));
gsysr = blkdiag(gsysr,eye(ncntrl));
gsysfc = blkdiag(gsysfc,zeros(nmeas,ncntrl));

% Now form the interconnection - using stable factorizations
% and employing state cancellations

icd = dsysl*icprev/dsysr;
if any(blkr(:,1) < 0)
   %    levin = 1.02*bmax;
   %    icnew = (icd - levin*gsysfc)*gsysr;
   %    incnew.Ts = icprev.Ts;
   [ag,bg,cg,dg] = ssdata(gsysfc);
   [~,~,ch,dh] = ssdata(gsysr);
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
      icnew = ss(dnew,icprev.Ts);
   else
      icnew = ss(anew,bnew,cnew,dnew,icprev.Ts);
   end
else
   icnew = icd;  % complex mu problem
end
