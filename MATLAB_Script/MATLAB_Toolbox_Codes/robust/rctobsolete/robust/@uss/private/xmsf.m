function [dsysl,dsysr] = xmsf(clpg,bnds,muinfo,sens,blk,diagnos,dflag)
% Copyright 2003-2005 The MathWorks, Inc.


if nargin == 5
   diagnos = [];
   dflag = [];
elseif nargin==6
   dflag = [];
end

if isempty(diagnos)
   diagnos = 0;
end
if isempty(dflag)
   dflag = 0;  % continuous
end
lmagax = 'liv,lm';
magax = 'liv,m';
phax = 'liv,p';
if dflag~=0
   lmagax = 'iv,lm';
   magax = 'iv,m';
   phax = 'iv,p';
end

blk = abs(blk);    % no reals yet
nblk = size(blk,1);
for i=1:nblk
   if blk(i,1)==1 && blk(i,2)==0
      blk(i,2) = 1;
   end
end

bdim = blk;
rp = find(blk(:,2)==0);
dsize = ones(nblk,1);
if ~isempty(rp)
   bdim(rp,2) = blk(rp,1);
   dsize(rp) = blk(rp,1).*blk(rp,1);
end
bp = cumsum([1 1;bdim]);
syscell = {};
pimp = cumsum([1;dsize]);

%bdim(rp,2) = blk(rp,1);
%dsize(rp) = blk(rp,1).*blk(rp,1);
[nblk,~] = size(blk);
[dmatl,dmatr] = mussvunwrap(muinfo);
omega = muinfo.dvec.Frequency;
%convert invertible D to Hermtitan, positive-definite D
for i=1:nblk
   lind = bp(i,2):bp(i+1,2)-1;
   rind = bp(i,1):bp(i+1,1)-1;
   if blk(i,2) == 0
      invd = dmatl(lind,lind);
      hpdd = sqrtm(invd'*invd);
      dmatl(lind,lind) = hpdd;
      dmatr(rind,rind) = hpdd;
   end
end
%normalize through very last entry of D
szd = size(dmatl);
nd = dmatl(szd(1),szd(1));
dmatl = nd\dmatl;
dmatr = nd\dmatr;

% block-by-block GENPHASE
for i=1:nblk
   lind = bp(i,2):bp(i+1,2)-1;
   rind = bp(i,1):bp(i+1,1)-1;
   if blk(i,2) == 0
      mrows = bdim(i,1);
      hpdd = dmatl(lind,lind);
      for j=1:mrows                            %
         sd = genphase(hpdd(j,j));
         sdphz = sd/abs(sd);               % complex, mag=1
         hpdd(j,:) = sdphz*hpdd(j,:);      % scale row
      end
      dmatl(lind,lind) = hpdd;
      dmatr(rind,rind) = hpdd;
   else
      hpdd = dmatl(lind(1),lind(1));
      scald = genphase(hpdd);
      ll = diag( scald*ones(1,blk(i,2)) );
      rr = diag( scald*ones(1,blk(i,1)) );
      dmatl(lind,lind) = ll;
      dmatr(rind,rind) = rr;
   end
end
dmatlorig = dmatl;
dmatrorig = dmatr;
currentdl = dmatl;
currentdr = dmatr;
cursclmat = currentdl*(clpg/currentdr);
cursclbnd = fnorm(cursclmat);
maxord = 5;
perctol = 1.03;

if diagnos == 1
   clf
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
               title(['Block ' si '[' sj ' ' sk ']']);
               disp('Paused...');pause;disp('Running...')
            end
         end
      end
   end
   clf
   uplot(magax,sel(bnds,1,1),cursclbnd)
   title('DIAGNOSIS: Mu Upper Bounds and Scaled(GenPhase) Bounds')
   disp('Paused...');pause;disp('Running...')
   clf
end

clf
if isempty(rp)
   comparebnds = axes('position',[.1 .72 .85 .23]);
   plotmag = axes('position',[.1 .41 .85 .23]);
   plotsens = axes('position',[.1 .1 .85 .23]);
else
   comparebnds = axes('position',[.1 .6 .35 .35]);
   plotmag = axes('position',[.6 .6 .35 .35]);
   plotphase = axes('position',[.6 .1 .35 .35]);
%    plotphasehidden = axes('position',[.6 .1 .35 .35],'visible','off',...
%       'xlim',[0 1],'ylim',[0 1]);
   hiddentext = text(.5,.5,'No Phase');
   set(hiddentext,'visible','off','horizontalalignment','center');
   plotsens = axes('position',[.1 .1 .35 .35]);
end

op = 'onlydata';
iblk = 1;
irow = 1;
icol = 1;
% ord = -1;
gostop = 1;
skipchoice = 0;
firsttime = 1;
m1 = [];
m2 = [];
%fitstat = zeros(nblk,1);
fitstat = zeros(pimp(nblk+1)-1,1); %old way, MSF

axes(comparebnds);
uplot(magax,cursclbnd,'--');
while gostop
   pimpointer = pimp(iblk) + (icol-1)*blk(iblk,1) + (irow-1);
   lfr = bp(iblk,2) + irow - 1;
   lfc = bp(iblk,2) + icol - 1;
   rir = bp(iblk,1) + irow - 1;
   ric = bp(iblk,1) + icol - 1;
   data = dmatl(lfr,lfc);
   wt = sens(1,iblk);
   if strcmp(op,'changedisplay')
      disp(m1);
      sys = syscell{pimpointer};
      curord = xord(sys);
      sysg = frd(sys,data.Frequency);
      if curord >= 0
         tt = ['Block[' int2str(iblk) '], Element[' int2str(irow) ' ' int2str(icol) ...
            '], Order[' int2str(curord) ']'];
      else
         tt = ['(' int2str(iblk) ';' int2str(irow) ' ' int2str(icol) ...
            ':' 'd)'];
      end
      axes(plotmag)
      uplot(lmagax,data,'-',sysg,'--');
      title(['Magnitude Data and Fit: ' tt])
      if blk(iblk,2)==0 && blk(iblk,1) > 1 && ~isempty(rp)
         axes(plotphase)
         uplot(phax,data,'-',sysg,'--');
         title(['Phase Data and Fit'])
         set(plotphase,'visible','on');
         set(hiddentext,'visible','off')
      elseif ~isempty(rp)
         set(plotphase,'visible','off');
         delete(get(plotphase,'children'))
         set(hiddentext,'visible','on')
      end
      axes(plotsens)
      uplot(lmagax,wt);
      title(['Sensitivity'])
      disp(m2);
   elseif strcmp(op,'newfit')
      disp(m1);
      sys = fitfrd(data,ord,[],wt);
      curord = ord;
      %disp(['PIMPOINTER after call to FITFRD: ' int2str(pimpointer)])
      syscell{pimpointer} = sys;
      fitstat(pimpointer) = 1;
      sysg = frd(sys,omega);
      if blk(iblk,2) == 0
         currentdl(lfr,lfc) = sysg;
         currentdr(rir,ric) = sysg;
      else
         lind = bp(iblk,2):bp(iblk+1,2)-1;
         rind = bp(iblk,1):bp(iblk+1,1)-1;
         ll = diag(sysg*ones(1,blk(iblk,2)));
         rr = diag(sysg*ones(1,blk(iblk,1)));
         currentdl(lind,lind) = ll;
         currentdr(rind,rind) = rr;
      end
      cursclmat = currentdl*(clpg/currentdr);
      cursclbnd = fnorm(cursclmat);
      axes(comparebnds);
      uplot(magax,bnds(1,1),'-',cursclbnd,'--');
      title('Scaled Bound and MU')
      axes(plotmag)
      uplot(lmagax,data,'-',sysg,'--');
      tt = ['(' int2str(iblk) ';' int2str(irow) ' ' int2str(icol) ...
         ':' int2str(curord) ')'];
      title(['Mag Data and Fit: ' tt])
      if blk(iblk,2)==0 && blk(iblk,1) > 1 && ~isempty(rp)
         axes(plotphase)
         uplot(phax,data,'-',sysg,'--');
         title(['Phase Data and Fit'])
         set(plotphase,'visible','on');
         set(hiddentext,'visible','off')
      elseif ~isempty(rp)
         set(plotphase,'visible','off');
         delete(get(plotphase,'children'))
         set(hiddentext,'visible','on')
      end
      disp(m2);
   elseif strcmp(op,'status')
      disp(m1)
      for i=1:nblk
         si = int2str(i);
         if blk(i,2)==0
            disp(['Block ' si ':']);
            for j=1:blk(i,1)
               str = [];
               sj = int2str(j);
               for k=1:blk(i,1)
                  sk = int2str(k);
                  pp=pimp(i)+(j-1)*blk(i,1)+(k-1);
                  if isempty(syscell)
                     os = 'data';
                  else
                     ind_d = syscell{pp};
                     nxord = xord(ind_d);
                     if isempty(ind_d)
                        os = 'data';
                     else
                        os=int2str(nxord);
                     end
                  end
                  str=[str '  (' sj ',' sk ')=' os ';'];
               end
               disp(['      ' str]);
            end
         else
            pp=pimp(i);
            %ind_d = xpii(syspim,pp);
            if isempty(syscell)
               os = 'data';
            else
               ind_d = syscell{pp};
               nxord = xord(ind_d);
               if isempty(ind_d)
                  os = 'data';
               else
                  os=int2str(nxord);
               end
               disp(['Block ' si ': ' os]);
            end
         end
      end
      disp(['AutoPrefit Fit Tolerance: ' num2str(perctol)]);
      disp(['AutoPrefit Maximum Order: ' int2str(maxord)]);
      disp(m2)
   elseif strcmp(op,'onlydata')
      disp(m1);
      curord = -1;
      tt = ['(' int2str(iblk) ';' int2str(irow) ' ' int2str(icol) ...
         ':' 'd)'];
      if firsttime == 1
         firsttime = 0;
         axes(comparebnds);
         uplot(magax,bnds(1,1),'-');
         title('MU Upper Bound')
      end
      axes(plotmag)
      uplot(lmagax,data,'-');
      title(['Magnitude Data: ' tt])
      if blk(iblk,2)==0 && blk(iblk,1) > 1 && ~isempty(rp)
         axes(plotphase)
         uplot(phax,data,'-');
         title(['Phase Data'])
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
      disp(m2);
   elseif strcmp(op,'autofit')
      disp(m1)
      [syscell,fitstat,currentdl,currentdr] = ...
         xmudkaf(clpg,bnds,dmatl,dmatr,sens,blk,maxord,perctol,[],dflag);
      disp(m2)
      skipchoice = 1;
   elseif strcmp(op,'hold')
      disp(m1)
      disp(m2)
   elseif strcmp(op,'tryagain')
      disp(m1);
   else
      error('Message not found')
   end
   if ~skipchoice
      c = input('Enter Choice (return for list): ','s');
      if isempty(c)
         c = 'ch';
      end
      [gostop,op,iblk,ord,irow,icol,perctol,maxord,m1,m2] = ...
         xfitchose(c,blk,iblk,irow,icol,fitstat,pimp,curord,...
         perctol,maxord);
   else
      skipchoice = 0;
      op = 'changedisplay';
      gostop = 1;
      m1 = [];
      m2 = [];
      cursclmat = currentdl*(clpg/currentdr);
      cursclbnd = fnorm(cursclmat);
      axes(comparebnds);
      uplot(magax,bnds(1,1),'-',cursclbnd,'--');
      title('Scaled Bound and MU')
   end
end

% reconstruct D
dsysl = [];
dsysr = [];
for i=1:nblk
   ls = bp(i,2);
   rs = bp(i,1);
   if blk(i,2) == 0
      le = bp(i+1,2)-1;
      re = bp(i+1,1)-1;
      dblk = [];
      for j=1:blk(i,1)    % col
         syscol = [];
         for k=1:blk(i,1)
            pp = pimp(i) + (j-1)*blk(i,1) + (k-1);
            syscol = [syscol; syscell{pp}];
         end
         dblk = [dblk syscol];
      end
      nxord = xord(dblk);
      if nxord > 0
         try
            smpdblk = spectralfact(dblk,[]);
         catch
            error('Fatal error in DKSYN (xmsf/spectralfact)');
         end
%          if dflag==1
%             smpdblk = c2dbil(xextsmp(d2cbil(dblk)));
%             smpdblk.Ts = dblk.Ts;
%          else
%             smpdblk = xextsmp(dblk);
%          end
%          if isempty(smpdblk)
%             %                save muerrfil
%             %                disp('D-FIT has failed, error file named MUERRFIL has been saved');
%             error('Fatal error in DKSYN(xmsf/xextsmp)');
%          end
         if diagnos == 1
            disp('D~D-Ds~Ds')
            dd1 = smpdblk'*smpdblk;
            dd2 = dblk'*dblk;
            if nxord < 12  % XXX
               norm(dd1-dd2,inf,.00001)
            else
               dd1g = frd(dd1,omega);
               dd2g = frd(dd2,omega);
               norm(dd1g-dd2g,inf)
            end
            %sbmudl only handles cont time
            [bdblk,hsv] = sbmudl(smpdblk);  % XXX
            disp('Hankel Singular Values')
            hsv
            disp('Paused...');pause;disp('Running...')
         end
      else
         smpdblk = dblk;
      end
      smpdblkg = frd(smpdblk,currentdl.Frequency);
      if diagnos == 1
         clf
         dll = currentdl(ls:le,ls:le);
         uplot(magax,svd(dll/smpdblkg))
         title(['DIAGNOSIS: Left SV: Block ' int2str(i)])
         disp('Paused...')
         pause
         disp('Running...')
         drr = currentdr(rs:re,rs:re);
         uplot(magax,svd(drr/smpdblkg))
         title(['DIAGNOSIS: Right SV: Block ' int2str(i)])
         disp('Paused...');pause;disp('Running...')
         clf
      end
      dsysl = blkdiag(dsysl,smpdblk);
      dsysr = blkdiag(dsysr,smpdblk);
   else
      dblk = [];
      %sys = xpii(syspim,pimp(i));
      sys = syscell{pimp(i)};
      nxord = xord(sys);
      if nxord > 0
         try
            smpsys = spectralfact(sys,[]);
         catch
            error('Fatal error in DKSYN (xmsf/spectralfact)');
         end         
%          if dflag==1
%             smpsys = c2dbil(xextsmp(d2cbil(sys)));
%             smpsys.Ts = sys.Ts;
%          else
%             smpsys = xextsmp(sys);
%          end
%          if isempty(smpsys)
%             %                save muerrfil
%             %                disp('D-FIT has failed, error file named MUERRFIL has been saved');
%             error('Fatal error in DKSYN(xmsf/xextsmp)');
%             return
%          end
         if diagnos == 1
            disp('D~D-Ds~Ds')
            % XXX
            norm(smpsys'*smpsys-sys'*sys,inf,.000001)
            disp('Paused...');pause;disp('Running...')
         end
      else
         smpsys = sys;
      end
      for l=1:min(blk(i,:))
         dblk = blkdiag(dblk,smpsys);
      end
      if diagnos == 1
         smpsysg = frd(smpsys,currentdl.Frequency);
         clf
         dll = currentdl(ls,ls);
         uplot(magax,abs(dll/smpsysg))
         title(['DIAGNOSIS: Left Abs: Block ' int2str(i)])
         disp('Paused...')
         pause
         disp('Running...')
         drr = currentdr(rs,rs);
         uplot(magax,abs(drr/smpsysg))
         title(['DIAGNOSIS: Right Abs: Block ' int2str(i)])
         disp('Paused...');pause;disp('Running...')
         clf
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
end
if diagnos == 1
   dsyslg = frd(dsysl,currentdl.Frequency);
   clf
   uplot(magax,svd(currentdl/dsyslg))
   title('DIAGNOSIS: Left SV: Full')
   disp('Paused...');pause;disp('Running...')
   clf
   disp('POLES of DSYSL and DSYSR')
   damp(dsysl)
   damp(dsysr)
   disp('Paused...');pause;disp('Running...')
   disp('POLES of minv(DSYSL) and minv(DSYSR)')
   damp(dsysl^{-1})
   damp(dsysr^{-1})
   disp('Paused...');pause;disp('Running and returning...')
end


%----------------------- End of XMSF -----------------------%
function ord = xord(sys)

if isa(sys,'double')
   ord = 0;
else
   ord = length(sys.StateName);
end
