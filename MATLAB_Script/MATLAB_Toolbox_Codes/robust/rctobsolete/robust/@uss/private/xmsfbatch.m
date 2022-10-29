function [dsysl,dsysr] = ...
    xmsfbatch(clpg,bnds,muinfo,sens,blk,maxord,visflag,dflag)
% Copyright 2003-2015 The MathWorks, Inc.
if nargin==6
   visflag = [];
   dflag = [];
elseif nargin==7
   dflag = [];
end

if isempty(visflag)
   visflag = 0;
end
if isempty(dflag)
   dflag = 0;
end

blk = abs(blk);    % no reals yet
nblk = size(blk,1);
syscell = cell(nblk,1);

for i=1:nblk
   if blk(i,1)==1 && blk(i,2)==0
      blk(i,2) = 1;
   end
end

bdim = blk;
rp = find(blk(:,2)==0);
bdim(rp,2) = blk(rp,1);
bp = cumsum([1 1;bdim]);
% dsize = ones(nblk,1);
% dsize(rp) = blk(rp,1).*blk(rp,1);

[nblk,~] = size(blk);
[dmatl,dmatr] = mussvunwrap(muinfo);
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
szdl = size(dmatl);
szdr = size(dmatr);
nd = dmatl(szdl(1),szdl(1));
dmatl(1:szdl(1),:) = nd\dmatl(1:szdl(1),:);
dmatr(1:szdr(1),:) = nd\dmatr(1:szdr(1),:);

% block-by-block GENPHASE
for i=1:nblk
   lind = bp(i,2):bp(i+1,2)-1;
   rind = bp(i,1):bp(i+1,1)-1;
   if blk(i,2) == 0
      mrows = bdim(i,1);
      hpdd = dmatl(lind,lind);
      for ii=1:mrows
         sd = genphase(hpdd(ii,ii));
         sdphz = sd/abs(sd);               % complex, mag=1
         hpdd(ii,:) = sdphz*hpdd(ii,:);    % scale row
      end
      dmatl(lind,lind) = hpdd;
      dmatr(rind,rind) = hpdd;
   else
      hpdd = dmatl(lind(1),lind(1));
      scald = genphase(abs(hpdd));
      ll = diag(scald*ones(1,blk(i,2)) );
      rr = diag(scald*ones(1,blk(i,1)));
      dmatl(lind,lind) = ll;
      dmatr(rind,rind) = rr;
   end
end

%  currentdl = dmatl;
%  currentdr = dmatr;
%perctol = 1.02;
perctol = 1.03; % GJB changed 14Oct04, consistent with XMSF improve numerics
[syscell,~,~,~] = ...
   xmudkaf(clpg,bnds,dmatl,dmatr,sens,...
   blk,maxord,perctol,visflag,dflag);

% reconstruct D
dsysl = [];
dsysr = [];
pp = 0;
for i=1:nblk
%    ls = bp(i,2);
%    rs = bp(i,1);
   if blk(i,2) == 0
%       le = bp(i+1,2)-1;
%       re = bp(i+1,1)-1;
      dblk = [];
      sysdiag = [];
      for j=1:blk(i,1)    % col
         syscol = [];
         for k=1:blk(i,1)
            ppi =  pp + (j-1)*blk(i,1) + k; %XXX change GJB 1Apr05
            syscol = [syscol;syscell{ppi}];
            if j==k
               sysdiag = blkdiag(sysdiag,syscell{ppi});
            end
         end
         dblk = [dblk,syscol];
      end
      if ~isempty(dblk.StateName)
         try
            smpdblk = spectralfact(dblk,[]);
         catch
            smpdblk = ss(eye(size(dblk,1)),'Ts',dblk.Ts);
         end
      else
         smpdblk = dblk;
      end
      %smpdblkg = frd(smpdblk,currentdl.Frequency);
      dsysl = blkdiag(dsysl,smpdblk);
      dsysr = blkdiag(dsysr,smpdblk);
      pp = ppi; % Added 1Apr05
   else
      dblk = [];
      pp = pp + 1;
      sys = syscell{pp};
      if isa(sys,'ss')
         nxsys = length(sys.StateName);
      else
         nxsys = 0;
      end
      if nxsys > 0
         try
            smpsys = spectralfact(sys,[]);
         catch
            smpsys = ss(eye(size(sys,1)),'Ts',sys.Ts);
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
end
