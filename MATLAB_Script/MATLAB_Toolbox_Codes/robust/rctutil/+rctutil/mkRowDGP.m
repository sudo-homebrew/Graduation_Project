function [rowd,rowg,rowp,sens] = mkRowDGP(blk, UBcert, LBcert, matin, FBI)
%

%   Copyright 2004-2016 The MathWorks, Inc.

if nargin<5
   FBI = zeros(0,1); % fixed block index
end
npts = numel(UBcert);
nblk = size(blk,1);
[index,blk] = rctutil.mkBlkData(blk,FBI);
ridxm = index.ridxm; % col2 of tblkp
cidxm = index.cidxm; % col1 of tblkp

wp=sum(sum(index.masks.DeltaFull_mask))+index.allrep.num;
wd=sum(sum(index.masks.Drep_mask))+index.full.num;
wg=sum(index.allreal.repeated);

rowd = zeros([1 wd npts]);
rowg = zeros([1 wg npts]);
rowp = zeros([1 wp npts]);
sens = zeros([1 nblk npts]);

[Mr,Mc] = size(matin);
M = freqresp(matin,[LBcert.w]);

for k=1:npts
   Dr = UBcert(k).VLmi.Dr;
   Gcr = UBcert(k).VLmi.Gcr;
   ub = UBcert(k).gUB;
   Delta = LBcert(k).Delta;
   
   Dcell = cell(nblk,1);  Gcell = cell(nblk,1);
   for i=1:nblk
      if blk(i,2)==0
         rptr = ridxm(i):(ridxm(i+1)-1);
         cptr = cidxm(i):(cidxm(i+1)-1);
      else
         rptr = ridxm(i);
         cptr = cidxm(i);
      end
      Dcell{i} = Dr(rptr,rptr);
      Gcell{i} = Gcr(cptr,rptr);
   end
   [rowd(:,:,k),rowg(:,:,k)] = rctutil.ami2ynrow(Dcell,Gcell,blk,ub);
   
   [tdl,tdr] = mussvunwrap(rowd(:,:,k),blk);
   sm = tdl*M(:,:,k)/tdr;
   if nblk == 1
      sens(1,1,k) = 1;
   else
      for ib=1:nblk
         sensr = sm(ridxm(ib):ridxm(ib+1)-1,[1:cidxm(ib)-1 cidxm(ib+1):Mc]);
         sensc = sm([1:ridxm(ib)-1 ridxm(ib+1):Mr],cidxm(ib):cidxm(ib+1)-1);
         sens(1,ib,k) = norm(sensr) + norm(sensc);
      end
   end
   
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
   rowp(:,:,k) = trowp;
end

