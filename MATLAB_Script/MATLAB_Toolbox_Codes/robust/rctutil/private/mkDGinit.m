function DGinit = mkDGinit(M,blkData,opt)
%   Copyright 2004-2016 The MathWorks, Inc.

if nargin<3 || isempty(opt)
   opt = '';
end
osbopt = 'd';

blk = blkData.simpleblk;
FV = blkData.FVidx;
blkFIdx = FV.fixedBlkIdx;  % EMPTY (RS), Generic (WCG); 1 Full block (RG)

nAD = size(M,3);
Mavg = sum(M,3)/nAD;

if isempty(blkFIdx)
   ub = norm(Mavg);
   DGinit.Dr = eye(size(M,1));
   DGinit.Dc = eye(size(M,2));
   DGinit.Gcr = zeros(size(M,2),size(M,1));
   DGinit.Grc = DGinit.Gcr';
   DGinit.ub = 1.01*ub;
   if blkData.allreal.num>0 && ub>10*eps
      [ubFast,DrFast,DcFast,GcrFast] = mufastub(Mavg,blkData,[ub 0]);
      if ubFast<ub
         DGinit.Dr = DrFast;
         DGinit.Dc = DcFast;
         DGinit.Gcr = GcrFast;
         DGinit.Grc = GcrFast';
         DGinit.ub = 1.01*ubFast;
      end
      if nAD>1
         DGinit.ub = 5*DGinit.ub;
      end
   else
      nrm = zeros(nAD,1);
      for i=1:nAD
         nrm(i) = norm(M(:,:,i));
      end
      DGinit.ub = 1.01*max(nrm);
   end
else
   blkF = blk(blkFIdx,:); %
   blkV = blk;
   blkV(blkFIdx,:) = [];
   
   %% blkF as a single full block
   tmpF = abs(blkF);
   sbF = tmpF(:,2)==0;
   tmpF(sbF,2) = tmpF(sbF,1);
   blkFasFull = sum(tmpF,1);
   blkR = [blkV;blkFasFull];
   blkRdata = rctutil.mkBlkData(blkR);  
   
   %%
   MF = Mavg(FV.FixedRows, FV.FixedCols);
   if isscalar(blkFIdx) && blk(blkFIdx,2)>0
      bndsF = norm(MF);
      dcF = eye(size(MF,2));
      drF = eye(size(MF,1));
      GcrF = zeros(size(MF,2),size(MF,1));
      GrcF = GcrF';
      DLeft = drF;
      DRight = dcF;
   else
      [bndsF,mui] = mussv(MF,blkF,[opt 'U']); % standard MU (no Fixed blocks) on MF
      [~,VSigma,VLmi] = mussvextract(mui);
      dcF = VLmi.Dc;
      drF = VLmi.Dr;
      GcrF = VLmi.Gcr;
      GrcF = VLmi.Grc;
      DLeft = VSigma.DLeft;
      DRight = VSigma.DRight;
   end
   if bndsF(1)<1
      Mo = Mavg;
      Mo(FV.FixedRows,:) = DLeft*Mo(FV.FixedRows,:);
      Mo(:,FV.FixedCols) = Mo(:,FV.FixedCols)/DRight; % xxxx
      % rerun osborne (with DeltaF set to full) to obtain some decent DV.
      MoS = Mo([FV.VaryRows,FV.FixedRows],[FV.VaryCols,FV.FixedCols]);
      MS = Mavg([FV.VaryRows,FV.FixedRows],[FV.VaryCols,FV.FixedCols]);
      [~,drV,dciV] = osbal(MoS,blkRdata,osbopt);
      drV = drV(1:end-blkFasFull(2),1:end-blkFasFull(2))/drV(end,end);
      drV = drV'*drV;
      dcV = inv(dciV(1:end-blkFasFull(1),1:end-blkFasFull(1))/dciV(end,end));
      dcV = dcV'*dcV;
      M12 = Mavg(FV.VaryRows,FV.FixedCols);
      M22 = Mavg(FV.FixedRows,FV.FixedCols);
      B = M12'*drV*M12;
      A = dcF - M22'*drF*M22 - 1j*(GcrF*M22 - M22'*GrcF);
      ev = real(eig(A,B));
      tmax = min(ev(ev>0));  % e should be smaller
      e = min([1 tmax/2]);
      
      GcrE = blkdiag(zeros(size(dcV,1),size(drV,1)),GcrF);
      DrE = blkdiag(e*drV,drF);
      DcE = blkdiag(e*dcV,dcF);
      
      A = MS'*DrE*MS + 1j*(GcrE*MS - MS'*GcrE') - blkdiag(zeros(size(dcV)),dcF);
      B = blkdiag(e*dcV,zeros(size(dcF)));
      ev = real(eig(A,B));
      ubsq = max(ev(~isinf(ev)));  % e should be smaller
      
      DGinit.Dr([FV.VaryRows,FV.FixedRows],[FV.VaryRows,FV.FixedRows]) = DrE;
      DGinit.Dc([FV.VaryCols,FV.FixedCols],[FV.VaryCols,FV.FixedCols]) = DcE;
      DGinit.Gcr([FV.VaryCols,FV.FixedCols],[FV.VaryRows,FV.FixedRows]) = GcrE;
      DGinit.ub = 1.01*sqrt(ubsq);
      if nAD>1
         DGinit.ub = 5*DGinit.ub;
      end
      %  M'*blkdiag(e*DV,DF)M + j(M'*blkdiag(0,GF)-blkdiag(0,GF')*M) <= blkdiag(u^2 e*DV,DF).
      %  22: e*M12'*DV*M12 + M22'*DF*M22 j*(M22'*GF - GF'*M22) <= DF
      %  tA<=B, where B>0
   else
      % muFV = inf
      DGinit = struct('Dr',[],'Dc',[],'Gcr',[],'ub',inf);
   end
end

