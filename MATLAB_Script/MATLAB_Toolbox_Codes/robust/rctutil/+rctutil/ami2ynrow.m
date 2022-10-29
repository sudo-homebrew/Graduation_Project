function [rowd,rowg] = ami2ynrow(Dcell,Gcell,blk,beta,dfFlag)
% Utility.

if nargin==4
   dfFlag = false;
end

%   Copyright 2010-2011 The MathWorks, Inc.

[nblk,~] = size(blk);
%     blkp = ptrs(abs(blk));
%     mcolp = blkp(:,1);
%     mrowp = blkp(:,2);
%     nc = mcolp(nblk+1)-1;
%     nr = mrowp(nblk+1)-1;
cds = [];
rds = [];
scalds = [];
gs = [];

for i=1:nblk
   if blk(i,1) < -1 && blk(i,2)==0
      bd = -blk(i,1);
      if dfFlag
         df = Dcell{i};
      else
         da = Dcell{i};
         df = sqrtm(da);
      end
      ga = Gcell{i};
      if all(ga(:)==0) % AKP, Feb 13, 2011
         gf = ga;  % extra conditional
      else
         gf = (1/beta)*(df\ga/df);
      end
      [evc,evl] = eig(gf);
      gp = real(evl);
      gpd = diag(gp);
      dp = diag(sqrt(sqrt(ones(bd,1)+gpd.*gpd)))*evc'*df;
      gs = [gs gpd.']; %#ok<*AGROW>
      rds = [rds (reshape(dp,bd*bd,1)).'];
   elseif blk(i,1)>1 && blk(i,2)==0
      bd = blk(i,1);
      if dfFlag
         df = Dcell{i};
      else
         da = Dcell{i};
         df = sqrtm(da);
      end
      dp = df;
      cds = [cds (reshape(dp,bd*bd,1)).'];
   elseif blk(i,1)>0 && blk(i,2)>0
      %             rdim = blk(i,2);
      %             cdim = blk(i,1);
      if dfFlag
         df = Dcell{i};
      else
         da = Dcell{i};
         df = sqrt(real(da));
      end
      dp = df;
      scalds = [scalds dp];
   elseif  blk(i,1)==1 && blk(i,2)==0
      if dfFlag
         df = Dcell{i};
      else
         da = Dcell{i};
         df = sqrt(real(da));
      end
      dp = df;
      scalds = [scalds dp];
   elseif  blk(i,1)==-1
      %             bd = -blk(i,1);
      if dfFlag
         df = Dcell{i};
         da = df*df;
      else
         da = Dcell{i};
         df = sqrt(real(da));
      end
      ga = real(Gcell{i});
      gf = (1/beta)*(ga/da);
      gp = real(gf);
      dp = sqrt(sqrt(1+gp*gp))*df;
      gs = [gs gp];
      rds = [rds dp];
   end
end
rowd = [rds cds scalds];
rowg = gs;
