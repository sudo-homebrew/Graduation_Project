% function [dar,dac,garc,gacr] = ynftdam2(rowd,rowg,blk,beta)

% Copyright 2003-2004 The MathWorks, Inc.
function [dar,dac,garc,gacr] = ynftdam2(rowd,rowg,blk,beta)
% M'*dar*M - (MUBND^2)*dac + j*(gacr*M-M'*garc) is NSD        
% dar is NY x NY, dac is NU x NU, gacr is NU x NY, garc is NY x NU

    [nblk,dum] = size(blk);
    blkp = ptrs(abs(blk));
    mcolp = blkp(:,1);
    mrowp = blkp(:,2);
    nc = mcolp(nblk+1)-1;
    nr = mrowp(nblk+1)-1;

    [dl,dr,gl,gm,gr] = mussvunwrap(rowd,rowg,blk);       % packed YN to Block YN

    dar = zeros(nr,nr);
    dac = zeros(nc,nc);
    garc = zeros(nr,nc);
    gacr = zeros(nc,nr);

    for i=1:nblk
        if blk(i,1) < -1 & blk(i,2)==0
            bd = -blk(i,1);
            dyn = dl(mrowp(i):mrowp(i+1)-1,mrowp(i):mrowp(i+1)-1);
            gyn = gl(mrowp(i):mrowp(i+1)-1,mrowp(i):mrowp(i+1)-1);
            gscl = diag(gyn);
            dx = diag(ones(bd,1)./sqrt(sqrt(ones(bd,1)+gscl.*gscl)))*dyn;
            [u,s,v] = svd(dx);
            % u*v'*v*s*v' = dx
            hx = v*s*v';
            ux = u*v';
            df = hx;
            gf = ux'*gyn*ux;
            da = df'*df;
            ga = beta*df'*gf*df;
            dac(mcolp(i):mcolp(i+1)-1,mcolp(i):mcolp(i+1)-1) = da;
            dar(mrowp(i):mrowp(i+1)-1,mrowp(i):mrowp(i+1)-1) = da;
            gacr(mcolp(i):mcolp(i+1)-1,mrowp(i):mrowp(i+1)-1) = ga;
            garc(mrowp(i):mrowp(i+1)-1,mcolp(i):mcolp(i+1)-1) = ga;
        elseif blk(i,1)>1 & blk(i,2)==0
            bd = blk(i,1);
            dyn = dl(mrowp(i):mrowp(i+1)-1,mrowp(i):mrowp(i+1)-1);
            dx = dyn;
            [u,s,v] = svd(dx);
            % u*v'*v*s*v' = dx
            hx = v*s*v';
            df = hx;
            da = df'*df;
            dac(mcolp(i):mcolp(i+1)-1,mcolp(i):mcolp(i+1)-1) = da;
            dar(mrowp(i):mrowp(i+1)-1,mrowp(i):mrowp(i+1)-1) = da;
        elseif blk(i,1)>0 & blk(i,2)>0
            rdim = blk(i,2);    % col of delta, row of M
            cdim = blk(i,1);    % row of delta, col of M
            dyn = dl(mrowp(i),mrowp(i));
            dx = dyn;
            hx = abs(dx);
            df = hx;
            da = df*df;
            dac(mcolp(i):mcolp(i+1)-1,mcolp(i):mcolp(i+1)-1) = da*eye(cdim);
            dar(mrowp(i):mrowp(i+1)-1,mrowp(i):mrowp(i+1)-1) = da*eye(rdim);
        elseif  blk(i,1)==1 & blk(i,2)==0
            rdim = 1;
            cdim = 1;
            dyn = dl(mrowp(i),mrowp(i));
            dx = dyn;
            hx = abs(dx);
            df = hx;
            da = df*df;
            dac(mcolp(i):mcolp(i+1)-1,mcolp(i):mcolp(i+1)-1) = da*eye(cdim);
            dar(mrowp(i):mrowp(i+1)-1,mrowp(i):mrowp(i+1)-1) = da*eye(rdim);
        elseif  blk(i,1)==-1
            bd = -blk(i,1);
            dyn = dl(mrowp(i),mrowp(i));
            gyn = gl(mrowp(i),mrowp(i));
            gscl = gyn;
            dx = dyn/sqrt(sqrt(1+gscl*gscl));
            hx = abs(dx);
            ux = dx/hx;
            df = hx;
            gf = ux'*gyn*ux;
            da = df'*df;
            ga = beta*df'*gf*df;
            dac(mcolp(i),mcolp(i)) = da;
            dar(mrowp(i),mrowp(i)) = da;
            gacr(mcolp(i),mrowp(i)) = ga;
            garc(mrowp(i),mcolp(i)) = ga;
        end
    end
