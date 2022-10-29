% function [syspim,fitstat,fitvec,fdl,fdr] = ...
%    gmudkafd(clpg,bnds,dmatl,dmatr,gmatl,gmatm,gmatr,bmax,sens,blk,maxord,perctol,dispflag)

%   Copyright (c) 1991-96 by MUSYN Inc. and The MathWorks

%   Modified for mixed mu by P.M. Young - February 1997
%   Modified GJB to use RCT commands - 2 Sept 08

 function [syscell,fitstat,fitvec,fdl,fdr] = ...
    gmudkafd(clpg,bnds,dmatl,dmatr,gmatl,gmatm,gmatr,bmax,sens,blk,...
                maxord,perctol,dispflag)

 if nargin==12
    dispflag = [];
 end
 
 if isempty(dispflag)
    dispflag = 1;
 end

 blkr = blk;  
 blk = abs(blk);
 nblk = size(blk,1);

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
 
 if max(size(maxord)) ~= nblk || min(size(maxord)) ~= 1
    maxord = maxord(1)*ones(nblk,1);
 end
 bdim = blk;
 rp = find(blk(:,2)==0);
 bdim(rp,2) = blk(rp,1);
 bp = cumsum([1 1;bdim]);
 dsize = ones(nblk,1);
 dsize(rp) = blk(rp,1).*blk(rp,1);
 pimp = cumsum([1;dsize]);
 pertrow = norm(bdim(:,1),1);
 omega = dmatl.Frequency;

 uppermu = bnds(1,1).ResponseData;
 peakmu = max(uppermu);
 lowcutoff = 0.5;    % needs to be adjustable
 lowtestval = 0.1;
 lowmu_f = find(uppermu<lowcutoff*peakmu);
 %low_pts = length(lowmu_f);
 lowmu = bnds(1,1).ResponseData(lowmu_f);
 highmu_f = find(uppermu>=lowcutoff*peakmu);
 %high_pts = length(highmu_f);
 highmu = bnds(1,1).ResponseData(highmu_f);

 curdl = dmatl;
 curdr = dmatr;
 fdl = dmatl;
 fdr = dmatr;
 syscell = {};

 if dispflag
   fprintf(1,'Auto Fit in Progress \n');
 end

 for i=1:nblk
    wt = sens(1,i);
    if blk(i,2) == 0 && blk(i,1) > 1
        if dispflag == 1
            fprintf(1,'\t D Block %d \n',i);
        end
        for j=1:blk(i,1)        % icol
            for k=1:blk(i,1)    % irow
                if dispflag == 1
                   fprintf(1,'\t \t Element[%d %d], MaxOrder=%d, Order =',...
                      [k j maxord(i)]);
                end
                pp = pimp(i) + (j-1)*blk(i,1) + (k-1);
                lfr = bp(i,2) + k - 1;
                lfc = bp(i,2) + j - 1;
                rir = bp(i,1) + k - 1;
                ric = bp(i,1) + j - 1;
                data = dmatl(lfr,lfc);
                haved = 0;
                ord = 0;
                while haved == 0
                    if dispflag == 1
                        fprintf(1,' %d',ord);
                    end
                    if j==k % diagonal element, fit with min phase, stable
                       [sys,fitfail] = fitfrd(data,ord,[],wt,1);
                    else    % off diagnal, fit with anything
                       [sys,fitfail] = fitfrd(data,ord,[],wt,0);
                    end
                    sysg = frd(sys,omega);
                    curdl(lfr,lfc) = sysg;
                    curdr(rir,ric) = sysg;
                    tempscl_clpg = curdl*(clpg/curdr); 
                    if any(blkr(:,1)<0)
                       tempdmdjg = tempscl_clpg - bmax*sqrt(-1)*gmatm;
                       tempgfac = eye(pertrow) + gmatr*gmatr;
                       tempgscal = inv(sqrtm(tempgfac));
                       scl_clpg = tempdmdjg*tempgscal;
                    else
                       scl_clpg = tempscl_clpg;
                    end
                    nscl_clpg = fnorm(scl_clpg);
                    if ~isempty(lowmu_f) 
                        low_nscl = nscl_clpg.ResponseData(lowmu_f);
                        lowtest = low_nscl-lowmu-lowtestval*peakmu;
                    else
                        lowtest = -1;
                    end
                    high_nscl = nscl_clpg.ResponseData(highmu_f);
                    hightest = high_nscl-perctol*highmu-perctol;
                    if ( all(lowtest<0) && all(hightest<0) ) || ord >= maxord(i) ...
                          || fitfail
                        haved = 1;
                    else
                        ord = ord+1;
                    end
                end
                if dispflag == 1
                    fprintf(1,'\n');
                end
                syscell{pp} = sys;
                fitstat(pp) = 1;
                fitvec(pp) = ord;
                fdl(lfr,lfc) = sysg;
                fdr(rir,ric) = sysg;
                curdl = dmatl;
                curdr = dmatr;
            end
        end
    else
        j = 1;
        k = 1;
        if dispflag == 1
            fprintf(1,'\t D Block %d, MaxOrder=%d, Order =',[i maxord(i)]);
        end
        pp = pimp(i) + (j-1)*blk(i,1) + (k-1);
        lfr = bp(i,2) + k - 1;
        lfc = bp(i,2) + j - 1;
        data = dmatl(lfr,lfc);
        haved = 0;
        ord = 0;
        while haved == 0
            if dispflag == 1
                fprintf(1,' %d',ord);
            end
            [sys,fitfail] = fitfrd(data,ord,[],wt);
            sysg = frd(sys,omega);
            lind = bp(i,2):bp(i+1,2)-1;
            rind = bp(i,1):bp(i+1,1)-1;
            ll = diag(sysg*ones(1,blk(i,2)));
            rr = diag(sysg*ones(1,blk(i,1)));
            curdl(lind,lind) = ll;
            curdr(rind,rind) = rr;
            tempscl_clpg = curdl*(clpg/curdr); 
            if any(blkr(:,1)<0)
               tempdmdjg = tempscl_clpg - bmax*sqrt(-1)*gmatm;
               pertrow=round(pertrow); 
               tempgfac = eye(pertrow) + gmatr*gmatr;
               tempgscal = inv(sqrtm(tempgfac));
               scl_clpg = tempdmdjg*tempgscal;
            else
               scl_clpg = tempscl_clpg;
            end
            nscl_clpg = fnorm(scl_clpg);
            if ~isempty(lowmu_f) 
                low_nscl = nscl_clpg.ResponseData(lowmu_f);
                lowtest = low_nscl-lowmu-lowtestval*peakmu;
            else
                lowtest = -1;
            end
            high_nscl = nscl_clpg.ResponseData(highmu_f);
            hightest = high_nscl-perctol*highmu;
            if ( all(lowtest<0) && all(hightest<0) ) || ord >= maxord(i) ...
               || fitfail
                haved = 1;
            else
                ord = ord+1;
            end
        end
        if dispflag == 1
            fprintf(1,'\n');
        end
        syscell{pp} = sys;
        fitstat(pp) = 1;
        fitvec(pp) = ord;
        fdl(lind,lind) = ll;
        fdr(rind,rind) = rr;
        curdl = dmatl;
        curdr = dmatr;
    end
 end
