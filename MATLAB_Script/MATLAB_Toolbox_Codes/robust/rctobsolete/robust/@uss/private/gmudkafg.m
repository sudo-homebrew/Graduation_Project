% function [syspim,fitstat,fitvec,fgl,fgm,fgr] = ...
%   gmudkafg(clpg,bnds,dmatl,dmatr,gmatl,gmatm,gmatr,bmax,sens,blk,maxord,...
%              perctol,syscell,fitstat,fitvec,dispflag)

%   Copyright (c) 1991-96 by MUSYN Inc. and The MathWorks

%   P.M. Young - February 1997
%   GJB modified 3Sept08 to call RCT commands

 function [syscell,fitstat,fitvec,fgl,fgm,fgr] = ...
  gmudkafg(clpg,bnds,dmatl,dmatr,gmatl,gmatm,gmatr,bmax,sens,blk,maxord,...
            perctol,syscell,fitstat,fitvec,dispflag)

 if nargin==14
    dispflag = [];
 end
 
 if isempty(dispflag)
    dispflag = 1;
 end

 blkr = blk;  
 blk = abs(blk);

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
 
 indexg = find(blkr(:,1)<0);
 numg = length(indexg);

 if max(size(maxord)) ~= numg || min(size(maxord)) ~= 1
    maxord = maxord(1)*ones(numg,1);
 end
 
 nblk = size(blk,1);
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
 
 lowmu_f  = find(uppermu<lowcutoff*peakmu);
 lowindx  = logical(lowmu_f);
 lowmu    = frd(uppermu(lowindx),omega(lowindx),bnds.Ts);
 highmu_f = find(uppermu>=lowcutoff*peakmu);
 highindx = logical(highmu_f);
 highmu   = frd(uppermu(highindx),omega(highindx),bnds.Ts);

 curgl = gmatl;
 curgm = gmatm;
 curgr = gmatr;
 fgl = gmatl;
 fgm = gmatm;
 fgr = gmatr;
 pp = 0;
 ppc = 0;

 if dispflag==1
 	fprintf(1,'Auto Fit in Progress \n');
 end

 for ii=1:numg
    i = indexg(ii);
    wt = sens(1,i);
    if blk(i,2) == 0 && blk(i,1) > 1  %repeated complex and/or real scalars
        if dispflag == 1
            fprintf(1,'\t G Block %d \n',i);
        end
        for j=1:blk(i,1)         % icol
            for k=1:j    	     % irow has to be <= icol
                if dispflag == 1
                    fprintf(1,'\t \t Element[%d %d], MaxOrder=%d, Order =',...
                        [k j maxord(ii)]);
                end
                pp = pimp(i) + (j-1)*blk(i,1) + (k-1);
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
                haved = 0;
                ord = 0;
                while haved == 0
                    if dispflag == 1
                        fprintf(1,' %d',ord);
                    end
                    if k~=j
                        sys = fitfrd(sqrt(-1)*data,ord,[],wt);
                    else
                        sys = gfitgdat(data,ord,wt);	% pass G (real) but returns fit to jG
                    end
                    sysg = (-1)*sqrt(-1)*frd(sys,omega); % note sys is a fit to jG
                    if k~=j
                        curgl(lfr,lfc) = sysg;
                        curgm(lfr,ric) = sysg;
                        curgr(rir,ric) = sysg;
                        curgl(lfrc,lfcc) = sysg';  % assign hermitian conjugate
                        curgm(lfrc,ricc) = sysg';
                        curgr(rirc,ricc) = sysg';
                    else
                        curgl(lfr,lfc) = real(sysg);  % diagonal
                        curgm(lfr,ric) = real(sysg);
                        curgr(rir,ric) = real(sysg);
                    end
                    tempscl_clpg = dmatl*clpg/dmatr;
                    if any(blkr(:,1)<0)            % repeated, real scalars
                        tempdmdjg = tempscl_clpg - bmax*sqrt(-1)*curgm;
                        tempgfac = eye(pertrow) + curgr*curgr;
                        tempgscal = inv(sqrtm(tempgfac));
                        scl_clpg = tempdmdjg*tempgscal;
                    else
                        scl_clpg = tempscl_clpg;
                    end
                    nscl_clpg = fnorm(scl_clpg);
                    if ~isempty(lowmu_f)
                        low_nscl = frd(nscl_clpg.ResponseData(lowindx),...
                                        omega(lowindx),nscl_clpg.Ts);
                        lowtest = low_nscl - lowmu - lowtestval*peakmu;
                    else
                        lowtest = frd(-1,1,nscl_clpg.Ts);
                    end
                    high_nscl = frd(nscl_clpg.ResponseData(highindx),...
                                     omega(highindx),nscl_clpg.Ts);
                    hightest = high_nscl - perctol*highmu;
                    if k~=j
                      if ( all(lowtest.ResponseData(:)<0) && ...
                            all(hightest.ResponseData(:)<0) ) || (ord >= maxord(ii))
                         haved = 1;
                      else
                         ord = ord+1;
                      end
                    else
                      if ( all(lowtest.ResponseData(:)<0) && ...
                              all(hightest.ResponseData(:)<0) ) || ((ord+1) >= maxord(ii))
                         haved = 1;
                      else
                         ord = ord+2;
                      end
                    end
                end
                if dispflag == 1
                   fprintf(1,'\n');
                end
                syscell{pp} = sys; 
                fitstat(pp) = 1;
                fitvec(pp) = ord;
                if k~=j  % form gji as gji = -gij~ (since sys fits jG)
                    [atmp,btmp,ctmp,dtmp] = ssdata(sys);
                    if isempty(atmp)
                        sysc = -dtmp';
                    else
                        sysc = ss(-atmp',ctmp',btmp',-dtmp');
                        sysc.Ts = sys.Ts;
                    end
                    syscell{ppc} = sysc;
                    fitstat(ppc) = 1;
                    fitvec(ppc) = ord;
                end
                if k~=j
                    fgl(lfr,lfc) = sysg;
                    fgm(lfr,ric) = sysg;
                    fgr(rir,ric) = sysg;
                    fgl(lfrc,lfcc) = sysg';  % assign hermitian conjugate
                    fgm(lfrc,ricc) = sysg';
                    fgr(rirc,ricc) = sysg';
                else
                    fgl(lfr,lfc) = real(sysg);  % diagonal
                    fgm(lfr,ric) = real(sysg);
                    fgr(rir,ric) = real(sysg);
                end
                curgl = gmatl;
                curgm = gmatm;
                curgr = gmatr;
            end
        end
    else                % complex full and/or real scalar
        j = 1;
        k = 1;
        if dispflag == 1
            fprintf(1,'\t G Block %d, MaxOrder=%d, Order =',[i maxord(ii)]);
        end
        pp = pimp(i) + (j-1)*blk(i,1) + (k-1);
        lfr = bp(i,2) + k - 1;
        lfc = bp(i,2) + j - 1;
        data = gmatl(lfr,lfc);
        haved = 0;
        ord = 0;
        while haved == 0
            if dispflag == 1
                fprintf(1,' %d',ord);
            end
            sys = gfitgdat(data,ord,wt);		% pass G (real) but returns fit to jG
            sysg = -sqrt(-1)*frd(sys,omega); 	% note sys is a fit to jG

            lind = bp(i,2):bp(i+1,2)-1;  % should only ever be scalar here 
            rind = bp(i,1):bp(i+1,1)-1;  % because no real full blocks allowed
            ll = real(diag(sysg*ones(1,blk(i,2))));
            rr = real(diag(sysg*ones(1,blk(i,1))));
            curgl(lind,lind) = ll;
            curgm(lind,rind) = ll;
            curgr(rind,rind) = rr;
            tempscl_clpg = dmatl*clpg/dmatr;
            if any(blkr(:,1)<0)              % real scalar
                tempdmdjg = tempscl_clpg - bmax*sqrt(-1)*curgm;
                pertrow = round(pertrow);
                tempgfac = eye(pertrow) + curgr*curgr;
                tempgscal = inv(sqrtm(tempgfac));
                scl_clpg = tempdmdjg*tempgscal;
            else
                scl_clpg = tempscl_clpg;
            end
            nscl_clpg = fnorm(scl_clpg);
            if ~isempty(lowmu_f)
              low_nscl = frd(nscl_clpg.ResponseData(lowindx),omega(lowindx),...
                              nscl_clpg.Ts);
              lowtest = low_nscl - lowmu - lowtestval*peakmu;
            else
              lowtest = frd(-1,1,nscl_clpg.Ts);
            end
            high_nscl = frd(nscl_clpg.ResponseData(highindx),omega(highindx),...
                             nscl_clpg.Ts);
            hightest = high_nscl - perctol*highmu;
            if ( all(lowtest.ResponseData(:)<0) && all(hightest.ResponseData(:)<0) ) || ...
                            (ord+1 >= maxord(ii))
                haved = 1;
            else
                ord = ord+2;
            end
        end
        if dispflag == 1
            fprintf(1,'\n');
        end
        syscell{pp} = sys;
        fitstat(pp) = 1;
        fitvec(pp) = ord;        
        fgl(lind,lind) = ll;
        fgm(lind,rind) = ll;
        fgr(rind,rind) = rr;
        curgl = gmatl;
        curgm = gmatm;
        curgr = gmatr;
    end
 end
 
