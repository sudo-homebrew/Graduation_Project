function [bnd,rowdn,rowgn,sensn] = grmures(matin,rowdi,rowgi,sensi,blk,levin,visflag)
% function [bnd,rowdn,rowgn,sensn] = grmures(matin,rowd,rowg,sens,blk,levin,...
%                                                   visflag,muinfo)
%
% RMURESCGJB recomputes ROWDN, ROWGN and SENSN to be usable for
% fitting in a mixed mu synthesis iteration.  Also returns
% the level of BND achieved

% P. M. Young, December, 1996.
% GJB modified 28Aug08: Changed to RCT functions

%   Copyright 2009 The MathWorks, Inc.

if nargin < 6
	disp('usage: [bnd,rowdn,rowgn,sensn] = grmures(matin,rowd,rowg,sens,blk,levin,visflag)')
	return
end

if nargin==6
   visflag = [];
end
if isempty(visflag)
   visflag=1;
end

% [mtype,mrows,mcols,mnum] = minfo(matin);
[mrows,mcols] = size(matin);
%------------------ Remove ALL Error checking  --------------------%
for ii=1:length(blk(:,1))
    if all( blk(ii,:) == [ 1 0] )
        blk(ii,:)  = [ 1 1] ;
    end
    if all( blk(ii,:) == [-1 1] )
        blk(ii,:)  = [-1 0] ;
    end
end
%----------------- End of Error checking -----------------------%

% [dl,dr] = muunwrap(rowdi,blk);
% Change to RCT
[dl,dr] = mussvunwrap(rowdi,blk);
[Or,Oc,Ur,Uc,K,I,J] = reindex(blk);
if any([length(Oc),length(Or)]~=[mrows,mcols]);
	error('   MATIN dimensions incompatible with BLK dimensions')
end
	[Kc,Kr,Ic,Ir,Jc,Jr,Fc,Fr,kc,kr,ic,ir,jc,jr,fc,fr,sc,sr,csc,csr,...
        csF,nK,nI,nJ,nF,nc,nr,kcK,icI,jcJ,jrJ,fcF] = rubind(K,I,J);

dmask = fcF'*fcF;
ds = [];
wd = sum(sum(dmask)) + nJ;
rowgn = [];	
wg = sum(K);
nT = nK+nI+nJ; % total number of blocks

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isa(matin,'double')      % strcmp(mtype,'cons')
    % CONSTANT matrix
    M = matin(Oc,Or)/levin;  
    dc = dl(Oc,Oc);  
    dri = inv(dr(Or,Or));
    % RUBRESCV4 cleaned up GJB 22Aug08: Only Matlab commands
    [tbnd,sensn,dcd,drid,g] = grubresc(M,K,I,J,Kc,Kr,Ic,Ir,Jc,Jr,Fc,Fr,kc,kr,...
        ic,ir,jc,jr,fc,fr,sc,sr,csc,csr,csF,nK,nI,nJ,nF,nc,nr,kcK,icI,jcJ,...
        jrJ,fcF,dc,dri,sensi);
	if sum(Fc)
		Fclog = logical(Fc);	
		df = dcd(Fclog,Fclog); 
        rowdn = df(logical(dmask));
	end
	if sum(Jc)
        Jclog = logical(Jc);	
        ds = max(jcJ'.*(diag(dcd(Jclog,Jclog))*ones(1,nJ)));
    end
    bnd = levin*tbnd;
    rowdn = [rowdn(:); ds(:)].';
    rowgn = g.';
elseif isa(matin,'lti')      % strcmp(mtype,'vary')
    % FRD matrix - initialize output matrices
    rowdn = rowdi;
    rowgn = rowgi;
    sensn = sensi;
    % bnd  =	zeros(  mnum+1,2);
    % bnd(  mnum+1,2) = inf; 
    % bnd(  mnum+1,1) = mnum; 
    % bnd(1:mnum  ,2) = matin(1:mnum,mcols+1);
    omega = rowdi.Frequency;
    mnum = size(omega,1);
    bnd = frd(zeros(1,2,mnum),omega,rowdi.Ts);
    pm = 0:mrows:mnum*mrows;  
    pcm = 0:mcols:mnum*mcols;
    count = 0;
    if visflag==1
        fprintf(' points completed....\n')
        fprintf(' ')
    end
    %   main loop on varying matrix
    for ii = 1:mnum
        % M = matin(pm(ii)+1:pm(ii+1),1:mcols);   % mutools
        M = matin.ResponseData(:,:,ii);
        M = M(Oc,Or)/levin;  
        % dc = dl(pm(ii)+1:pm(ii+1),1:mrows);     % mutools
        dc = dl.ResponseData(:,:,ii);
        dc = dc(Oc,Oc);  
        % dri = dr(pcm(ii)+1:pcm(ii+1),1:mcols);  % mutools 
        dri = dr.ResponseData(:,:,ii);
        dri = inv(dri(Or,Or));
        % tsensi = sensi(ii,1:nT);                % mutools
        tsensi = sensi.ResponseData(:,:,ii);
        [tbnd,tsens,dcd,drid,g] = grubresc(M,K,I,J,Kc,Kr,Ic,Ir,Jc,Jr,Fc,Fr,...
            kc,kr,ic,ir,jc,jr,fc,fr,sc,sr,csc,csr,csF,nK,nI,nJ,nF,nc,nr,kcK,...
            icI,jcJ,jrJ,fcF,dc,dri,tsensi);  

        trowd = [];
        if sum(Fc)
            Fclog = logical(Fc);
            df = dcd(Fclog,Fclog); 
            trowd = df(logical(dmask));
        end 
        if sum(Jc)
            Jclog = logical(Jc);
            ds = max(jcJ'.*(diag(dcd(Jclog,Jclog))*ones(1,nJ)));
        end 
        trowd = [trowd(:); ds(:)].';
        trowg = g.';

        % bnd(ii,1) = levin*tbnd;                % mutools
        % rowdn(ii,1:wd) = trowd;                % mutools
        bnd.ResponseData(1,1,ii)= levin*tbnd;
        rowdn.ResponseData(1,1:wd,ii)= trowd;
        if wg
            % rowgn(ii,1:wg) = trowg;	         % mutools
            rowgn.ResponseData(1,1:wg,ii)= trowg;
        end
        % sensn(ii,1:nT) = tsens;	             % mutools
        sensn.ResponseData(1,1:nT,ii)= tsens;
        count = count + 1;
        if count < 18
            if visflag==1
                fprintf([int2str(ii) '.'])
            end
        else
            if visflag==1
                fprintf('\n')
                fprintf(' ')
                fprintf([int2str(ii) '.'])
            end
            count = 0;
        end
    end % for ii
    if visflag==1
       fprintf('\n')
    end
end
