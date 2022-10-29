function [q,gopt] = cmsclsyn(r,u,v,blk,opt,qinit,niter)
% CMSCLSYN  Constant-matrix Upper-Bound mu-Synthesis.
%
%  [QOPT,GAMMAOPT] = CMSCLSYN(R,U,V,BLK) minimizes, by choice
%  of Q, the upper bound of MUSSV(R+U*Q*V,BLK).  The algorithm
%  is iterative, alternatively holding Q fixed, and computing
%  the MUSSV upper bound, followed by holding the upper bound
%  multipliers fixed, and minimizing the bound implied by choice
%  of Q.  If U or V is square and invertible, then the optimization
%  is reformulated (exactly) as an LMI, and solved directly, without
%  resorting to the iteration.
%
%  [QOPT,GAMMAOPT] = CMSCLSYN(R,U,V,BLK,OPT) uses the options
%  specified by OPT in the calls to MUSSV.  See MUSSV for more
%  information.
%
%  [QOPT,GAMMAOPT] = CMSCLSYN(R,U,V,BLK,OPT,QINIT) initializes
%  the iterative computation from Q = QINIT.  Due to the
%  nonconvexity of the overall problem, different starting points
%  often yield different final answers.  If QINIT is an N-D array,
%  then the iterative computation is performed multiple times - the
%  i'th optimization is initialized at Q = QINIT(:,:,i).  The
%  output arguments are associated with the best solution obtained
%  in this brute force approach.
%
%  [QOPT,GAMMAOPT] = CMSCLSYN(R,U,V,BLK,OPT,'random',N) initializes
%  the iterative computation from N random instances of QINIT.  If
%  NCU is the number of columns of U, and NRV is the number of rows
%  of V, then this equivalent to setting QINIT = randn(NCU,NRV,N).
%
%  See also: MUSSV

%   Copyright 2004-2010 The MathWorks, Inc.
if nargin<4
   disp('usage: [qopt,mubnd] = cmsclsyn(r,u,v,blk,opt,Qinit)');
   return
end

if nargin==4
   opt = [];
   qinit = [];
   niter = [];
elseif nargin==5
   qinit = [];
   niter = [];
elseif nargin==6
   niter = [];
end

szr = size(r);
szu = size(u);
szv = size(v);
ned = max([length(szr) length(szu) length(szv)]) - 2;
szr = [szr ones(1,ned+2-length(szr))];
szu = [szu ones(1,ned+2-length(szu))];
szv = [szv ones(1,ned+2-length(szv))];
exd = max([szr(3:end);szu(3:end);szv(3:end)],[],1);

if all(szr(3:end)==1 | szr(3:end)==exd) && ...
      all(szu(3:end)==1 | szu(3:end)==exd) && ...
      all(szv(3:end)==1 | szv(3:end)==exd)
   r = repmat(r,[1 1 exd./szr(3:end)]);
   u = repmat(u,[1 1 exd./szu(3:end)]);
   v = repmat(v,[1 1 exd./szv(3:end)]);
else
   error('Invalid extra dimensions.');
end
RFAC = 10;

if szr(1) ~= szu(1) || szr(2) ~= szv(2)
   error('R and U need same # of rows, R and V need same # of columns')
end
ablk = abs(blk);
sblk = find(ablk(:,2)==0);
if ~isempty(sblk)
   ablk(sblk,2) = ablk(sblk,1);
end
dims = sum(ablk);
if dims(1)~=szr(2) || dims(2)~=szr(1)
   error('Inconsistent Matrix/Uncertainty dimensions')
end

if isempty(opt)
   opt = 'cUsw';
elseif ~ischar(opt)
   error('Option should be a string');
else
   opt = [opt 'Usw'];
end
diagnose = 0;
if any(opt=='D')
   diagnose = 1;
end

if isempty(niter)
   niter = 1;
else
   niter = ceil(abs(niter(1)));
   if niter < 1
      error('NITER should be positive integer')
   end
end

if isempty(qinit)
   qinit = zeros(szu(2),szv(1));
else
   szqi = size(qinit);
   if isa(qinit,'char') && ndims(qinit)==2 && size(qinit,1)==1 && ...
         strncmpi(qinit,'Random',length(qinit))
      qinit = randn(szu(2),szv(1),niter)/RFAC;
   elseif ~isa(qinit,'double') || szqi(1)~=szu(2) || szqi(2)~=szv(1)
      error('Qinit should be a DOUBLE, and dimensions consistent with R, U, V');
   end
end


perctol = 0.001;
if isa(r,'double') && isa(u,'double') && isa(v,'double')
   q = zeros([szu(2) szv(1) exd]);
   gopt = zeros([1 1 exd]);
   ngpts = prod(exd);
   for i=1:ngpts
      %       [q(:,:,i),gopt(:,:,i),DGInfo] = LOCALcmsclsyn(r(:,:,i),u(:,:,i),v(:,:,i),...
      %          blk,opt,qinit,perctol,diagnose);
      [q(:,:,i),gopt(:,:,i)] = LOCALcmsclsyn(r(:,:,i),u(:,:,i),v(:,:,i),...
         blk,opt,qinit,perctol,diagnose);
   end
else
   error('The input arguments R, U and V should be DOUBLE.');
end

function [qbest,bestcost,DGInfo] = LOCALcmsclsyn(r,u,v,blk,opt,Qinit,perctol,diagnose)

bestcost = inf;
szr = size(r);
szu = size(u);
szv = size(v);
nr = szr(1);
nc = szr(2);
ur = szu(1);
uc = szu(2);
vr = szv(1);
vc = szv(2);

if ur==uc && rcond(u)>1000*eps
   [bestcost,UQ] = mussv(r,v,blk);
   qbest = u\UQ;
   %[dumbnd,DGInfo] = mussv(r+u*qbest*v,blk,'C3');
elseif vr==vc && rcond(v)>1000*eps
   blkt = blk;
   fb = find(blk(:,1).*blk(:,2)>0);
   blkt(fb,[1 2]) = blkt(fb,[2 1]);
   [bestcost,VTQT] = mussv(r.',u.',blkt);
   qbest = ((v.')\VTQT).';
   %[dumbnd,DGInfo] = mussv(r+u*qbest*v,blk,'C3');
else
   [~,QL] = mussv(r,v,blk);
   Qu = ruqvsol(QL,-u,eye(vr));
   blkt = blk;
   fb = find(blk(:,1).*blk(:,2)>0);
   blkt(fb,[1 2]) = blkt(fb,[2 1]);
   [~,VTQT] = mussv(r.',u.',blkt);
   QR = VTQT.';
   Qv = ruqvsol(QR,eye(uc),-v);
   Qsmart = cat(3,Qu,Qv,0.5*(Qu+Qv));
   Qinit = cat(3,Qsmart,Qinit);
   szqi = size(Qinit);
   ngpts = prod(szqi(3:end));
   for i=1:ngpts
      qinit = Qinit(:,:,i);
      iter = 0;
      dl = eye(nr);
      dr = eye(nc);
      go = 1;
      if all(blk(:,1)>0)
         [bnd,info] = mussv(r+u*qinit*v,blk,opt);
         starticost = bnd(1);
         if bnd(1) < bestcost
            bestcost = bnd(1);
            qbest = qinit;
            ibest = info;
            besti = i;
         end
         lastcost = bnd(1);
         [dl,dr] = mussvunwrap(info);
         while go==1
            [q,gam] = ruqvsol(dl*r/dr,dl*u,v/dr);
            [bnd,info] = mussv(r+u*q*v,blk,opt);
            if bnd(1) < bestcost
               bestcost = bnd(1);
               qbest = q;
               ibest = info;
               besti = i;
            end
            if diagnose==1
               disp([iter lastcost gam bnd(1) besti])
            end
            if lastcost-bnd(1)<perctol*bnd(1)
               go = 0;
               disp([i starticost iter bnd(1) nan bestcost besti])
            end
            lastcost = bnd(1);
            [dl,dr] = mussvunwrap(info);
            iter = iter + 1;
         end
         DGInfo = ibest;
      else
         el = eye(nr); er = eye(nc);
         [bnd,info] = mussv(r+u*qinit*v,blk,opt);
         starticost = bnd(1);
         if bnd(1) < bestcost
            bestcost = bnd(1);
            qbest = qinit;
            ibest = info;
            besti = i;
         end
         lastcost = bnd(1);
         [dl,dr,gl,gm,gr] = mussvunwrap(info);
         while go==1
            ll = inv(sqrtm(sqrtm(el+gl*gl)));
            rr = inv(sqrtm(sqrtm(er+gr*gr)));
            r0 = ll*dl*r/dr*rr;
            r1 = -sqrt(-1)*ll*gm*rr;
            uu = ll*dl*u; vv = v/dr*rr;
            
            % [size(r0);size(r1);size(uu);size(vv)]
            maxlam = ruqvsolb(r0,r1,uu,vv);
            xtry = 1/maxlam;
            newr = ll*( (1/xtry)*dl*r/dr-sqrt(-1)*gm )*rr;
            newu = (1/xtry)*ll*dl*u;
            newv = v/dr*rr;
            [q,gam] = ruqvsol(newr,newu,newv);
            [bnd,info] = mussv(r+u*q*v,blk,opt);
            if bnd(1) < bestcost
               bestcost = bnd(1);
               qbest = q;
               ibest = info;
               besti = i;
               %disp([iter lastcost gam bnd(1) besti])
            end
            if diagnose==1
               disp([iter lastcost gam bnd(1) besti])
            end
            if lastcost-bnd(1)<perctol*bnd(1)
               go = 0;
               disp([i starticost iter bnd(1) nan bestcost besti])
            end
            lastcost = bnd(1);
            [dl,dr,gl,gm,gr] = mussvunwrap(info);
            iter = iter + 1;
         end
         DGInfo = ibest;
      end
   end
end
