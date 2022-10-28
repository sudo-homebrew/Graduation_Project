function [lvec,rvec,gamma,alpha,Delta,DeltaCell] = onepiter(mat,blk,ne,nd,varargin)
%   Power method for WCPERFORMANCE, with BLK representing an upper-loop
%   LFT on MAT.  the peformance channel is assumed to be [NE x ND]

%   Copyright 2010-2012 The MathWorks, Inc.
try
   % Problem exists if Delta's have no effect on gain, and gain equals 0.
   % The easiest manner to catch this is to let this algorithm bomb out.
   % If that happens, the returned value of Delta is irrelevant, so the
   % CATCH part of this code simply sets all DELTA blocks to 0.  ALPHA is
   % returned as 1, which indicates that the worst-case gain is not INF.
   %
   % An example problem matrix is mat = [-.2 0;-1 0]; blk = [1 1]
   [sb,fb,perfe,perfd,rmpert,cmpert,whichtype] = oblk2ds(blk,ne,nd);
   bwpert = 1:cmpert;
   bwperf = perfd;
   azpert = 1:rmpert;
   azperf = perfe;
   nblk = size(blk,1) + 1;
   
   szm = size(mat);
   bblk = [blk;nd ne];
   %[bnd,dvec] = mu(mat,bblk); [DL,DR] = unwrapd(dvec,bblk); M = DL*(mat/DR);
   M = munochk(mat,bblk);
   
   CNTMAX = 200;
   stol = 1e-4;
   
   z = zeros(szm(1),1);
   a = z;
   newa = a; newz = z;
   if nargin==5
      % Compute initial values for B,W
      % varargin = random number generator
      RNG = varargin{1};
      [~,~,v] = svd(M);
      %b = crandn(szm(2),1);
      %b = complex(randn(szm(2),1),randn(szm(2),1));
      %b = v(:,1) + 0.05*crandn(szm(2),1);
      b = v(:,1) + 0.05*complex(randn(RNG,szm(2),1),randn(RNG,szm(2),1));
      b = b/norm(b);
      %w = crandn(szm(2),1);
      %w = randn(szm(2),1) + randn(szm(2),1)*sqrt(-1);
      %w = v(:,2) + 0.05*crandn(szm(2),1);
      w = v(:,2) + 0.05*complex(randn(RNG,szm(2),1),randn(RNG,szm(2),1));
      w = w/norm(w);
      %XXXX need to study this more...
   elseif nargin==6
      % Initial values BSTART,WSTART supplied
      b = varargin{1};
      w = varargin{2};
   else
      error('Invalid argument list');
   end
   newb = b; neww = w;
   go = 1;
   cnt = 0;
   M11 = M(azpert,bwpert);
   M12 = M(azpert,bwperf);
   M21 = M(azperf,bwpert);
   M22 = M(azperf,bwperf);
   M11s = M11';
   M12s = M12';
   M21s = M21';
   M22s = M22';
   while go==1 && cnt<CNTMAX
      blkuse = ones(nblk,1);
      cnt = cnt + 1;
      bbt = b(bwpert);
      bbf = b(bwperf);
      M11b = M11*bbt;
      M12b = M12*bbf;
      M21b = M21*bbt;
      M22b = M22*bbf;
      [gamma1,alpha1,newa] = s4vecp(M11b,M12b,M21b,M22b);
      for i=1:length(sb.azidx)
         wk = w(sb.bwidx{i});
         ak = newa(sb.azidx{i});
         cn = wk'*ak;
         if abs(cn)>100*eps
            newz(sb.azidx{i}) = (cn/abs(cn))*wk;
         else
            newz(sb.azidx{i}) = 0*wk;
            blkuse(sb.loc(i)) = 0;
         end
      end
      for i=1:length(fb.azidx)
         wk = w(fb.bwidx{i});
         ak = newa(fb.azidx{i});
         nak = norm(ak);
         if nak>100*eps
            newz(fb.azidx{i}) = (norm(wk)/nak)*ak;
         else
            newz(fb.azidx{i}) = 0*ak;
            blkuse(fb.loc(i)) = 0;
         end
      end
      if alpha1==0
         newz(perfe) = zeros(ne,1);
      else
         wk = w(perfd);
         ak = newa(perfe);
         newz(perfe) = (norm(wk)/norm(ak))*ak;
      end
      zzt = newz(azpert);
      zzf = newz(azperf);
      M11sz = M11s*zzt;
      M12sz = M12s*zzt;
      M21sz = M21s*zzf;
      M22sz = M22s*zzf;
      [gamma2,alpha2,neww] = s4vecp(M11sz,M21sz,M12sz,M22sz);
      for i=1:length(sb.azidx)
         wk = neww(sb.bwidx{i});
         ak = newa(sb.azidx{i});
         cn = ak'*wk;
         if abs(cn)>100*eps
            newb(sb.bwidx{i}) = (cn/abs(cn))*ak;
         else
            newb(sb.bwidx{i}) = 0*ak;
            blkuse(sb.loc(i)) = 0;
         end
      end
      for i=1:length(fb.azidx)
         wk = neww(fb.bwidx{i});
         ak = newa(fb.azidx{i});
         nwk = norm(wk);
         if nwk>100*eps
            newb(fb.bwidx{i}) = (norm(ak)/nwk)*wk;
         else
            newb(fb.bwidx{i}) = 0*wk;
            blkuse(fb.loc(i)) = 0;
         end
      end
      if alpha2==0
         newb(perfd) = zeros(nd,1);
         blkuse(nblk) = 0;
      else
         wk = neww(perfd);
         ak = newa(perfe);
         newb(perfd) = (norm(ak)/norm(wk))*wk;
      end
      chng = [norm(newb-b) norm(newa-a) norm(newz-z) norm(neww-w)];
      if all(chng<stol)
         go = 0;
      end
      a = newa;
      b = newb;
      z = newz;
      w = neww;
      %[cnt alpha1 alpha2 gamma1 gamma2 max(chng)]
      %[cnt chng]
   end
   % REFERENCE: pp. 84-85, Packard & Doyle, Automatica, vol. 29, no. 1, 1993
   % Eq 7.9  xi_2=0 --> b2=z2=w2=a2=0,   also if any (abwz)_2 = 0, then all are.
   % If gammas are 1, then performance has been made bad (to 1/alpha) with delta=1 (1/gamma).
   % If alphas are 0, then singularity has been made, with delta = 1/gamma.
   % If blkuse(i)==0, then that block does not enter into problem, and we
   %   can just set associated DELTA to 0.
   % Data structure to return Delta info could be lvec,rvec cell arrays
   alpha = sqrt(alpha1*alpha2);
   gamma = sqrt(gamma1*gamma2);
   Delta = [];
   nb = size(blk,1);
   DeltaCell = cell(nb,1);
   lvec = cell(nb,1);
   rvec = cell(nb,1);
   for i=1:nb
      if blkuse(i)==0
         if blk(i,2)==0
            % Repeated scalars are only returned as scalar-valued LRvecs
            lvec{i} = 0;
            rvec{i} = 0;
            Delta = blkdiag(Delta,zeros(blk(i,1),blk(i,1)));
            DeltaCell{i} = zeros(blk(i,1));
         else
            lvec{i} = zeros(blk(i,1),1);
            rvec{i} = zeros(1,blk(i,2));
            Delta = blkdiag(Delta,zeros(blk(i,1),blk(i,2)));
            DeltaCell{i} = zeros(blk(i,1),blk(i,2));
         end
      else
         idx = abs(whichtype(i));
         if blk(i,2)==0
            ai = newa(sb.azidx{idx});
            wi = neww(sb.bwidx{idx});
            cn = ai'*wi;
            % Repeated scalars are only returned as scalar-valued LRvecs
            lvec{i} = cn/abs(cn)/gamma;
            rvec{i} = 1;
            Delta = blkdiag(Delta,lvec{i}*eye(blk(i,1)));
            DeltaCell{i} = lvec{i}*eye(blk(i,1));
         else
            ai = newa(fb.azidx{idx});
            bi = newb(fb.bwidx{idx});
            %XXXMAJOR
            %lvec{i} = bi/norm(ai)/sqrt(gamma);
            %rvec{i} = (ai')/norm(ai)/sqrt(gamma);
            lvec{i} = bi/norm(ai)*sqrt(gamma);
            rvec{i} = (ai')/norm(ai)*sqrt(gamma);
            Delta = blkdiag(Delta,lvec{i}*rvec{i});
            DeltaCell{i} = lvec{i}*rvec{i};
         end
      end
   end
catch %#ok<CTCH>
   Delta = [];
   nb = size(blk,1);
   DeltaCell = cell(nb,1);
   lvec = cell(nb,1);
   rvec = cell(nb,1);
   for i=1:nb
      if blk(i,2)==0
         % Repeated scalars are only returned as scalar-valued LRvecs
         lvec{i} = 0;
         rvec{i} = 0;
         Delta = blkdiag(Delta,zeros(blk(i,1),blk(i,1)));
         DeltaCell{i} = zeros(blk(i,1));
      else
         lvec{i} = zeros(blk(i,1),1);
         rvec{i} = zeros(1,blk(i,2));
         Delta = blkdiag(Delta,zeros(blk(i,1),blk(i,2)));
         DeltaCell{i} = zeros(blk(i,1),blk(i,2));
      end
   end
   alpha = 1;
   gamma = 0;
end


