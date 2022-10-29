function [Ncell,den] = fitsyseng(respdata,thetavec,n,m,weight,idnum,fixedDen)
% RESPDATA: NPTS-by-ELE
% THETAVEC: NPTS-by-1
% N: scalar
% M: ELE-by-1
% WEIGHT: NPTS-by-ELE
% IDNUM: number of least-square iterations

%   Copyright 2003-2009 The MathWorks, Inc.

mx = max([n;m(:)]);
var = exp(sqrt(-1)*thetavec(:));
szr = size(respdata);
npts = szr(1);
ele = szr(2);
if nargin<7
   fixedDen = [];
end

if mx>0
   mxp1 = mx+1;
   powermat = ones(npts,mxp1);
   for i=1:mx
      powermat(:,mxp1-i) = powermat(:,mxp1-i+1).*var;
   end
   for i=1:ele
      wr = weight(:,i).*respdata(:,i);
      baseA{i} = repmat(weight(:,i),[1 m(i)+1]).*powermat(:,mxp1-m(i):mxp1);
      baseB{i} = repmat(-wr,[1 n]).*powermat(:,mxp1-(n-1):mxp1);
      baseH{i} = wr.*powermat(:,mxp1-n);
      Acell{i} = [real(baseA{i});imag(baseA{i})];
      Bcell{i} = [real(baseB{i});imag(baseB{i})];
      Hcell{i} = [real(baseH{i});imag(baseH{i})];
   end
   if isempty(fixedDen)
      [Ncell,D] = LOCALsslsq(Acell,Bcell,Hcell);
      NN{1} = Ncell;
      DD{1} = D;
      den = [1;D];
      vones = ones(npts,1);
      for idn=1:idnum
         adest = sqrt(abs(powermat(:,mxp1-n:mxp1)*den)); % npts x 1
         nweight = vones./adest;
         for i=1:ele
            tmp = repmat(nweight,[1 m(i)+1]).*baseA{i};
            Acell{i} = [real(tmp);imag(tmp)];
            tmp = repmat(nweight,[1 n]).*baseB{i};
            Bcell{i} = [real(tmp);imag(tmp)];
            tmp = nweight.*baseH{i};
            Hcell{i} = [real(tmp);imag(tmp)];
         end
         [Ncell,D] = LOCALsslsq(Acell,Bcell,Hcell);
         NN{idn+1} = Ncell;
         DD{idn+1} = D;
         dchange = norm(D-den(2:end));
         den = [1;D];
      end
   else
      D = fixedDen(2:end);
      Ncell = cell(ele,1);
      for i=1:ele
          Ncell{i} = Acell{i}\(Hcell{i} - Bcell{i}*D(:));
      end
      den = fixedDen;
   end
else
   % XXX same code - is this all OK for dord=0?  NO.
   error('XXXMAJOR do not call this with order=0');
end

function [Ncell,D] = LOCALsslsq(Acell,Bcell,Hcell)
% structured least squares for rational fitting
% [A B]*[N;d] = H
% A (block diagonal), B, H all as CELLs, N returned as CELL, D as DOUBLE
hw = ctrlMsgUtils.SuspendWarnings; %#ok<NASGU>
n = length(Acell);
skip = 1;
if ~skip
	alldims = zeros(n,2);
	for i=1:n
       alldims(i,:) = size(Acell{i});
	end
	rc = alldims(:,1)-alldims(:,2);
	szb = size(Bcell{1});
	trdim = sum(rc);
	LHS = zeros(trdim,szb(2));
	RHS = zeros(trdim,1);
	rp = cumsum([1;rc]);
	for i=1:n
       [Q,R] = qr(Acell{i});
       U2t = Q(:,alldims(i,2)+1:end)';
       LHS(rp(i):rp(i+1)-1,:) = U2t*Bcell{i};
       RHS(rp(i):rp(i+1)-1,:) = U2t*Hcell{i};
	end
	if nargout==2 || nargout==0
       D = LHS\RHS;
       %resid1 = norm(LHS*D-RHS);
       Ncell = cell(n,1);
       for i=1:n
          Ncell{i} = Acell{i}\(Hcell{i} - Bcell{i}*D);
          %resid2(i) = norm(Acell{i}*Ncell{i} - (Hcell{i} - Bcell{i}*D));
       end
       %[resid1 norm(resid2)]
	elseif nargout==1
       Ncell = LHS\RHS;  % only compute D
	end
else
   % unstructured solution approach.  generally has better numerical
   % results.
    A = [];
    B = [];
    H = [];
    Ncell = cell(n,1);
    for i=1:n
        A = blkdiag(A,Acell{i});
        B = [B;Bcell{i}]; %#ok<AGROW>
        H = [H;Hcell{i}]; %#ok<AGROW>
    end
    x = [A B]\H;
    pt = 1;
    for i=1:n
        dim = size(Acell{i},2);
        Ncell{i} = x(pt:pt+dim-1);
        pt = pt + dim;
    end
    D = x(pt:end);
end
