function [Gm,aug,hhsv,slbig,srbig] = schbal(G,mrtype,no)
%SCHBAL Schur balanced truncation (stable plant).
%
% [SS_H,AUG,HSV,SLBIG,SRBIG] = SCHBAL(SS_,MRTYPE,NO) performs
%    Schur method model reduction on G(s):=(a,b,c,d) such that the infinity-
%    norm of the error (Ghed(s) - G(s)) <= sum of the 2(n-k) smaller Hankel
%    singular values.
%
%         (ahed,bhed,ched,dhed) = (slbig'*a*srbig,slbig'*b,c*srbig,d)
%
%   Based on the "MRTYPE" selected, you have the following options:
%
%    1). mrtype = 1  --- no: size "k" of the reduced order model.
%
%    2). mrtype = 2  --- find k-th order model such that the total error
%                      is less than "no".
%
%    3). mrtype = 3  --- display all the Hankel SV prompt for "k" (in this
%                      case, no need to specify "no").
%
%    AUG = [No. of state(s) removed, Error bound], HSV = Hankel SV's.

% R. Y. Chiang & M. G. Safonov 9/11/87
% Copyright 1988-2004 The MathWorks, Inc. 
% All Rights Reserved.
%------------------------------------------------------------------------

nag1=nargin;
%[emsg,nag1,xsflag,Ts,a,b,c,d,mrtype,no]=mkargs5x('ss',varargin); error(emsg);

[a,b,c,d,Ts] = ssdata(G);

if Ts, error('LTI inputs must be continuous time (Ts=0)'), end

[ma,na] = size(a);
[md,nd] = size(d);
[hhsv,p,q] = hksv(G);
%
% ------ Model reduction based on your choice of mrtype:
%
if mrtype == 1
   kk = no+1;
end
%
if mrtype == 2
   tails = 0;
   kk = 1;
   for i = ma:-1:1
       tails = tails + hhsv(i);
       if 2*tails > no
          kk = i + 1;
          break
       end
   end
end
%
if mrtype == 3
   no = input('Please enter the desired order: ');
   kk = no + 1;
end
%
% Check the desired order vs. minimal
%
ord = kk-1;
ordmin = min([ord,length(hhsv(find(hhsv>eps)))]);
if  ord > ordmin
   warning('A minimal order model has replaced your desired model.')
end
kk = ordmin + 1;
%
% ------ Save all the states:
%
if kk > na & sum(hhsv) > 1e-16
   ahed = a; bhed = b; ched = c; dhed = d;
   slbig = eye(na); srbig = eye(na);
   aug = [0 0];
   Gm = ss(ahed,bhed,ched,dhed);
   return
end
%
% ------ Disgard all the states:
%
if kk <= 1 | sum(hhsv) < 1e-16
   ma = 0; na = 0;
   ahed = zeros(0,0);   bhed = zeros(0,nd);
   ched = zeros(md,0);   dhed = d;
   slbig = zeros(na,0); srbig = zeros(na,0);
   bnd = 2*sum(hhsv);
   aug = [na bnd];
   Gm = ss(ahed,bhed,ched,dhed);
   return
end
%
% ------ k-th order Schur balanced model reduction:
%
bnd = 2*sum(hhsv(kk:na));
strm = na-kk+1;
aug = [strm bnd];
%
% ------ Find the left-eigenspace basis :
%
ro = (hhsv(kk-1)^2+hhsv(kk)^2)/2.;
gammaa = p*q-ro*eye(na);
[va,ta,msa] = blkrsch(gammaa,1,na-kk+1);
vlbig = va(:,(na-kk+2):na);
%
% ------ Find the right-eigenspace basis :
%
gammad = -gammaa;
[vd,td,msd] = blkrsch(gammad,1,kk-1);
vrbig = vd(:,1:kk-1);
%
% ------ Find the similarity transformation :
%
ee = vlbig'*vrbig;
[ue,se,ve] = svd(ee);
%
seih = diag(ones(kk-1,1)./sqrt(diag(se)));
slbig = vlbig*ue*seih;
srbig = vrbig*ve*seih;
%
ahed = slbig'*a*srbig;
bhed = slbig'*b;
ched = c*srbig;
dhed = d;
%
Gm = ss(ahed,bhed,ched,dhed);
%
% ------ End of SCHBAL.M --- RYC/MGS 9/11/87 %
