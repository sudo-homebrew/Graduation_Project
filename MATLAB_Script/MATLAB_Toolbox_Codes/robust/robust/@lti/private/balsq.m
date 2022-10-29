function [Gm,aug,hhsv,slbig,srbig] = balsq(G,mrtype,no)
%BALSQ Square-root balanced truncation (stable plant).
%
% [SS_Q,AUG,SVH,SLBIG,SRBIG] = BALSQ(SS_,MRTYPE,NO) performs
%    balanced-truncation model reduction on G(s):=(a,b,c,d) via the
%    square-root of PQ such that the infinity-norm of the error
%    (Ghed(s) - G(s))<=sum of the 2(n-k) smaller Hankel singular
%    values (SVH).
%
%           (aq,bq,cq,dq) = (slbig'*a*srbig,slbig'*b,c*srbig,d)
%
%   Based on the "MRTYPE" selected, you have the following options:
%
%    1). MRTYPE = 1  --- no: order "k" of the reduced order model.
%
%    2). MRTYPE = 2  --- find a k-th order model such that the total error
%                      is less than "no"
%
%    3). MRTYPE = 3  --- display Hankel SV and prompt for "k".
%
%    AUG = [No. of state(s) removed, Error bound], SVH = Hankel SV's.

% R. Y. Chiang & M. G. Safonov 9/11/87 (rev. 9/21/97)
% Copyright 1988-2004 The MathWorks, Inc. 
% All Rights Reserved.

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
   aq= a; bq= b; cq= c; dq= d;
   slbig = eye(na); srbig = eye(na);
   aug = [0 0];
   Gm = ss(aq,bq,cq,dq);
   return
end
%
% ------ Disgard all the states:
%
if kk <= 1 | sum(hhsv) < 1e-16
   ma = 0; na = 0;
   aq = zeros(0,0);   bq = zeros(0,nd);
   cq = zeros(md,0);   dq = d;
   slbig = zeros(na,0); srbig = zeros(na,0);
   bnd = 2*sum(hhsv);
   aug = [na bnd];
   Gm = ss(aq,bq,cq,dq);
   return
end
%
% ------ k-th order Schur balanced model reduction:
%
bnd = 2*sum(hhsv(kk:na));
strm = na-kk+1;
aug = [strm bnd];
%
% ----- Find the square root for the basis of left/right eigenspace :
%
[up,sp,vp] = svd(p);
lr = up*diag(sqrt(diag(sp)));
[uq,sq,vq] = svd(q);
lo = uq*diag(sqrt(diag(sq)));
%
[uc,sc,vc] = svd(lo'*lr);
%
% ------ Find the similarity transformation :
%
s1 = sc(1:(kk-1),1:(kk-1));
scih = diag(ones(kk-1,1)./sqrt(diag(s1)));
uc = uc(:,1:(kk-1));
vc = vc(:,1:(kk-1));
slbig = lo*uc*scih;
srbig = lr*vc*scih;
%
aq = slbig'*a*srbig;
bq = slbig'*b;
cq = c*srbig;
dq = d;
%
Gm = ss(aq,bq,cq,dq);
%
% ------ End of BALSQ.M --- RYC/MGS 9/11/87 %
