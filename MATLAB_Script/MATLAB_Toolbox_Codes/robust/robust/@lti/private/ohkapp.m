function [Gx,Gy,aug] = ohkapp(G,mrtype,in)
%OHKAPP Optimal Hankel norm approximation (stable plant).
%
% [SS_X,SS_Y,AUG] = OHKAPP(SS_,MRTYPE,IN) produces
%   an optimal Hankel norm approximation via descriptor system.
%   (AX,BX,CX,DX) is the reduced model, (AY,BY,CY,DY) is the
%   anticausal part of the solution.
%   The infinity norm of (G - Ghed) <= sum of the Kth Hankel s.v. to
%   nth Hankel s.v. times 2. NO BALANCING IS REQUIRED.
%
%   mrtype = 1, in = order "k" of the reduced model.
%
%   mrtype = 2, find a reduced order model such that the total error is
%             less than "in".
%
%   mrtype = 3, display Hankel singular values, prompt for order "k" (in
%             this case, no need to specify "in").
%
%   AUG = [max. Hank SV, state(s) removed, Error Bound, Hankel SV's].
%
%   Note: If size(Gr) is not equal to the "order" you specified. The
%   optimal Hankel MDA algorithm has selected the best Minimum Degree
%   Approximate it can find within the allowable machine accuracy. 

% R. Y. Chiang & M. G. Safonov 4/11/87
% Copyright 1988-2004 The MathWorks, Inc. 
%        Improved code on "sigi" RYC 2/28/04
%       %Revision: 1.1.4.3 $ Improved code on hsv multiplicity and minimal
%       subspace index of "ee" RYC 5/31/04
% All Rights Reserved.
%--------------------------------------------------------------------
%
nag1=nargin;
%[emsg,nag1,xsflag,Ts,a,b,c,d,mrtype,in]=mkargs5x('ss',varargin); error(emsg);

[a,b,c,d,Ts] = ssdata(G);

if Ts, error('LTI inputs must be continuous time (Ts=0)'), end

[ma,na] = size(a);
[md,nd] = size(d);
[hhsv,p,q] = hksv(G);
%
if mrtype == 1
   kk = in+1;
end
%
if mrtype == 2
   tails = 0;
   kk = 1;
   for i = ma:-1:1
       tails = tails + hhsv(i);
       if 2*tails > in
          kk = i+1;
          break
       end
   end
end
%
if mrtype == 3
   in = input('Please assign the order of the K_th Hankel MDA: ');
   r = input('Please input the Multiplicity of the (K+1)th Hankel SV: ');
   kk = in+1;
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
% ---- Keep all the state:
%
if kk > na & sum(hhsv) > 1e-16
   ax = a; bx = b;
   cx = c; dx = d;
   ay = [];   by = [];
   cy = [];   dy = zeros(md,nd);
   aug = [0 0 0 hhsv'];
   Gx = ss(ax,bx,cx,dx);
   Gy = ss(ay,by,cy,dy);
   return 
end

%
% ------ Disgard all the states:
%
if sum(hhsv) < 1e-16
   ma = 0; na = 0;
   ax = zeros(0,0);   bx = zeros(0,nd);
  cx = zeros(md,0);   dx = d;
   ay = []; by = []; cy = []; dy = zeros(md,nd);
   bnd = 2*sum(hhsv);
   if isempty(hhsv)
       aug = [0 ma bnd hhsv'];
   else
       aug = [hhsv(1) ma bnd hhsv'];
   end
   Gx = ss(ax,bx,cx,dx);
   Gy = ss(ay,by,cy,dy);
   return
end

%
if isempty(hhsv)
    ro = 0;
else
    ro = hhsv(kk,1);
end
kp1 = 0;
r0 = 0; % recount the multiplicity of r
for i = 1 : na
    if hhsv(i,1) == ro
            r0 = r0+1;   % recount the multiplicity of r
    end
    if hhsv(i,1) < ro
        kp1 = kp1+1;
    end
end
r = r0; % recount the multiplicity of r
strm = r + kp1;
bnd = 2*sum([hhsv(kk);hhsv(kk+r:end)]);
aug = [hhsv(1,1) strm bnd hhsv'];
%
aa = ro^2*a' + q*a*p;
bb = q*b;
cc = c*p;
dd = d;
ee = q*p - ro*ro*eye(ma);
[u,s,v] = svd(ee);
%
rr  = length(find(diag(s)>sqrt(eps)));
rr2 = length(find(diag(s)>ro*ro));
if rr2 > rr
   rr = rr2;
end
%
u1 = u(:,1:rr);
u2 = u(:,rr+1:na);
v1 = v(:,1:rr);
v2 = v(:,rr+1:na);
%
sigi = diag(1./sqrt(diag(s(1:rr,1:rr))));  
a11 = u1'*aa*v1;
a12 = u1'*aa*v2;
a21 = u2'*aa*v1;
a22i = pinv(u2'*aa*v2);
b1 = u1'*bb; b2 = u2'*bb;
c1 = cc* v1; c2 = cc*v2;
%
axy = sigi*(a11 - a12*a22i*a21)*sigi;
bxy = sigi*(b1  - a12*a22i*b2);
cxy = (c1 - c2*a22i*a21)*sigi;
dxy = dd - c2 * a22i * b2;
%
sysxy = ss(axy,bxy,cxy,dxy);
[Gx,Gy,msat] = stabproj(sysxy);
if r == 1
    bnd = 2*sum(hhsv(msat+1:end));
    aug = [hhsv(1,1) strm bnd hhsv'];
end
if msat ~= (kk-1)
    disp(['Note: size(Gr) = ' num2str(msat) ', Order requested = ' num2str(kk-1) '. Reaching machine accuracy, see Help HANKELMR.'])
end
%
% ------ End of OHKAPP.M --- RYC/MGS %
