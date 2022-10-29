function [hhsv,p,q] = hksv(a,b,c)
%HKSV Hankel singular values and grammians P, Q.
%
% [HHSV,P,Q] = HKSV(A,B,C) computes reachability and observability
%    grammians P, Q, and the Hankel singular values "hsv".
%
%    For unstable system, (-a,-b,c) is used instead and
%    "hsv = [hsv_stable;hsv_unstable]".

% R. Y. Chiang & M. G. Safonov 3/86
%   Copyright 1988-2006 The MathWorks, Inc.
% All Rights Reserved.
% ------------------------------------------------------------------

hsv=[]; p=NaN*a;q=p;  % initialize variables

lamda = eig(a);
ra = size(a)*[1;0];
indstab = find(real(lamda) < 10.e-16);
m = length(indstab);
if m == ra       % completely stable
  % p = gram(a,b);
  % q = gram(a',c');
  p = lyap(a,b*b');
  q = lyap(a',c'*c);
   [up,sp,vp] = svd(p);
   lr = up*diag(sqrt(diag(sp)));
   [uq,sq,vq] = svd(q);
   lo = uq*diag(sqrt(diag(sq)));
   hhsv = svd(lo'*lr);
elseif m == 0    % completely unstable
%   p = gram(-a,-b);
%   q = gram(-a',c');
    p = lyap(-a,b*b');
    q = lyap(-a',c'*c);
   [up,sp,vp] = svd(p);
   lr = up*diag(sqrt(diag(sp)));
   [uq,sq,vq] = svd(q);
   lo = uq*diag(sqrt(diag(sq)));
   hhsv = svd(lo'*lr);
elseif m > 0 & m < ra
   ddummy = zeros(size(b)*[0;1],size(c)*[1;0]);
   [al,bl,cl,dl,ar,br,cr,dr,msta] = stabproj(a,b,c,ddummy);
   % stable
%   pl = gram(al,bl);
%   ql = gram(al',cl');
    pl = lyap(al,bl*bl');
    ql = lyap(al',cl'*cl);
   [up,sp,vp] = svd(pl);
   lr = up*diag(sqrt(diag(sp)));
   [uq,sq,vq] = svd(ql);
   lo = uq*diag(sqrt(diag(sq)));
   hsvl = svd(lo'*lr);
   % unstable
%   pr = gram(-ar,-br);
%   qr1 = gram(-ar',cr');
   pr = lyap(-ar,br*br');
   qr1 = lyap(-ar',cr'*cr);
   [up,sp,vp] = svd(pr);
   lr = up*diag(sqrt(diag(sp)));
   [uq,sq,vq] = svd(qr1);
   lo = uq*diag(sqrt(diag(sq)));
   hsvr = svd(lo'*lr);
   hhsv = [hsvr;hsvl];
end
%
% -------- End of HKSV.M --- RYC/MGS
