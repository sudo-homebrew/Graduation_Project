function soln= sdn_step(a,b1,b2,c1,c2,invT,T,S,s,h,delay,k)

% SOLN contains 0 if gamma is too small
%               1 if norm less than gamma
%

%   Copyright 1991-2004 MUSYN Inc. and The MathWorks, Inc.

% construct equivalent discrete system
[ad,bd,cd,dd,pass] = rctutil.sdequiv(a,b1,b2,c1,c2,invT,T,S,s,h,delay);
if ~pass
   soln=0;
else
   row2=length(c2(:,1)); col2=length(b2(1,:));
   clsys=lft(ss(ad,bd,cd,dd,h),k,col2,row2);
   gam = getPeakGain(clsys,1e-6);
   soln = (gam<1);
end
