function [b,tm,psi,L,P,ejpsi] = d2dmax(t1,t2,theta)
% [b,tm,psi,L,P,ejpsi] = d2dmax(t1,t2,theta)
%
% Finds a first-order disk-to-disk mapping of the
% form q = (z+b)/(bz+1) =: K(z) such that:
%         1)   -1 < b < 1 (mapping maps inside to inside)
%         2)   z=e^(j*t1) and z=e^(j*t2) get mapped maximally far apart
%              from one another (over all choices of b).  This actually
%              puts their mapped values on opposite sides of e^(j*pi/2),
%              conjugate symmetric.
% It is assumed that 0 < T1 < T2 < pi.
% The value b is returned, and the angle TM (which is between T1 and T2)
% has the property that z=e^(j*TM) gets mapped to j.  For any values of
% THETA, PSI satisfies exp(j*PSI) = (exp(j*THETA)+b)./(b*exp(j*THETA)+1).
%
% The inverse mapping is z = (q-b)/(-bq+1) =: B(q)
% L is a 2x2 matrix, which satisfies F_u(L,1/z) = 1/q.
% P is a 2x2 matrix, which satisfies F_u(P,1/q) = 1/z.

%   Copyright 2003-2012 The MathWorks, Inc.

% In above, angle in the Z-plane is THETA, and angle in the Q-plane is PSI.

% If G(z) is star(1/z,M), then it is also G(B(q)) = star(1/q,star(P,M)).
% If H(q) is star(1/q,N), then it is also H(K(z)) = star(1/z,star(L,N)).
if 0<t1 && t1<t2 && t2<pi
   c1 = cos(t1);
   c2 = cos(t2);
   if abs(c1+c2)<1e-8
      % Condition means that the T1+T2 = pi, so 1+cos(T1+T2)==0,
      % Applying L'hospital to general formula gives result as below.
      tm = pi/2; % b = 0;
   else
      ct = (1+cos(t1+t2))/(c1+c2);
      tm = acos(ct);
   end
   [b,L,P] = d2dblt(tm);
   if nargin==3
      ejt = exp(1i*theta);
      ejpsi = (ejt+b)./(b*ejt+1);
      psi = abs(angle(ejpsi));  % watch for wrapping pi+eps -> -pi
   end
else
   error('The angles T1 and T2 must satisfy 0 < T1 < T2 < PI.');
end

% TEST PROGRAM
% skip = 1;
% if ~skip
% t1t2 = sort(pi*rand(1,2));
% t1 = t1t2(1);
% t2 = t1t2(2);
% [b,tm] = d2dmax(t1,t2);
% ejtm = exp(sqrt(-1)*tm);
% ejt1 = exp(sqrt(-1)*t1);
% ejt2 = exp(sqrt(-1)*t2);
% psim = angle((ejtm+b)./(b*ejtm+1));
% psi1o = angle((ejt1+b)./(b*ejt1+1));
% psi2o = angle((ejt2+b)./(b*ejt2+1));
% [(psi1o+psi2o) psim]  % should be [pi pi/2]
% b = linspace(-0.99,0.99,100);
% psi1 = angle((ejt1+b)./(b*ejt1+1));
% psi2 = angle((ejt2+b)./(b*ejt2+1));
% plot(1:100,psi2-psi1,1:100,repmat(psi2o-psi1o,1,100))  % curve just touches line
% end
