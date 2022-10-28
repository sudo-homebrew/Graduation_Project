function [b,L,P] = d2dblt(tm)
% Finds a first-order disk-to-disk mapping of the
% form q = (z+b)/(bz+1) =: K(z) such that:
%         1)   -1 < b < 1 (mapping maps inside to inside)
%         2)   z=e^(j*tm) gets mapped to j.
% It is assumed that 0 < TM < pi.
%
% The inverse mapping is z = (q-b)/(-bq+1) =: B(q)
% L is a 2x2 matrix, which satisfies F_u(L,1/z) = 1/q.
% P is a 2x2 matrix, which satisfies F_u(P,1/q) = 1/z.

% Copyright 2003-2004 The MathWorks, Inc.

% In above, angle in the Z-plane is THETA, and angle in the Q-plane is PSI.

% If G(z) is star(1/z,M), then it is also G(B(q)) = star(1/q,star(P,M)).
% If H(q) is star(1/q,N), then it is also H(K(z)) = star(1/z,star(L,N)).

if all(diff([0 tm pi])>0)
   cm = cos(tm);
   sm = sin(tm);
   if abs(cm)<10*eps
      b = 0;
   else
      b = (sm-1)/cm;
   end
   L = [-b sqrt(1-b^2);sqrt(1-b^2) b];
   P = [b sqrt(1-b^2);sqrt(1-b^2) -b];
else
   error('The angle TM must satisfy 0 < TM < PI.');
end
