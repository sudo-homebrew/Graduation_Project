function [alpha,wm,theta,L,P,ejtheta] = hp2dmax(w1,w2,w)
% [alpha,wm,theta,L,P,ejtheta] = hp2dmax(w1,w2,w)
%
% Finds a first-order halfplane-to-disk mapping of the
% form z = (-s-alpha)/(s-alpha) =: K(s) such that:
%         1)   alpha>0 (mapping maps Re(s)<0 to |z|<1)
%         2)   s=j*w1 and s=j*w2 get mapped maximally far apart
%              from one another (over all choices of alpha).  This actually
%              puts their mapped values on opposite sides of e^(j*pi/2),
%              conjugate symmetric.
% It is assumed that 0 < w1 < w2 < INF.
% The value ALPHA is returned, and the frequency WM (which is between W1 and W2)
% has the property that s=j*WM gets mapped to j.  For any values of
% W, THETA satisfies exp(j*THETA) = (-j*W-ALPHA)./(j*W-ALPHA).
%
% The inverse mapping is s = alpha(z-1)/(z+1) =: B(z)
% L is a 2x2 matrix, which satisfies F_u(L,1/s) = 1/z.
% P is a 2x2 matrix, which satisfies F_u(P,1/z) = 1/s.

% Copyright 2003-2004 The MathWorks, Inc.

% In above, frequency in S-plane is W, angle in the Z-plane is THETA.

% If G(s) is star(1/s,M), then it is also G(B(z)) = star(1/z,star(P,M)).
% If H(z) is star(1/z,N), then it is also H(K(s)) = star(1/s,star(L,N)).

if all(diff([0 w1 w2 inf])>0)
   wm = sqrt(w1*w2);
   [alpha,L,P] = hp2dblt(wm);
   if nargin==3
      jw = sqrt(-1)*w;
      ejtheta = (-jw-alpha)./(jw-alpha);
      theta = angle(ejtheta);
   end
else
   error('The frequencies W1 and W2 must satisfy 0 < W1 < W2 < INF.');
end
