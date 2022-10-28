function [alpha,L,P] = hp2dblt(wm)
% Finds a first-order halfplane-to-disk mapping of the
% form z = (-s-alpha)/(s-alpha) =: K(s) such that:
%         1)   alpha>0 (mapping maps Re(s)<0 to |z|<1)
%         2)   s=j*wm gets mapped to j.
% It is assumed that 0 < wm < INF.
% The value ALPHA is returned.
%
% The inverse mapping is s = alpha(z-1)/(z+1) =: B(z)
% L is a 2x2 matrix, which satisfies F_u(L,1/s) = 1/z.
% P is a 2x2 matrix, which satisfies F_u(P,1/z) = 1/s.

% Copyright 2003-2004 The MathWorks, Inc.

% In above, frequency in S-plane is W, angle in the Z-plane is THETA.

% If G(s) is star(1/s,M), then it is also G(B(z)) = star(1/z,star(P,M)).
% If H(z) is star(1/z,N), then it is also H(K(s)) = star(1/s,star(L,N)).


if wm>0 && ~isinf(wm)
   alpha = wm;
   P = [1 sqrt(2/alpha);sqrt(2/alpha) 1/alpha];
   L = [-alpha sqrt(2*alpha);sqrt(2*alpha) -1];
else
   error('The frequency WM must satisfy 0 < WM < INF.');
end
