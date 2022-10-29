function [gr,gfc] = gfactorg(gm)
% Factors the fits for (square) full G scaling matrices.
% Produces stable SYSTEM matrices GR and GFC such that
%    GR*GR' = (I+G'*G)^(-1)
%    GFC = G*(I+G'*G)^(-1/2) = G*GR;

%   Author(s): P. M. Young, 1996.
%   Copyright 2009-2018 The MathWorks, Inc.
[a,b,c,d] = ssdata(gm); 
n = size(a,1);
m = size(d,1);
[u,s,v] = svd(d);
s = diag(s);
t = sqrt(1+s.^2);
dm1 = v * (t.\v');       % (I+d'*d)^(-1/2)
dm2 = u * ((s./t).*v');  % d*(I+d'*d)^(-1/2)
if isempty(a)
   am = [];  bm = [];  k1 = [];  k2 = [];
else
   r = [eye(m) d';d -eye(m)];
   [x,k] = icare(a,[b zeros(n,m)],0,r,[zeros(n,m) c']);
   if ~(size(x,1)==n && all(isfinite(x(:))))
      error('jw axis eigenvalues in Hamiltonian for (I+G*G)^(1/2)')
   end
   k1 = k(1:m,:);        % (I+d'*d)\(b'*x+d'*c)
   k2 = k(m+1:2*m,:);  % (I+d*d')\(d*b'*x-c)
   am = a-b*k1;
   bm = b*dm1;
end
gr = ss(am,bm,-k1,dm1);
gfc = ss(am,bm,-k2,dm2);