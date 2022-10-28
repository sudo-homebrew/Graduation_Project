function [c,d] = sisormzero(a,b,c,d,z,m)

% Copyright 2003-2004 The MathWorks, Inc.

n = size(a,1);
for i=1:m
   [u,s,v] = svd([-z*eye(n)+a b;c d]);
   ds = diag(s);
   if min(ds)<=100*eps*max(ds)
      rowv = u(:,end)';
      c = -rowv(1:end-1)/rowv(end);
      d = 0;
   else
      error('Z is not a zero of SYS');
   end
end