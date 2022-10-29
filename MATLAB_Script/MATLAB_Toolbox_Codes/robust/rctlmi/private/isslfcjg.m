function [bool,t]=isslfcjg(A,B)
% tests whether B = t.A or A=t.B (bool=1)

% Authors: P. Gahinet and A. Nemirovski 3/95
% Copyright 1995-2016 The MathWorks, Inc.
bool = 0;  t = 0;
if isequal(size(A),size(B))
   k = find(A,1);
   if isempty(k)
      % A=0
      bool = 1;
   else
      t = B(k)/A(k);
      if norm(B-t*A,1) <= 100*eps*norm(B,1)
         bool = 1;
      end
   end
end
