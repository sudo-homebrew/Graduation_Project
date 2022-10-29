function g = genGain(A,B)
% Computes
%   
%     G = min GAM such that || A + B/GAM ||oo <= 1 
%
% where A and B are two matrices or two FRD models.

%   Copyright 2019 The MathWorks, Inc.
if isnumeric(A)
   g = localStaticPb(A,B);
else
   [A,f,Ts] = frddata(A);
   B = B.ResponseData;
   nf = numel(f);
   gam = zeros(nf,1);
   for ct=1:nf
      gam(ct) = localStaticPb(A(:,:,ct),B(:,:,ct));
   end
   g = frd(gam,w,Ts);
end

%----------------
function gam = localStaticPb(A,B)
% Solves static problem
if norm(A)>1
   gam = Inf;
else
   [p,m] = size(A);
   gam = -min(real(eig([zeros(p) B;B' zeros(m)],[eye(p) A;A' eye(m)])));
end


   







