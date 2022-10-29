function out = crand(dim1,dim2)
% out = crand(dim1,dim2)
%
%   Complex random matrix generator: generates a complex,
%   random matrix with a uniform distribution of dimension
%   DIM1 x DIM2. If only one dimension argument is given,
%   the output matrix is square.
%
%   See also CRANDN, RAND, RANDN and SYSRAND.

%   Copyright 2003-2011 The MathWorks, Inc.
nin = nargin;
if nin==2
   out = complex(rand(dim1,dim2),rand(dim1,dim2));
elseif nin==1
   out = complex(rand(dim1),rand(dim1));
else
   error('CRAND needs one or 2 positive integer arguments');
end