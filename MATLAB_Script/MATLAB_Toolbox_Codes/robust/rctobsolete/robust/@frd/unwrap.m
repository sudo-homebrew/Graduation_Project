% UNWRAP Unwrap phase angle.
%  UNWRAP(P) unwraps radian phases P by changing absolute 
%  jumps greater than pi to their 2*pi complement. It unwraps 
%  along the first non-singleton dimension of P. P can be a 
%  scalar, vector, matrix, N-D array or FRD. 
% 
%  UNWRAP(P,TOL) uses a jump tolerance of TOL rather
%  than the default TOL = pi.
% 
%  UNWRAP(P,[],DIM) unwraps along dimension DIM using the
%  default tolerance. UNWRAP(P,TOL,DIM) uses a jump tolerance
%  of TOL.
% 
%  See also ANGLE, ABS.

function m = unwrap(m,varargin)

%   Copyright 2003-2011 The MathWorks, Inc.

m = absorbDelay(m);
szm = size(m);
% Data
[mResponseData,Frequency,~] = frdata(m);
Nfreq = length(Frequency);
ResponseData = zeros([szm(1:2) Nfreq szm(3:end)]);

for k = 1:prod(szm(3:end))
   for i = 1:szm(1)
      for j = 1:szm(2)
         ResponseData(i,j,:,k) = unwrap(mResponseData(i,j,:,k),varargin{:});
      end
   end
end
m.ResponseData = ResponseData;
