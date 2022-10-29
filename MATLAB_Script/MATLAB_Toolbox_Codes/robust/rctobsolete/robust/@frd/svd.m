%SVD  Singular value decomposition, for FRD models
%   [U,S,V] = SVD(SYS) computes the singular value decomposition of the
%   frequency response contained in the FRD model SYS.  The outputs U, S
%   and V are FRD objects containing the singular values and vectors across 
%   frequencies.  Cycling over frequency, the command is logically the same
%   [FSELECT(U,I),FSELECT(S,I),FSELECT(V,I)] = SVD(FSELECT(SYS,I)).
%  
%   S = SVD(X) returns a vector containing the singular values.
%  
%   [U,S,V] = SVD(X,0) produces the "economy size"
%   decomposition. If X is m-by-n with m > n, then only the
%   first n columns of U are computed and S is n-by-n.
%  
%   See also SVDS, GSVD.
% 
%Overloaded methods
%   help frd/svd.m
function [u,s,v] = svd(m,flg)

%   Copyright 2003-2011 The MathWorks, Inc.

szm = size(m);
% Data
m = absorbDelay(m);
FUnit = m.FrequencyUnit;
[ResponseData,Frequency,ts] = frdata(m);
Nfreq = length(Frequency);
u = [];
s = [];
v = [];

if nargout > 1 && nargin == 1
   u = zeros([szm(1) szm(1) Nfreq szm(3:end)]);
   s = zeros([szm(1) szm(2) Nfreq szm(3:end)]);
   v = zeros([szm(2) szm(2) Nfreq szm(3:end)]);
	for k = 1:prod(szm(3:end))
       for i = 1:Nfreq
          [u1,s1,v1] = svd(ResponseData(:,:,i,k));
          u(:,:,i,k) = u1;
          s(:,:,i,k) = s1;
          v(:,:,i,k) = v1;
       end
	end
   u = frd(u,Frequency,ts,'FrequencyUnit',FUnit);
   s = frd(s,Frequency,ts,'FrequencyUnit',FUnit);
   v = frd(v,Frequency,ts,'FrequencyUnit',FUnit);
elseif nargout > 1 && nargin == 2
   if szm(2)>=szm(1)
       u = zeros([szm(1) szm(1) Nfreq szm(3:end)]);
       s = zeros([szm(1) szm(2) Nfreq szm(3:end)]);
       v = zeros([szm(2) szm(2) Nfreq szm(3:end)]);
    else
       u = zeros([szm(1) szm(2) Nfreq szm(3:end)]);
       s = zeros([szm(2) szm(2) Nfreq szm(3:end)]);
       v = zeros([szm(2) szm(2) Nfreq szm(3:end)]);
    end
	 for k = 1:prod(szm(3:end))
       for i = 1:Nfreq
          [u1,s1,v1] = svd(ResponseData(:,:,i,k),flg);
          u(:,:,i,k) = u1;
          s(:,:,i,k) = s1;
          v(:,:,i,k) = v1;
       end
	 end
    u = frd(u,Frequency,ts,'FrequencyUnit',FUnit);
    s = frd(s,Frequency,ts,'FrequencyUnit',FUnit);
    v = frd(v,Frequency,ts,'FrequencyUnit',FUnit);
elseif nargout == 1 
   msz = min([szm(1) szm(2)]);
   s = zeros([msz 1 Nfreq szm(3:end)]);
	for k = 1:prod(szm(3:end))
       for i = 1:Nfreq
          s(:,:,i,k) = svd(ResponseData(:,:,i,k));
       end
	end
   u = frd(s,Frequency,ts,'FrequencyUnit',FUnit);
end
