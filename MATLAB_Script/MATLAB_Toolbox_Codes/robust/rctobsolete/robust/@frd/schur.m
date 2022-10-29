function [out1,out2] = schur(m,type)
% SCHUR  Schur decomposition.
%  [U,T] = SCHUR(X) produces a quasitriangular Schur matrix T and
%  a unitary matrix U so that X = U*T*U' and U'*U = EYE(SIZE(U)).
%  X must be square.
% 
%  T = SCHUR(X) returns just the Schur matrix T.
% 
%  If X is complex, the complex Schur form is returned in matrix T.
%  The complex Schur form is upper triangular with the eigenvalues
%  of X on the diagonal.
% 
%  If X is real, two different decompositions are available.
%  SCHUR(X,'real') has the real eigenvalues on the diagonal and the
%  complex eigenvalues in 2-by-2 blocks on the diagonal.
%  SCHUR(X,'complex') is triangular and is complex if X has complex
%  eigenvalues.  SCHUR(X,'real') is the default.
% 
%  See RSF2CSF to convert from Real to Complex Schur form.
% 
%  See also ORDSCHUR, QZ.

%   Copyright 2003-2011 The MathWorks, Inc.

m = absorbDelay(m);
FUnit = m.FrequencyUnit;
szm = size(m);
% Data
[mResponseData,Frequency,Ts] = frdata(m);
Nfreq = length(Frequency);
out1 = [];
out2 = [];
if nargin == 1
   type = [];
end

if nargout == 1
   out1ResponseData = zeros([szm(1:2) Nfreq szm(3:end)]);
	for k = 1:prod(szm(3:end))
       for i = 1:Nfreq
          if isempty(type)
             out1 = schur(mResponseData(:,:,i,k));
          else
             out1 = schur(mResponseData(:,:,i,k),type);
          end
          out1ResponseData(:,:,i,k) = out1;
       end
	end
   out1 = frd(out1ResponseData,Frequency,Ts,'FrequencyUnit',FUnit);
elseif nargout == 2
   out1ResponseData = zeros([szm(1:2) Nfreq szm(3:end)]);
   out2ResponseData = zeros([szm(1:2) Nfreq szm(3:end)]);
	for k = 1:prod(szm(3:end))
       for i = 1:Nfreq
          if isempty(type)
             [out1,out2] = schur(mResponseData(:,:,i,k));
          else
             [out1,out2] = schur(mResponseData(:,:,i,k),type);
          end
          out1ResponseData(:,:,i,k) = out1;
          out2ResponseData(:,:,i,k) = out2;
       end
	end
   out1 = frd(out1ResponseData,Frequency,Ts,'FrequencyUnit',FUnit);
   out2 = frd(out2ResponseData,Frequency,Ts,'FrequencyUnit',FUnit);
end
