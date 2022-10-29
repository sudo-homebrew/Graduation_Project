function [muVal,Delta] = special2by2MU(M)
%

%   Copyright 1986-2020 The MathWorks, Inc.

% M*Delta has one eval at 1, other free
% M22*d2 + M11*d1 = 1 + det(M)*d1*d2

% d1 in terms of d2
% d1 = (M22*d2 - 1)/(det(M)*d2 - M11)   % deno=0 special case
% M11/detM is real and equals 1/M22 (equaling d2)
% M22 must be real, nonzero; and det(M)=M11*M22, M12 =0 or M12=0;

%  d2 = M11/det(M), special

% "real"-M case
% (+-)d2 = (M22*d2 - 1)/(det(M)*d2 - M11)
%      (+-)detM*d2^2 - ((+-)M11+M22)*d2  + 1 = 0;

% RHS needs to be real: (a+jb) = (c+jd)*real = c*real + jd*real
%   a = c*real, b = d*real   a/c=b/d  ->   ad-bc = 0;
%   a = rM22*d2-1, b = iM22*d2, c = rdetM*d2-rM11, d = idetM*d2-iM11
%   ad - bc = 0  --->   quadratic in d2
%  quadC = idetM*rM22 - iM22*rdetM
%  linC = rDetM-idetM - iM11*rM22 + rM11*iM22
%  conC = iM11 - rM11

detM = det(M);
rdetM = real(detM);
idetM = imag(detM);
rM11 = real(M(1,1));
rM22 = real(M(2,2));
iM11 = imag(M(1,1));
iM22 = imag(M(2,2));

if iM22==0 && (M(1,2)==0 || M(2,1)==0)
   muVal = abs(rM22);
   Delta = diag([0 1/rM22]);
elseif iM11==0 && (M(1,2)==0 || M(2,1)==0)
   % handled below too, but handle directly here
   muVal = abs(rM11);
   Delta = diag([1/rM11 0]);
elseif abs(iM11)<10*eps && abs(iM22)<10*eps && abs(idetM)<10*eps
   % +
   d2PossibleP = roots([rdetM -(rM11+rM22) 1]);
   d1PossibleP = d2PossibleP;
   % - 
   d2PossibleN = roots([-rdetM -(-rM11+rM22) 1]);
   d1PossibleN = -d2PossibleN;
   
   d2Possible = [d2PossibleP;d2PossibleN];
   idx = find(imag(d2Possible)~=0);
   d1Possible = [d1PossibleP;d1PossibleN];
   d1Possible(idx) = [];
   d2Possible(idx) = [];
   [V,I] = min(max([abs(d1Possible), abs(d2Possible)],[],2));
   muVal = 1/V;
   Delta = diag([d1Possible(I), d2Possible(I)]);
else
   quadC = idetM*rM22 - iM22*rdetM;
   linC = -idetM - iM11*rM22 + rM11*iM22;
   conC = iM11;
   if linC^2-4*quadC*conC>=0
      d2Possible = roots([quadC linC conC]);
      d1Possible = (M(2,2)*d2Possible-1)./(detM*d2Possible - M(1,1));
      d1Possible = real(d1Possible);
      [V,I] = min(max([abs(d1Possible), abs(d2Possible)],[],2));
      muVal = 1/V;
      Delta = diag([d1Possible(I), d2Possible(I)]);
   else
      muVal = 0;
   end
end
if muVal==0
   Delta = [];
end
   
   

