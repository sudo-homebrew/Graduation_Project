function sys = mkfilter(fc,ord,type,psbndr)
% MKFILTER  Construct a continuous-time filter.
%
% SYS = MKFILTER(FC,ORD,TYPE) creates a state-space single-input, 
% single-output continuous-time filter.  The cutoff frequency, in Hertz, 
% is defined by FC and the filter order is ORD.  The string variable, 
% TYPE, specifies the type of filter. Valid TYPEs are .
%
%    TYPE = 'Butterw'     Butterworth
%    TYPE = 'Cheby'       Chebyshev
%    TYPE = 'Bessel'      Bessel
%    TYPE = 'RC'          Series of resistor/capacitor filters
%
%  The dc gain of each filter (except even order Chebyshev)
%  is set to unity.
%
% SYS = MKFILTER(FC,ORD,TYPE,PSBNDR) 
% The argument PSBNDR specifies the Chebyshev passband ripple (in dB). 
% At the cutoff frequency, the magnitude is -PSBNDR dB.  For even 
% order Chebyshev filters the DC gain is also -PSBNDR dB.
%
% The Bessel filters are calculated using the recursive 
% polynomial formula.  This is poorly conditioned for high 
% order filters (order > 8).
%
%  See Also: FILTER

%   Copyright 2003-2010 The MathWorks, Inc.
narginchk(3,4)
type = ltipack.matchKey(type,{'butterw','bessel','cheby','rc'});
if isempty(type)
   error('Filter type must be one of the following strings: "butterw", "bessel", "cheby", "rc".')
end

omegac = 2*pi*fc;
switch type
   case 'butterw'
      dang = 2*pi/(2*ord);
      pang = (pi+dang)/2:dang:(3*pi-dang)/2;
      pls = omegac*exp(sqrt(-1)*pang);
      %--- new to handle numerical sensitivity issues --%
      [~,indx] = sort(real(pls));
      pls = pls(indx);
      indx = find(abs(imag(pls))<1e-12);
      pls(indx) = real(pls(indx));
      dindx = find(real(diff(pls))<1e-12);
      pls(dindx+1) = pls(dindx)';
      zrs = [];
      sys = zpk(zrs,pls,1);
   case 'bessel'
      b1 = [1/omegac,1];
      b2 = [1/omegac^2,3/omegac,1];
      if ord == 1
         sys = tf(1,b1);
      elseif ord == 2
         sys = tf(1,b2);
      else
         bn2 = b1;
         bn1 = b2;
         for i = 3:ord
            bn = [0, (2*i-1)*bn1] + [1/(omegac^2)*bn2,0,0];
            bn2 = bn1;
            bn1 = bn;
         end
         sys = tf(1,bn);
      end
   case 'cheby'
      if nargin ~= 4
         error('passband ripple must be specified for Chebyshev')
      end
      peps = sqrt(10^(abs(psbndr)/10) - 1);
      alpha = 1/peps + sqrt(1+1/peps^2);
      a = (alpha^(1/ord) - alpha^(-1/ord))/2;
      b = (alpha^(1/ord) + alpha^(-1/ord))/2;
      dang = 2*pi/(2*ord);
      pang = (pi+dang)/2:dang:(3*pi-dang)/2;
      ipls = imag(b*omegac*exp(sqrt(-1)*pang));
      rpls = (omegac*a)^2*(1 - (ipls/(omegac*b)).^2);
      rpls = -1*sqrt(rpls);
      pls = rpls + sqrt(-1)*ipls;
      %--- new to handle numerical sensitivity issues --%
      [~,indx] = sort(real(pls));
      pls = pls(indx);
      indx = find(abs(imag(pls))<1e-12);
      pls(indx) = real(pls(indx));
      dindx = find(real(diff(pls))<1e-12);
      pls(dindx+1) = pls(dindx)';
      zrs = [];
      sys = zpk(zrs,pls,1);
   case 'rc'
      pls = -1*ones(1,ord)*omegac;
      zrs = [];
      sys = zpk(zrs,pls,1);
end

%	set the dc gain to unity
[a,b,c,d] = ssdata(sys);
if strcmp(type,'cheby')
   fcmag = abs(d + c*((1i*omegac*eye(ord)-a)\b));
   gainfact = (1/sqrt(1+peps^2))/fcmag;
else
   gainfact = 1/(d-c/a*b);
end
gain = sqrt(abs(gainfact));
gainsgn = sign(gainfact);
b = b.*gain;
c = c.*gain.*gainsgn;
d = d.*gain.*gain.*gainsgn;
sys = ss(a,b,c,d);

