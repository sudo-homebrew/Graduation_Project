function [marg, freq] = ncfmargin(p,c,varargin)
%NCFMARGIN  Normalized coprime stability margin of feedback loop.
%
%   For the negative-feedback loop
%
%          u --->O--->[ C ]-->[ P ]---+---> y
%              - |                    |
%                +<-------------------+
%
%   NCFMARGIN computes the stability margin b(P,C) defined as the
%   reciprocal of
%
%       || [I;C] (I+P*C)^(-1) [I,P] ||oo  = 
%
%                 || [I;P] (I+C*P)^(-1) [I,C] ||oo
%
%   Note that b(P,C)>0 if and only if the feedback loop is stable.
%   
%   [MARG,FREQ] = NCFMARGIN(P,C) returns the robust stability margin 
%   MARG=b(P,C) and the frequency FREQ where the margin is achieved.
%
%   [MARG,FREQ] = NCFMARGIN(P,C,SIGN) specifies the feedback sign. Set
%   SIGN=-1 for negative feedback (default) and SIGN=+1 for positive
%   feedback.
%
%   [MARG,FREQ] = NCFMARGIN(P,C,SIGN,TOL) specifies the relative accuracy
%   TOL for the computed value MARG. By default MARG is computed with
%   0.1% accuracy (TOL=1e-3).
%
%   See also ncfsyn, gapmetric, lncf, rncf, diskmargin, getPeakGain.

%   Copyright 1991-2018 The MathWorks, Inc.
narginchk(2,4);

% validate plant and controller
% Note: Convert ss or frd to avoid loss of accuracy in, e.g., TF*TF
if isa(p,'FRDModel')
   p = frd(p);
elseif isnumeric(p) || isa(p,'DynamicSystem')
   p = ss(p);
else
   error(message('Robust:analysis:ncfmargin1'));
end
if isa(c,'FRDModel')
   c = frd(c);
elseif isnumeric(c) || isa(c,'DynamicSystem')
   c = ss(c);
else
   error(message('Robust:analysis:ncfmargin2'));
end

% check compatibility of the I/O sizes of plant and controller
[ny,nu] = iosize(p);
if ~isequal([nu ny],iosize(c))
   error(message('Robust:analysis:ncfmargin3'));
end

% get tolerance and sign
tol=1e-3;
sgn=-1;
for id=1:numel(varargin)
   prm = varargin{id};
   if isnumeric(prm) && isscalar(prm) && isreal(prm)
      prm = double(prm);
      if (prm>0) && (prm<1) % valid tolerance
         tol = prm;
      elseif (prm==1) || (prm==-1) % valid sign
         sgn = prm;
      else
         error(message('Robust:analysis:ncfmargin4'));
      end
   elseif ~isempty(prm)
      error(message('Robust:analysis:ncfmargin4'));
   end
end

% compute ncfmargin
try
   H = [eye(ny);c] * [p,eye(ny)];
   % CL = [I;C] inv(I-sign*PC) [P,I]
   cl = feedback( H , eye(nu), 1:nu, ny+1:ny+nu, sgn);
catch Me
   throw(Me)
end

% compute closed-loop norm
try
   [gpeak,freq] = hinfnorm(cl,tol);
catch
   % May fail due to internal delays
   error(message('Robust:analysis:ncfmargin6'));
end
marg = 1./gpeak;