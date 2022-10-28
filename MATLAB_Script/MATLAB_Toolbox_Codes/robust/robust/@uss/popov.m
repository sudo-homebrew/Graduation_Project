function [varargout] = popov(usys,varargin)
%POPOV  Test for robust stability with Popov criterion.
%
%   [TAU,P,S,N] = popov(USYS,FLAG) uses the Popov criterion
%   to test if the uncertain model USYS is robustly stable.
%   Stability is established when TAU < 0, in which case POPOV
%   returns a Lyapunov matrix P and multipliers S and N proving
%   stability.
%
%   Inputs:
%    USYS     Uncertain continuous-time state-space model (see USS)
%    FLAG     optional, 0 by default. Setting FLAG=1 reduces
%             conservatism for real parameter uncertainty at
%             the expense of more intensive computations.
%
%   Outputs:
%    TAU      Optimal largest eigenvalue of the corresponding
%             LMI feasibility problem.  Robust stability is
%             guaranteed when TAU < 0, i.e., when the Popov
%             LMIs are feasible
%    P        x'*P*x  is the quadratic part of the Lyapunov
%             function proving stability
%    S,N      if TAU < 0, S and N are the Popov "multipliers"
%             proving stability.
%
%   See also QUADSTAB, PDLSTAB, USS.

%   Author: P. Gahinet  10/94
%   Copyright 1995-2011 The MathWorks, Inc.
if usys.Ts~=0
   error('Only applicable to continuous-time uncertain models')
end

% Extract uncertainty
[sys,Delta,BlkStruct] = lftdata(usys); 
if isempty(BlkStruct)
   error('Not applicable to models without uncertainty.')
end

% Check for arrays
sz = size(sys);
if prod(sz(3:end))>1
   error('popov does not work for uss arrays');
end
% Convert to obsolete representation
[a,b,c,d,e] = dssdata(sys);
sys = ltisys(a,b,c,d,e);

ublks = cell(1,0);
for ct=1:length(BlkStruct)
   b = BlkStruct(ct);
   switch b.Type
      case 'ureal'
         % Real scalar
         ub = {ublock(b.Size*b.Occurrences,1,'ltisr')};
      case 'ucomplex'
         % Complex scalar
         ub = {ublock(b.Size*b.Occurrences,1,'ltisc')};
      case {'ultidyn','ucomplexm'}
         % LTI dynamics
         if all(b.Size==1)
            ub = {ublock(b.Size*b.Occurrences,1,'ltisc')};
         else
            ub = cell(1,0);
            for cto=1:b.Occurrences
               ub{cto,1} = ublock(b.Size,1,'ltifc');
            end
         end
      case 'udyn'
         % Arbitrary dynamics
         if all(b.Size==1)
            ub = {ublock(b.Size*b.Occurrences,1,'nlsc')};
         else
            ub = cell(1,0);
            for cto=1:b.Occurrences
               ub{cto,1} = ublock(b.Size,1,'nlfc');
            end
         end
   end
   ublks = [ublks,ub];
end
Delta = udiag(ublks{:});
         
% Call POPOV
try
   [varargout{1:nargout}] = popov(sys,Delta,varargin{:});
catch
   rethrow(lasterror)
end
