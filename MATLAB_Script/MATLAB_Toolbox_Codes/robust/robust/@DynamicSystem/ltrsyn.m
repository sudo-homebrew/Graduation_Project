function varargout = ltrsyn(varargin)
% LTRSYN performs LQG/LTR (loop-transfer recovery) control synthesis
%
% [K,SVL,W1] = ltrsyn(G,F,XI,THETA,RHO)
% [K,SVL,W1] = ltrsyn(G,F,XI,THETA,RHO,W)
% [K,SVL,W1] = ltrsyn(G,F1,Q,R,RHO,'OUTPUT')
% [K,SVL,W1] = ltrsyn(G,F1,Q,R,RHO,W,'OUTPUT')
%
% [K,SVL,W1] = ltrsyn(G,F,XI,THETA,RHO) computes a reconstructed
% -state output-feedback controller K for LTI plant G so that
% K*G asymptotically 'recovers' full-state feedback loop at the plant
% input; i.e., at each frequency, as RHO increases toward infinity,
%       K*G --> F*inv(Is-A)*B
%
% [K,SVL,W1] = ltrsyn(G,F1,Q,R,RHO,'OUTPUT') computes the solution
% to the ‘dual’ problem of Kalman filter loop recovery given a
% Kalman filter gain matrix F1; i.e.,
%       G*K --> C*inv(Is-A)*F1
% where Q and R are state and control cost matrices (see LQR).
%
% Sigma plots (or, for SISO G only, Nyquist plots) of 'recovered' K*G
% (or, if OPT 'OUTPUT', G*K) loop-transfer functions are displayed.
%
% Inputs:
%     G       -- LTI plant
%  F,XI,THETA -- LQ full-state-feedback gain matrix F
%                plant noise intensity XI,
%                sensor noise intensity THETA
%  F1, Q, R   -- Kalman filter gain F1,
%                plant cost matrix Q,
%                control cost matrix R
%     RHO     -- a monotone increasing row vector containing a set
%                of positive recovery gains
% Optional Input:
%      W      -- vector of non-negative frequencies (to be used
%                for plots); if input W is not supplied, then a
%                reasonable default is automatically determined
%   'OUTPUT'  -- C*inv(Is-A)*F1 loop-recovery at plant output; if
%                present, 'OUTPUT' must be the last input argument
%  Outputs:
%    K   -- 'recovered' LTI output-feedback, for final RHO(end) only
%    SVL -- sigma plot data, if MIMO, or, if G is SISO,
%           Nyquist loci SVL = [re(1:nr) im(1:nr)]
%    W1  -- frequencies for SVL plots, same as W when present
%
% Limitation:  The plant D-matrix must be all zeros. LTRSYN requires a
%    continuous-time LTI system G; it cannot be with discrete-time G
%
% Comments:
%    The controller K is by default
%          K = ss(A-B*F-KF*C,KF,F,zeros(size(D)))
%       where KF=lqr(A,B,XI+RHO(end)*C'*C,THETA)  is an optimal
%       LQ state feedback and [A,B,C,D]=ssdata(G)
%    If 'OUTPUT' is the last input argument, then the controller is
%          K = ss(A-B*KC-F*C,F,KC,zeros(size(D)))
%       where KC=lqr(A',C',XI+RHO(end)*B*B,THETA)' is an optimal
%       Kalman-Bucy filter gain
%
% See also LQG LQR LQE KALMAN SIGMA

% Copyright 2003-2005 The MathWorks, Inc.
[varargin{:}] = convertStringsToChars(varargin{:});
% initialize
DocKeys={'INPUT ', 'OUTPUT '};
UndocKeys={'LTRY ','LTRU ', 'Y ', 'U ', 'KALMAN '};

% Error checking
if nargin<5,
   error('Too few input arguments')
end
if nargout>3,
   error('Too many output arguments'),
end
G=ss(varargin{1});
Ts=getTs(G);
D=get(G,'d');
if Ts~=0,
   error('LTRSYN method is for continuous-time plants only.  You must have G.Ts==0')
end
if any(any(D)),
   disp('         Plant G has a non-zero D-matrix.')
   error('LTRSYN is intended only for SS plants with a zero D matrix. You must have norm(G.D)==0');
end

% Determine value of OPT
test=ischar(varargin{end});
if test,
   opt=varargin{end};
   varargin=varargin(1:end-1);
else
   opt='INPUT'; % INPUT LTRU
end
opt=keymatch(opt,DocKeys,UndocKeys);

if length(varargin) < 6,
   [~,w]=sigma(varargin{1});
   varargin=[varargin, {w}];
end

% now call either LTRU or LTRY, depending on OPT
switch opt
   case {'OUTPUT ', 'LTRY ', 'Y ', 'KALMAN '}
      varargout=cell(1,nargout);
      if nargout,
         [varargout{:}] = ltry(varargin{:});
      else
         ltry(varargin{:});
      end
   case {'INPUT ', 'LTRU ', 'U '}
      varargout=cell(1,nargout);
      if nargout,
         [varargout{:}] = ltru(varargin{:});
      else
         ltru(varargin{:});
      end
end
if nargout>2,
   varargout{3}=varargin{6};  % frequencies w
end