function [af,bf,cf,df] = lqg(varargin)
%LQG Design of LQG regulators and servo-controllers.
%
%   KLQG = LQG(SYS,QXU,QWV) computes an optimal linear-quadratic Gaussian
%   (LQG) regulator KLQG given a state-space model SYS of the plant and
%   weighting matrices QXU and QWV.  The dynamic regulator KLQG uses the
%   measurements y to generate a control signal u that regulates y around
%   the zero value. Use positive feedback to connect this regulator to the
%   plant output y.
%
%                                     w |               | v
%                                       |   .-------.   |
%                       .--------.      '-->|       |   V
%               .------>|  KLQG  |--------->|  SYS  |---O--.-----> y
%               |       '--------'  u       |       |      |
%               |                           '-------'      |
%               |                                          |
%               '------------------------------------------'
%
%   The LQG regulator minimizes the cost function
%
%         J(u) = Integral [x',u'] * QXU * [x;u] dt
%
%   subject to the plant equations
%
%         dx/dt = Ax + Bu + w
%             y = Cx + Du + v
%
%   where the process noise w and measurement noise v are Gaussian white
%   noises with covariance:
%
%         E ([w;v] * [w',v']) = QWV.
%
%   LQG uses LQR and KALMAN to compute the LQG regulator. The state-space
%   model SYS should specify the A,B,C,D matrices, see SS for details.
%   LQG can be used for both continuous- and discrete-time plants. By
%   default, LQG uses x[n|n-1] as state estimate in discrete time, see
%   KALMAN for details. To use x[n|n] instead, type
%      KLQG = LQG(SYS,QXU,QWV,...,'current')
%
%   KLQG = LQG(SYS,QXU,QWV,QI) computes an LQG servo-controller KLQG that
%   uses the setpoint command r and measurements y to generate the control
%   signal u. KLQG has integral action to ensure that the output y tracks
%   the command r.
%
%                                       | w             | v
%                       .----------.    |   .-------.   |
%          r   -------->|          |    '-->|       |   V
%                       |   KLQG   |------->|  SYS  |---O--.-----> y
%          y   .------->|          |  u     |       |      |
%              |        '----------'        '-------'      |
%              |                                           |
%              '-------------------------------------------'
%
%   The LQG servo-controller minimizes the cost function
%
%         J(u) = Integral [x',u'] * QXU * [x;u] + xi' * Qi * xi dt
%
%   where xi is the integral of the tracking error r-y and SYS,w,v are as
%   described above. For MIMO systems, r, y, and xi must have the same
%   length. LQG uses the commands LQI and KALMAN to compute KLQG.
%
%   KLQG = LQG(SYS,QXU,QWV,QI,'1dof') computes a one-degree-of-freedom
%   servo-controller that takes e=r-y rather than [r;y] as input.
%
%   KLQG = LQG(SYS,QXU,QWV,QI,'2dof') is equivalent to LQG(SYS,QXU,QWV,QI)
%   and produces the two-degree-of-freedom servo-controller shown above.
%
%   [KLQG,INFO] = LQG(SYS,QXU,QWV,...) returns the controller and estimator
%   gains in the structure INFO. The controller equations are as follows:
%
%   Continuous:
%      dx_e = A x_e + B u + L (y - C x_e - D u)
%         u = - Kx x_e - Ki xi
%
%   Discrete:
%      x[n+1|n] = A x[n|n-1] + B u[n] + L (y[n] - C x[n|n-1] - D u[n])
%      delayed:  u[n] = - Kx x[n|n-1] - Ki xi[n]
%      current:  u[n] = - Kx x[n|n] - Ki xi[n] - Kw w[n|n]
%                     = - Kx x[n|n-1] - Ki xi[n] - (Kx*Mx+Kw*Mw) innov[n]
%                with innov[n] = y[n] - C x[n|n-1] - D u[n].
%
%   See also LQR, LQI, LQRY, KALMAN, SS, CARE, DARE.

%% Old Help
%LQG Continuous linear-quadratic-Gaussian control synthesis.
%
% [SS_F] = LQG(SS_,W,V) or
% [AF,BF,CF,DF] = LQG(A,B,C,D,W,V) computes Linear-Quadratic-Gaussian
%    optimal controller via the "separation principle", such that the
%    cost function
%                     T
%    J    = lim E{ int   |x' u'| W |x| dt} , W = |Q  Nc|, is minimized
%     LQG   T-->inf   0            |u|           |Nc' R|
%    and subject to the plant
%                       dx/dt = Ax + Bu + xi
%                           y = Cx + Du + th
%
%    where the white noises {xi} and {th} have the cross-correlation
%    function with intensity V, i.e.
%
%           E{|xi| |xi th|'} = V delta(t-tau), V =  |Xi  Nf|.
%             |th|                                  |Nf' Th|
%
%    The LQG optimal controller F(s) is returned in SS_F or (af,bf,cf,df).
%    The standard state-space can be recovered by "branch".

% R. Y. Chiang & M. G. Safonov 8/86
%   Copyright 1988-2009 The MathWorks, Inc.
nag1=nargin;
if nag1>0 && ~isa(varargin{1},'double')
   ctrlMsgUtils.error('Control:general:NotSupportedModelsofClass','lqg',class(varargin{1}));
end
[emsg,nag1,xsflag,Ts,A,B,C,D,W,V]=mkargs5x('ss',varargin); error(emsg);
if Ts, error('LTI inputs must be continuous time (Ts=0)'), end

%
[ns,ns] = size(A);
[nout,nin]= size(D);
%
% ------- Regular LQG problem:
%
Kf = lqrc(A',C',V); Kf = Kf';
Kc = lqrc(A,B,W);
%
% ------- Final Controller:
%
af = A - B*Kc - Kf*C + Kf*D*Kc;
bf = Kf;
cf = Kc;
df = zeros(nin,nout);
%
if xsflag
   af = mksys(af,bf,cf,df);
end
%
% ------- End of LQG.M -- RYC/MGS %
