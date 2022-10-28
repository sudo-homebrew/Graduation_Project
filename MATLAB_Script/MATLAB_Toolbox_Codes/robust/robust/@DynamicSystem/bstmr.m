function varargout = bstmr(G,varargin)
%BSTMR  Model reduction using balanced stochastic truncation (BST).
%
% [GRED,REDINFO] = BSTMR(G,ORDER,KEYWORD,VALUE) performs a relative error 
% model reduction on LTI object G. The infinity-norm of the relative error 
% is bounded as:
%           -1                           n
%      |GRED (s)(GRED(s)-G1(s))|    <=  prod (1+si)/(1-si) - 1
%                               inf      k+1
% where si denotes the i-th Hankel singular value of the all-pass "phase 
% matrix" of G(s). The algorithm is based on Balanced Stochastic Truncation 
% (BST) theory with Relative Error Method (REM).
%
% For unstable G the algorithm works by first splitting G into a sum of stable 
% and antistable part, reduces the stable part, then add the unstable part back 
% for the final output G. The unstable part is kept as default.
%
% Inputs:
%    G  - LTI system to be reduced
%
% Optional inputs:
%    ORDER - an integer array with desired order of reduced model. 
%		 A batch run of [m:n] can be specified for a pack of reduced 
% 		 order models to be generated. The unstable part is kept as
% 		 default.
%
%   KEY       |VALUE        | MEANING
%  -----------------------------------------------------------------------
%  'MaxError' |real no. or  | Reduce to achieve H-infinity error
%             |vector       | If present, 'MaxError' overrides ORDER input.
%  ------------------------------------------------------------------------
%  'Display'  |'off' or 'on'| Display HANKELSV plots (default 'off')
%
%  Outputs:
%    Gred    - LTI reduced order system
%    REDINFO - a struct of REDINFO.ErrorBound, REDINFO.StabSV,
%    REDINFO.Unstab.SV
%
% See also REDUCE, BALRED, NCFMR.

% Copyright 1988-2020 The MathWorks, Inc.

% Options
[varargin{:}] = convertStringsToChars(varargin{:});
ni = nargin-1;
no = nargout;
ORDERS = [];
MAXERR = [];
DISPLAY = (nargout==0);
if ni>0 && isnumeric(varargin{1})
   ORDERS = varargin{1};
   varargin = varargin(2:end);
   ni = ni-1;
end
for k=1:floor(ni/2)
   switch lower(varargin{2*k-1}(1))
      case 'd' % display
         DISPLAY = strcmpi(varargin{2*k},'on');
      case 'o' % order
         ORDERS = varargin{2*k};
      case 'm'
         MAXERR = varargin{2*k};
   end
end

% Split into stable, anti-stable, and undamped parts
opt = balredOptions('ErrorBound','rel','StateProjection','t');
try
   [H,H0] = modsep(ss(G),3,@(p) localAssignRegion(p,G.Ts,opt.Offset));
catch ME
   throw(ME)
end
G1 = subparen(H,{':',':',1});  G1.D = H0.D;  % stable
G2 = subparen(H,{':',':',2});  % anti-stable
G3 = subparen(H,{':',':',3});  % modes on jw-axis/unit circle
nx2 = order(G2);  nx3 = order(G3);  nns = nx2+nx3;

% Analyze stable part
[~,balInfo] = balred(G1,opt);
g1 = balInfo.HSV;
err = [inf(nns,1) ; balInfo.ErrorBound ; 0];

% Resolve orders
nr = numel(MAXERR);
if nr>0
   ORDERS = zeros(nr,1);
   for ct=1:nr
      ORDERS(ct) = find(err<=MAXERR(ct),1);
   end
end

% Compute HSVs of anti-stable part
if DISPLAY || no>1
   [~,info2] = balred(G2',opt);
   g2 = info2.HSV(1:nx2,:);  % DT conjugation adds states
   % For historic reasons, jw-axis modes contribute Inf to g1
   g1 = [inf(nx3,1) ; g1];
end

% Plot HSV
if DISPLAY
   ax = gca;
   cla(ax)
   nx1 = numel(g1);
   matlab.graphics.chart.primitive.Bar('XData',nx2+1:nx2+nx1,'YData',g1',...
      'Parent', ax, 'EdgeColor',[0 0 .6],'FaceColor',[0 0 .6]);
   if nx2>0
      matlab.graphics.chart.primitive.Bar('XData',1:nx2,'YData',g2',...
         'Parent', ax, 'EdgeColor',[1 0 0],'FaceColor',[1 0 0]);
      legend('HSV (stable Part)','HSV (unstable Part)')
   end
   ax.YScale = 'log';
   grid(ax,'on')
   title(ax,'Hankel Singular Values of Phase Matrix')
   xlabel(ax,'Order')
   ylabel(ax,'Hankel Singular Values');
end

% Compute reduced models and error bounds
if no>0
   if isempty(ORDERS)
      Gred = [];
   else
      Gred = balred(G1,max(0,ORDERS-nns),balInfo,opt) + (G2+G3);
   end
   if no>1
      info = struct('ErrorBound',err(ORDERS+1,:),'StabSV',g1,'UnstabSV',g2);
      varargout = {Gred,info};
   else
      varargout = {Gred};
   end
end

%---------------------------------------------------------------------
function r = localAssignRegion(p,Ts,Offset)
% Split consistent with stabsepOptions.
if Ts==0
   rp = real(p);
   thresh = Offset * max(1,abs(p));
   if rp<-thresh
      r = 1;
   elseif rp>thresh
      r = 2;
   else
      r = 3;
   end
else
   ap = abs(p);
   if ap<1-Offset
      r = 1;
   elseif ap>1+Offset
      r = 2;
   else
      r = 3;
   end
end