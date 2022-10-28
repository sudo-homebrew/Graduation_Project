function [Gr,info,GLr] = ncfmr(G,varargin)
%NCFMR  Model reduction from normalized coprime factorization.
%
%   NCFMR computes a reduced-order coprime factorization of the full-order
%   model. This method handles both stable and unstable plants. It is 
%   well-suited to controller order reduction by guaranteeing stability as
%   long as the approximation error is smaller than the robustness margin
%   computed by NCFMARGIN.
%
%   [GRED,INFO] = NCFMR(G,ORD) computes a reduced-order approximation
%   GRED of the LTI model G. The desired order (number of states) is
%   specified by ORD. You can try multiple orders at once by setting
%   ORDER to a vector of integers, in which case GRED is an array of
%   reduced models. NCFMR also returns a structure INFO with fields:
%     GL          Left normalized coprime factors [Ml,Nl] of G (see LNCF)
%     HSV         Hankel singular values of GL.
%     ErrorBound  Bound on approximation error || [MRED,NRED]-[Ml,Nl] ||oo 
%                 where G = Ml\Nl and GRED = MRED\NRED.
%
%   [~,INFO] = NCFMR(G) just computes the Hankel singular values and
%   error bounds. You can use this information to select the reduced order
%   ORD based on desired fidelity or robust stability considerations.
%  
%   NCFMR(G) displays the same information on a plot.
%
%   Note: When performance is a concern, use the syntax
%      [~,info] = ncfmr(G);
%      <select order, graphically or programmatically>
%      Gred = ncfmr(G,order,info);
%   to avoid computing the coprime factorization and Hankel singular values 
%   twice.
%
%   Note: When using NCFMR to reduce C or P in the feedback loop 
%  
%            u --->O--->[ C ]-->[ P ]---+---> y
%                - |                    |
%                  +<-------------------+
%
%   closed-loop stability is guaranteed as long as the approximation error
%   is smaller than the robustness margin b(P,C) computed by NCFMARGIN.
%
%   Example: Reduce a 30th-order model with NCFMR:
%      rng(0), G = rss(30,2,3);
%      % Visualize Hankel singular values
%      ncfmr(G)
%      % Pick first order for which the approximation error is below 0.01
%      Gr = ncfmr(G,13);
%      sigma(G,G-Gr)
%
%   See also BALRED, REDUCE. 

% Copyright 1988-2021 The MathWorks, Inc.
if nmodels(G)~=1
   error(message('Robust:transformation:ncfmr1'))
elseif hasdelay(G)
   error(message('Robust:transformation:ncfmr2'))
end

% Parse argument list
orders = [];
info = [];
maxerr = [];
DISPLAY = false;
narg = numel(varargin);
iarg = 1;
while iarg<=narg
   arg = varargin{iarg};
   if isnumeric(arg)
      orders = arg;  iarg = iarg+1;
   elseif isa(arg,'rctutil.ncfmrInfo')
      info = arg;  iarg = iarg+1;
   elseif strncmpi(arg,'m',1) && iarg<narg
      % Note: Overwrites ORDERS
      maxerr = varargin{iarg+1};  orders = [];  iarg = iarg+2;
      if ~(isnumeric(maxerr) && isvector(maxerr) && isreal(maxerr) && all(maxerr>0 & maxerr<Inf))
         error(message('Robust:transformation:ncfmr5'))
      end
   elseif strncmpi(arg,'o',1) && iarg<narg
      orders = varargin{iarg+1};  iarg = iarg+2;
   elseif strncmpi(arg,'d',1) && iarg<narg
      DISPLAY = strcmp(varargin{iarg+1},'on');  iarg = iarg+2;
   else
      error(message('Robust:transformation:ncfmr3'))
   end
end
      
% Compute LNCF
if isempty(info)
   try
      GL = lncf(G);
   catch ME
      error(message('Robust:transformation:ncfmr4',ME.message))
   end
else
   GL = info.GL;
end

% Compute reduced-orders model
if nargout>0
   % When no orders is explicitly specified, compute Hankel singular values of GL
   if isempty(orders)
      % Note: GL is stable and all-pass (GL*GL~ = I)
      [~,info] = balred(GL);
      if ~isempty(maxerr)
         % Resolve orders
         m = numel(maxerr);
         orders = zeros(1,m);
         for ct=1:m
            orders(ct) = max([0 find(info.ErrorBound>maxerr(ct),1,'last')]);
         end
      end
   else
      % Validate specified orders
      if ~(isnumeric(orders) && isvector(orders) && isreal(orders) && ...
            all(orders>=0 & orders<Inf & rem(orders,1)==0))
         error(message('Robust:transformation:ncfmr6'))
      end
   end
   
   if isempty(orders)
      Gr = [];  GLr = [];
   else
      opt = balredOptions('ErrorBound','absolute','StateProjection','truncate');
      orders = min(orders,order(GL));
      if isempty(info)
         [GLr,info] = balred(GL,orders,opt);
      else
         [GLr,~] = balred(GL,orders,opt,info);
      end
      [ny,nu] = size(G);
      try
         Gr = -imp2exp(GLr,1:ny,ny+1:ny+nu,'min');
      catch
         error(message('Robust:transformation:ncfmr7'))
      end
   end
   
   if ~isempty(info)
      % cast from balredInfo to ncfmrInfo
      info = rctutil.ncfmrInfo(info);
      info.GL = GL;
   end
end

if DISPLAY || (nargout==0 && isempty(orders))
   % Plot Hankel singular values
   hsvplot(GL);
end