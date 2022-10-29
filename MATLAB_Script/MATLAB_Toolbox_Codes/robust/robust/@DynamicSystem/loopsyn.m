function [K,CL,GAM,INFO]=loopsyn(G,Gd,varargin)
%LOOPSYN  Loop shaping with performance/robustness tradeoff.
%
%   LOOPSYN blends two loop shaping methods:
%     1) Mixed-sensitivity design (MIXSYN), which tends to optimize
%        performance and decoupling at the expense of robustness.
%     2) The Glover-McFarlane method (NCFSYN), which maximizes robustness
%        to plant uncertainty.
%   LOOPSYN lets you adjust the tradeoff between performance and robustness
%   to obtain satisfactory time responses while avoiding fragile designs 
%   with plant inversion or flexible mode cancellation.
%
%   [K,CL,GAM,INFO] = LOOPSYN(G,Gd) computes a stabilizing controller K
%   that shapes the open-loop response G*K to approximately match the
%   specified loop shape Gd. The plant G can be SISO or MIMO with at least
%   as many inputs as outputs. The loop shape Gd can be a transfer function
%   or an FRD model specifying gain vs. frequency. LOOPSYN returns the
%   closed-loop transfer CL=G*K/(I+G*K), the MIXSYN cost GAM, and a
%   structure INFO with fields:
%      W      Shaping prefilter (shaped plant Gs=G*W has the desired loop
%            shape Gd)
%      Gs     Shaped plant G*W
%      Ks     H-infinity controller for Gs (K = W*Ks)
%      emax   Maximal robustness margin in the NCFSYN/NCFMARGIN sense
%      W1,W3  Weights in MIXSYN formulation of loop shaping goal. 
%             The MIXSYN cost is 
%                 GAM = || [W1*S;W3*T] ||oo,  S = inv(I+G*K),  T = I-S.
%             A value GAM near or below 1 indicates G*K is close to Gd.
%      K2dof  Two-degree-of-freedom implementation (see documentation).
%
%   [K,CL,GAM,INFO] = LOOPSYN(G,Gd,ALPHA) explicitly specifies the tradeoff
%   ALPHA between performance and robustness:
%     * ALPHA=0 gives the MIXSYN design.
%     * ALPHA=1 gives the NCFSYN design.
%     * The default value ALPHA=0.5 is a balanced design that maximizes
%       performance (minimizes GAM) subject to being half as robust as 
%       the NCFSYN design.
%   You can adjust ALPHA between 0 and 1 to find the right tradeoff for
%   your application.
%
%   [K,CL,GAM,INFO] = LOOPSYN(G,Gd,ALPHA,ORD) explicitly specifies the
%   order of the controller K. This option only applies to 0<ALPHA<1.
%
%   Note: For MIMO plants G with NY outputs, you can specify the same loop
%   shape for all loops (SISO Gd), or a different loop shape for each loop
%   (NY-by-NY diagonal Gd). For example
%       s = tf('s');
%       G = [(1+0.1*s)/(1+s) 0.1/(s+2) ; 0 (s+2)/(s^2+s+3)];
%       wc = [1 5];  % target crossover frequencies
%       Gd = append(wc(1)/s,wc(2)/s)
%       K = loopsyn(G,Gd)
%       bodemag(G*K,{0.1 100})
%   specifies different bandwidths for the two feedback loops. This syntax
%   is not recommended when the loops are strongly coupled.
%
%   See also mixsyn, ncfsyn, ncfmargin, makeweight.

% Copyright 2003-2021 The MathWorks, Inc.
narginchk(2,4)

% Validate plant
[ny,nu,nsys] = size(G);
if nsys>1 || ny>nu
   error(message('Robust:design:loopsyn1'))
elseif hasdelay(G)
   error(message('Robust:design:loopsyn2'))
end
try
   G = ss(G,'exp');
catch ME
   error(message('Robust:design:loopsyn3'))
end

% Validate loop shape
[p,m,nsys] = size(Gd);
if ~isa(Gd,'DynamicSystem') || nsys>1
   error(message('Robust:design:loopsyn4'))
elseif hasdelay(Gd)
   error(message('Robust:design:loopsyn2'))
elseif (p>1 || m>1) && (p~=m || p~=ny)
   error(message('Robust:design:loopsyn5'))
elseif (p>1 || m>1) && ny~=nu
   error(message('Robust:design:loopsyn6'))
end

% Handle loop shape from FRD
if isa(Gd,'FRDModel')
   Gd = localFitMag(Gd);
end
   
% Align sample times
Ts = G.Ts;
Tsd = Gd.Ts;
if Ts==-1 || Tsd==-1
   error(message('Robust:design:loopsyn11'))
elseif Tsd~=Ts
   Gd = localResample(Gd,Ts);
end

% Process options
% Backward compatibility: ignore {wmin,wmmax}
ir = find(cellfun(@iscell,varargin),1);
if ~isempty(ir)
   varargin(:,ir) = [];
end
nopt = numel(varargin);
if nopt>0
   alpha = varargin{1};
   if ~(isnumeric(alpha) && isscalar(alpha) && isreal(alpha) && 0<=alpha && alpha<=1)
      error(message('Robust:design:loopsyn7'))
   end
else
   alpha = 0.5;
end
if nopt>1 && ~isempty(varargin{2})
   ordK = varargin{2};
   if ~(isnumeric(ordK) && isscalar(ordK) && isreal(ordK) && 0<=ordK && rem(ordK,1)==0)
      error(message('Robust:design:loopsyn8'))
   end
else
   ordK = [];
end

% Compute target crossover frequencies and plant conditioning near crossover
[wc,condG] = localCharacteristics(G,Gd);
if eps*condG>1
   error(message('Robust:design:loopsyn10'))
end

% Get MIXSYN weights
% Choice of CrossTol based on 1/s^1.5 slope at wc and squeezing log(cond(G)) in half
delta = condG^(1/6);
if p>1
   % multi-loop shaping
   [zd,pd,kd] = zpkdata(Gd);
   W1 = cell(p,1); W3 = cell(p,1);
   for j=1:p
      Gdjj = zpk(zd{j,j},pd{j,j},kd(j,j),Ts);
      [W1{j},W3{j}] = rctutil.getMIXWeight(Gdjj,wc(j),delta); % RETURN SS!
   end
   W1 = append(W1{:});  W3 = append(W3{:});
else
   [W1,W3] = rctutil.getMIXWeight(Gd,wc,delta);
end

% Perform synthesis
INFO = struct('W',[],'Gs',[],'Ks',[],'emax',[],'W1',W1,'W3',W3,'K2dof',[]);
if alpha==0
   % Compute MIXSYN controller
   opt = hinfsynOptions;
   [K,~,GAM] = mixsyn(G,W1,[],W3,opt);
else
   % Compute NCFSYN pre-filter W
   if p>1
      % multi-loop shaping
      W = cell(p,1);
      for j=1:p
         Gdjj = zpk(zd{j,j},pd{j,j},kd(j,j),Ts);
         Gsmjj = localSmoothPlantResponse(subparen(G,{j j}));
         W{j} = rctutil.getNCFWeight(Gdjj,wc(j),Gsmjj);
      end
      W = append(W{:});
   else
      % Smooth response of G to ensure smoothness of W1
      Gsm = localSmoothPlantResponse(G);
      W = rctutil.getNCFWeight(Gd,wc,Gsm);
   end
   INFO.W = W;
   Gs = G*W;
   INFO.Gs = Gs;   
   % Compute NCFSYN controller
   [K,~,GAM,NCFINFO] = ncfsyn(-G,W,[],0.01);
   emax = 1/GAM;  % maximum uncertainty level
   INFO.emax = emax;
   if GAM<Inf
      Ks = NCFINFO.Ks;  INFO.Ks = Ks;
   end
   
   if emax>0
      if alpha==1
         % Compute MIXSYN performance for NCFSYN controller
         GAM = getPeakGain(lft(augw(G,W1,[],W3),K),1e-3);
      else
         % Optimize performance/robutness tradeoff
         erob = alpha*emax;  % target robustness level
         % Initialize fixed-order controller from NCF controller Ks. Try
         % reducing order while preserving at least the desired NCF margin
         if isempty(ordK)
            % Pick suitable order based on robustness margin EMAX
            Ksr = localPickOrder(Ks,(1-alpha)*emax);
         else
            ordW = order(W);
            if p==1
               % SISO weight for MIMO loop: account for scalar expansion
               ordW = ordW*nu;
            end
            if ordK<ordW
               error(message('Robust:design:loopsyn12',ordW))
            end
            Ksr = ncfmr(Ks,ordK-ordW);
         end
         %fprintf('Ks order reduced from %d to %d\n',order(Ks),order(Ksr))
         KT = tunableSS('K',Ksr,'tridiag');
         
         % Build tunable closed-loop model
         Gs.InputName = 'uG';   Gs.OutputName = 'yG';
         KT.InputName = 'e';  KT.OutputName = 'u';
         W1 = W1*eye(ny);  W1.InputName = 'e';  W1.OutputName = 'e1';
         W3 = W3*eye(ny);  W3.InputName = 'y';  W3.OutputName = 'e3';
         S1 = sumblk('e = r-y',ny);
         S2 = sumblk('uG = u+du',nu);
         S3 = sumblk('y = yG+dy',ny);
         CL = connect(Gs,KT,S1,S2,S3,W1,W3,{'du','dy','r'},{'u','y','e1','e3'});
         % Minimize || r->e1,e3 || subject to || w1,w2->u,y || < 1/(alpha*emax)
         % The hard constraint ensures robustness against uncertainty of size
         % alpha*emax.
         SG = TuningGoal.Gain('r',{'e1','e3'},1);
         HG = TuningGoal.Gain({'du','dy'},{'u','y'},1/erob);
         [CL,GAM,gHard] = systune(CL,SG,HG,systuneOptions('Display','off'));
         if gHard<1.1
            INFO.Ks = getBlockValue(CL,'K');
            K = W * INFO.Ks;
         else
            % Could not improve on NCF controller (order too low)
            warning(message('Robust:design:loopsyn13'))
            K = W * Ks;
            GAM = getPeakGain(lft(augw(G,W1,[],W3),K),1e-3);
         end
      end
   end
end

if GAM<Inf
   CL = feedback(G*K,eye(ny));
   % Two DOF architecture to avoid derivative kick: [r;y]-->u
   % Put Ks in the feedback path and correct for DC gain on reference
   if alpha==0
      INFO.K2dof = K * [eye(ny) -eye(ny)];
   else
      INFO.K2dof = INFO.W * [dcgain(INFO.Ks) -INFO.Ks];
   end
else
   CL = [];
end
%fprintf('mixsyn performance for alpha=%.3g: %.2e\n',alpha,GAM)

%---------------------- local functions --------------------------------

function Gs = localSmoothPlantResponse(G)
% Dampen poles and zeros to get smooth gain curve.
[z,p,k,Ts] = zpkdata(G);
for ct=1:numel(k)
   [z{ct},p{ct},k(ct)] = ltipack.util.dampenModes(z{ct},p{ct},k(ct),Ts,0.7);
end
Gs = zpk(z,p,k,Ts);
%figure(3), sigma(G,Gs), title('Plant smoothing')


%-------------
function [wc,condG] = localCharacteristics(G,Gd)
% Analyze problem characteristics

% Validate loop shape and compute target crossover frequencies
p = size(Gd,1);
wc = zeros(p,1);
for j=1:p
   Gdjj = subparen(Gd,{j j});
   wcj = getGainCrossover(Gdjj,1);
   if ~(isscalar(wcj) && abs(dcgain(Gdjj))>1)
      error(message('Robust:design:loopsyn9'))
   end
   wc(j) = wcj;
end
wcavg = pow2(mean(log2(wc))); % log average

% Check plant conditioning near crossover
condG = 1;
if size(G,1)>1
   h = freqresp(G,[wcavg/3 wcavg 3*wcavg]);
   for ct=1:3
      hw = h(:,:,ct);
      if p>1
         % inf_D cond(GD)  = || |G| |inv(G)| ||
         kappa = norm(abs(hw)*abs(inv(hw)));
      else
         kappa = cond(hw);
      end
      condG = max(condG,kappa);
   end
end

%-------------
function Ksr = localPickOrder(Ks,maxerr)
% Pick lowest order compatible with ||KL-KLr||<maxerr where
% KL and KLr are the LNCF of Ks and Ksr.
[~,info] = ncfmr(Ks);
KL = info.GL; % LNCF of Ks
MaxOrder = max([3 find(info.ErrorBound>maxerr,1,'last')]);
% Try orders from r=MaxOrder to r=3
orders = MaxOrder:-1:3;
[Ksr,~,KLr] = ncfmr(Ks,orders,info);
k = 1;
while k<numel(orders) && getPeakGain(KL-subparen(KLr,{':' ':' k+1}),1e-3)<maxerr
   k = k+1;
end
Ksr = subparen(Ksr,{':' ':' k});
%fprintf('Ksr reduced from %d to %d\n',MaxOrder,order(Ksr))

%-------------
function Gd = localFitMag(fGd)
% Turn FRD loop shape into continuous ZPK model.
[R,f] = frdata(fGd);
nL = size(R,1);
FIT = cell(nL,1);
for j=1:nL
   FIT{j} = TuningGoal.fitMagProfile(f,squeeze(R(j,j,:)));
end
Gd = append(FIT{:}); % Note: Ts=0

%-------------
function Gd = localResample(Gd,Ts)
% Aligns sample time of Gd with that of G
ny = size(Gd,1);
AUX = cell(1,ny);
for j=1:ny
   AUX{j} = TuningGoal.resampleWeight(subparen(Gd,{j j}),Ts);
end
Gd = append(AUX{:});
   
