function [Si,So] = wcmargin(varargin)
%WCMARGIN  Worst-case stability margins of uncertain feedback loops.
%
%   WCMARGIN is obsolete, use WCDISKMARGIN instead.
%
%   See also WCDISKMARGIN, DISKMARGIN.           

% Old help:
%WCMARGIN  Worst-case stability margins of uncertain feedback loops.
%
%   WCMARG = WCMARGIN(L) calculates the worst-case loop-at-a-time disk 
%   margins for the N-channel uncertain feedback loop:
%
%            u --->O---->[ L ]----+---> y
%                  |              |         
%                  +<-------------+
%
%   For each single-input, single-output loop L(i,i), the worst-case disk 
%   margins provide mimimum guaranteed gain and phase margins (in the 
%   classical sense) and help determine how much combined gain/phase 
%   variations the feedback loop can robustly tolerate. See DISKMARGIN and 
%   DMPLOT for more details on the disk margins. 
%
%   The N-by-N loop transfer matrix L can be a USS or UFRD model. Negative 
%   feedback is assumed in the feedback path (specify -L otherwise).  
%
%   WCMARG is a N-by-1 struct array where WCMARG(i) gives the worst-case 
%   disk margins for the i-th feedback channel. The fields of WCMARG(i) 
%   are as follows:
%        GainMargin: Worst-case disk gain margin (lower bound on the
%                    worst-case gain margin for the i-th loop L(i,i))
%       PhaseMargin: Worst-case disk phase margin in degrees (lower 
%                    bound on the worst-case phase margin for L(i,i))
%         Frequency: Frequency where the margin disk touches the
%                    Nyquist plot envelope for the uncertain loop
%                    transfer L(i,i) (rad/sec)
%             WCUnc: Structure of the worst-case uncertainty values 
%                    associated with the worst-case disk gain and phase 
%                    margins for the i-th loop L(i,i))
%       Sensitivity: Structure with one field per uncertain element 
%                    in L. The field values are the sensitivity of
%                    the worst-case margins to each uncertain element.
%
%   [WCMARGI,WCMARGO] = WCMARGIN(P,C) calculates the worst-case disk
%   margins of the multivariable feedback loop consisting of a 
%   controller C, of size NU-by-NY, in negative feedback with a plant P,
%   of size NY-by-NU.  Either P or C (or both) are uncertain models.
%   Only specify the portion of C in the feedback path for "2-dof" 
%   architectures.  This returns:
%     * NU-by-1 structure WCMARGI containing the worst-case,
%       loop-at-a-time disk margins at the plant inputs (L = C*P)
%     * NY-by-1 structure WCMARGO containing the worst-case,
%       loop-at-a-time disk margins at the plant outputs (L = P*C).
%
%   WCMARG = WCMARGIN(L,OPT) and [WCMARGI,WCMARGO] = WCMARGIN(P,C,OPT)  
%   further specify options OPT (see WCOPTIONS for more details).
%   By default, WCMARGIN does not compute margin sensitivities to 
%   uncertainties.  To compute sensitivities, do
%
%      opt = wcOptions('Sensitivity','on');
%      [wcmargi,wcmargo] = wcmargin(P,C,opt)
%
%   Example: Consider a nominal 2-input, 2-output plant with 8% modeling 
%   errors at the plant outputs and a variation in the gain of the first 
%   input channel between 0.97 and 1.06.
%       ingain1 = ureal('ingain1',1,'Range',[0.97 1.06]);
%       a = [-0.2 10;-10 -0.2];
%       b = [ingain1 0;0 1];
%       c = [1 8;-10 1];
%       d = zeros(2,2);
%       sat = ss(a,b,c,d);
%       unmod = ultidyn('unmod',[2 2],'Bound',0.08);
%       sat = (eye(2)+unmod)*sat;
%       k = [1 -2;0 1];
%       [wcmi,wcmo] = wcmargin(sat,k)
%       % Use the worst-case uncertainty values and compare with result
%       % from DISKMARGIN
%       di = diskmargin(k*usubs(sat,wcmi(1).WCUnc));
%       wcmi(1).GainMargin
%       di(1).GainMargin
%
%   See also WCOPTIONS, DISKMARGIN, DMPLOT, LOOPSENS, ROBSTAB, 
%            WCGAIN, WCSENS.           

%   Copyright 2003-2018 The MathWorks, Inc.
nin = nargin;
narginchk(1,3)

% Look for options
if nin>1 && ~(isnumeric(varargin{nin}) || isa(varargin{nin},'InputOutputModel'))
   Opt = varargin{nin};  nin = nin-1;
else
   Opt = wcOptions;
end

% Validate L or P,C
if nin>2 || ~all(cellfun(@(x) isnumeric(x) || isa(x,'InputOutputModel'),varargin(1:nin)))
   error(message('Robust:obsolete:WCMARGIN3'))
end

% Validate and remap options
if isa(Opt,'rctoptions.wcmargin')
   Opt = wcOptions('Sensitivity',Opt.Sensitivity);
elseif isa(Opt,'rctoptions.wcgain')
   Opt = wcOptions('Sensitivity',Opt.Sensitivity,'SensitivityPercent',Opt.VaryUncertainty);
elseif ~isa(Opt,'rctoptions.wcAnalysis')
   error(message('Robust:obsolete:WCMARGIN1'))
end
Opt.VaryFrequency = 'off';

% Use WCDISKMARGIN to compute margins
try
   switch nin
      case 1
         % WCMARG = WCMARGIN(L)
         Si = localComputeMargins(varargin{1},Opt);
         So = Si;
      case 2
         % [WCMARGI,WCMARGO] = WCMARGIN(P,C)
         P = varargin{1};
         C = varargin{2};
         [nyC,nuC,~] = size(C);
         [nyP,nuP,~] = size(P);
         if nuP~=nyC
            error(message('Robust:analysis:OutputCInputPMismatch'))
         elseif nyP~=nuC
            error(message('Robust:analysis:InputCOutputPMismatch'))
         end
         Si = localComputeMargins(C*P,Opt);
         So = localComputeMargins(P*C,Opt);
   end
catch ME
   throw(ME)
end
   
   
function S = localComputeMargins(L,Opt)
% Compute margins via WCDISKMARGIN
% Note: For model arrays, WCMARGIN returned the margins for each model
[nL,~,nsys] = size(L);
S = struct('GainMargin',cell([nL getArraySize(L)]),...
   'PhaseMargin',[],'Frequency',[],'WCUnc',[],'Sensitivity',[]);
for ct=1:nsys
   [DM,WCU,INFO] = wcdiskmargin(subparen(L,{':',':',ct}),'siso',Opt);
   for j=1:nL
      S(j,ct).GainMargin = DM(j).GainMargin;
      S(j,ct).PhaseMargin = DM(j).PhaseMargin;
      S(j,ct).Frequency = DM(j).CriticalFrequency;
      S(j,ct).WCUnc = WCU(j);
      S(j,ct).Sensitivity = INFO(j).Sensitivity;
   end
end
   

