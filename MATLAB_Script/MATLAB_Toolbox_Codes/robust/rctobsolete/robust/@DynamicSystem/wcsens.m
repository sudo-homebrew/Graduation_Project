function wcst = wcsens(P,C,varargin)
%WCSENS  Worst-case sensitivity of plant-controller feedback loop.
%
%   WCST = WCSENS(P,C) calculates the worst-case sensitivity and 
%   complementary sensitivity functions for the feedback loop:
%
%                    u  
%          du --->O----->[ P ]-----+---> 
%               - |                |       
%                 |             y  |
%             <---+<-----[ C ]-----O<--- dy
%
%   The plant P and controller C can be numeric or uncertain LTI models 
%   (see NUMLTI and ULTI). For "2-dof" architectures, C should only include 
%   the portion of the controller in the feedback path. WCST is a structure 
%   with the following substructures:
%           Si: Worst-case input-to-plant sensitivity 
%           Ti: Worst-case input-to-plant complementary sensitivity 
%           So: Worst-case output-of-plant sensitivity 
%           To: Worst-case output-of-plant complementary sensitivity 
%          PSi: Worst-case plant times input-to-plant sensitivity 
%          CSo: Worst-case compensator times output-of-plant sensitivity 
%       Stable: 1 if nominal closed loop is stable, 0 otherwise, NaN for FRDs.
%   Each sensitivity substructures, WCST.Si, WCST.Ti, etc, is also a 
%   structure containing five fields: 
%     MaximumGain: Structure containing lower and upper bounds on the 
%            worst-case peak gain of the sensitivity function, and the  
%            critical frequency at which this peak gain occurs.
%     BadUncertainValues: Worst-case uncertainty values
%     System: Uncertain model of the sensitivity function of interest.
%     BadSystem: Worst-case value of this sensitivity function.
%     Sensitivity: Sensitivity of the peak gain of the sensitivity function
%               to each uncertain element.
%
%   WCST = WCSENS(L) calculates the worst-case sensitivity and complementary 
%   sensitivity functions for the MIMO feedback loop:
% 
%         u --->O---->[ L ]----+---> y
%             - |              |         
%               +<-------------+
%   
%   and is equivalent to setting P=L and C=I.
%   
%   WCST = WCSENS(L,TYPE,SCALING) AND WCST = WCSENS(P,C,TYPE,SCALING) allow
%   selection of the desired sensitivity and complementary sensitivity 
%   functions, TYPE, as 
%      'Si', 'So', 'PSi', 'CSo', 'Ti','To' correspond to the sensitivity
%          and complementary sensitivity functions.
%      'S' or 'T' selects all sensitivity or complementary sensitivity 
%          functions 
%      'Input' or 'Output' selects all input or output Sensitivity
%          functions.
%      'All' selects all Sensitivity functions (default).
%   
%   SCALING defines the scaling of the sensitivity function used in the 
%   worst-case analysis. SCALING is either the character strings 'Absolute'
%   (default), 'Relative' or a LTI object. If SCALING is 'Relative' or a 
%   LTI object, the worst-case analysis peaks over frequency relative to 
%   the magnitude of the nominal value of function. SCALING should be 
%   either 1-by-1 or dimensions compatible with P and C respectively. 
%   TYPE and SCALING can also be combined in a cell array, e.g.
%   
%    WCST = WCSENS(P,C,{'Ti','So'},'Abs','Si','Rel','PSi',WT)
%   
%   This calling sequence indicates that the 'Ti' and 'So' worst-case 
%   sensitivites are calculated with respect to an absolute magnitude 
%   weight. The 'Si' worst-case sensitivity is calculated with respect to 
%   a relative magnitude weight and the 'PSi' worst-case sensitivity is
%   calculated with respect to WT scaling.
%   
%   WCST = WCSENS(L,OPT), WCST = WCSENS(L,TYPE,SCALING,OPT),
%   WCST = WCSENS(P,C,OPT) or WCST = WCSENS(P,C,TYPE,SCALING,OPT) specifies 
%   options described by OPT. See WCGAINOPTIONS for more details on 
%   available options. The default behavior does not compute sensitivity 
%   of the worst-case margins to the uncertainties, MarginSens. To compute  
%   sensitivities, use:
%   
%      opt = wcgainOptions('Sensitivity','on');
%      wcst = wcsens(P,C,opt)
%   
%   If L, P or C are arrays, the substructures of WCST will have the 
%   appropriate array dimensions.
%   
%   Example: Consider a feedback loop with a first-order plant and a PI 
%   controller. The time constant is uncertain and the feedback loop 
%   accounts for unmodeled dynamic uncertainty.
%   
%      delta = ultidyn('delta',[1 1]);
%      tau = ureal('tau',5,'range',[4 6]);
%      P = tf(1,[tau 1])*(1+0.25*delta);
%      C = pid(4,4);
%      looptransfer = loopsens(P,C);
%      Snom = looptransfer.Si.NominalValue;
%      om = logspace(-0.5,1,30);
%      wcst = wcsens(ufrd(P,om),C,'Si');    
%      Swc = wcst.Si.BadSystem;
%      bodemag(frd(Snom,Swc.Frequency),'b-',Swc,'r-.');
%   
%   See also WCGAINOPTIONS, LOOPSENS, WCGAIN, DISKMARGIN, WCMARGIN, USUBS.

%   Copyright 2003-2011 The MathWorks, Inc.
nin = nargin;
exin = nin-2;
szp = size(P);
opt = [];
if nin>1
   [C,varargin{:}] = convertStringsToChars(C,varargin{:});
end

if nin == 1 % (L) square, make C=I,
   if szp(1)==szp(2)
      C = eye(szp(1),szp(1));
   else
      ctrlMsgUtils.error('Robust:obsolete:WCSENSNonSquare');
   end
elseif nin == 2 % either (L,OPT) or (P,C)
   if isa(C,'rctoptions.wcgain') % L, OPT
      opt = C;
      if szp(1)==szp(2)
         C = eye(szp(1),szp(1));
      else
         ctrlMsgUtils.error('Robust:obsolete:WCSENSNonSquare');
      end
   elseif ( isa(C,'InputOutputModel') || isa(C,'double') ) % P, C
      szc = size(C);
      if ~(szp(1)==szc(2) && szp(2)==szc(1))
         ctrlMsgUtils.error('Robust:obsolete:WCSENSDimError');
      end
   elseif isa(C,'char') % type
      varargin{1} = C;
      if szp(1)==szp(2)
         C = eye(szp(1),szp(1));
      else
         ctrlMsgUtils.error('Robust:obsolete:WCSENSNonSquare');
      end
   else
      ctrlMsgUtils.error('Robust:obsolete:WCSENSInvalid');
   end
end

if nin<3
   if isempty(varargin)
   varargin{1} = 'All';
   end
   varargin{2}  = 'Absolute';
   exin = 2;
end
if nin == 3 % (P,C,OPT) or (P,C,TYPE)
   if isempty(varargin{1})
      varargin{1} = 'All';
      varargin{2}  = 'Absolute';
       
   elseif ~isa(C,'rctoptions.wcgain') && isa(varargin{1},'rctoptions.wcgain')
      opt = varargin{1};
      varargin{1} = 'All';
      varargin{2}  = 'Absolute';
   elseif isa(C,'cell') || isa(C,'char')
      varargin{2}  = varargin{1};
      varargin{1}  = C;
      if szp(1)==szp(2)
         C = eye(szp(1),szp(1));
      else
         ctrlMsgUtils.error('Robust:obsolete:WCSENSNonSquare');
      end
   elseif isa(varargin{1},'cell') || isa(varargin{1},'char')
      varargin{2}  = 'Absolute';
   else
      ctrlMsgUtils.error('Robust:obsolete:WCSENSTypeError');
   end
   exin = 2;
else
   if mod(exin,2) %odd # of EX inputs, last must be OPT
      if isa(varargin{exin},'rctoptions.wcgain')
         opt = varargin{exin};
         exin = exin - 1;
      else
         ctrlMsgUtils.error('Robust:obsolete:WCSENSTypeError');
      end
   elseif exin==2 %is either TYPE/SCALING or TYPE/OPT
      if isa(varargin{exin},'rctoptions.wcgain')
         opt = varargin{exin};
         if isa(C,'cell') || isa(C,'char')
            varargin{2}  = varargin{1};
            varargin{1}  = C;
            if szp(1)==szp(2)
               C = eye(szp(1),szp(1));
            else
               ctrlMsgUtils.error('Robust:obsolete:WCSENSNonSquare');
            end
         else
            varargin{2}  = 'Absolute';
         end
      else
         opt = [];
      end
   else % even # of EX inputs
      opt = [];
   end
end

if isempty(opt)
   opt = wcgainOptions;
end
[names,values,~] = nvlist(varargin{1:exin});

typecell = {'Si' 'no';'Ti' 'no'; 'So' 'no'; 'To' 'no'; 'PSi' 'no'; 'CSo' 'no'};

for i=1:length(names)
   xname = names{i};
   if any(strncmpi(xname,'All',3)) % allsens functions
      typecell(:,2) = repmat(values(i),6,1);
   elseif any(strcmpi(xname,'T')) % wants all comp sens functions
      typecell([2 4],2) = repmat(values(i),2,1);
   elseif any(strcmpi(xname,'S')) % wants all comp sens functions
      typecell([1 3],2) = repmat(values(i),2,1);
   elseif any(strncmpi(xname,'I',1)) % wants all input sens functions
      typecell([1 2 5],2) = repmat(values(i),3,1);
   elseif any(strncmpi(xname,'O',1)) % wants all output sens functions
      typecell([3 4 6],2) = repmat(values(i),3,1);
   elseif strcmpi(xname,'Si') || strcmpi(xname,'Ti') || strcmpi(xname,'So') ...
         || strcmpi(xname,'To') || strcmpi(xname,'PSi') ||strcmpi(xname,'CSo')
      xloc = strcmpi(typecell(:,1),xname);
      typecell(xloc,2) = values(i);
   else
      ctrlMsgUtils.error('Robust:obsolete:WCSENSType');
   end
end

% All options should now be set in terms of Si, Ti, So, To, PSi, CSo
wcst = struct('Si',[],'So',[],'PSi',[],'Ti',[],'To',[],'CSo',[]);
try
   SF = loopsens(P,C);
catch ME
   % May fail because sampling time mismatch, unit mismatch,...
   throw(ME)
end
%Si
if ~strcmp(typecell{1,2},'no')
   scale = typecell{1,2};
   wcst.Si = LOCALwcsens(SF.Si,scale,opt);
end
%Ti
if ~strcmp(typecell{2,2},'no')
   scale = typecell{2,2};
   wcst.Ti = LOCALwcsens(SF.Ti,scale,opt);
end
%So
if ~strcmp(typecell{3,2},'no')
   scale = typecell{3,2};
   wcst.So = LOCALwcsens(SF.So,scale,opt);
end
%To
if ~strcmp(typecell{4,2},'no')
   scale = typecell{4,2};
   wcst.To = LOCALwcsens(SF.To,scale,opt);
end
%PSi
if ~strcmp(typecell{5,2},'no')
   scale = typecell{5,2};
   wcst.PSi = LOCALwcsens(SF.PSi,scale,opt);
end
%CSo
if ~strcmp(typecell{6,2},'no')
   scale = typecell{6,2};
   wcst.CSo = LOCALwcsens(SF.CSo,scale,opt);
end

%---------------------------------------------------------
function [wcanal,go] = LOCALwcsens(ST,scale,opts)

go = 1;

[tmpg,~] = LOCALprewt(ST,scale);
[maxgain,wcunc,info] = wcgain(tmpg,opts);

%if Si USS -> SiBad USS, Si UFRD -> SiBad UFRD
BadSystem = usubs(ST,wcunc);
unweightedmaxgain = wcgain(BadSystem,opts);

% Return UNWEIGHTED maxgain instead of WEIGHTED maxgain
for ii=1:numel(unweightedmaxgain)
   if isa(maxgain(ii).UpperBound,'double') && isinf(maxgain(ii).UpperBound)
      unweightedmaxgain(ii).UpperBound = maxgain(ii).UpperBound;
   else
      scl = unweightedmaxgain(ii).LowerBound/maxgain(ii).LowerBound;
      unweightedmaxgain(ii).UpperBound = maxgain(ii).UpperBound*scl;
   end
   if isa(maxgain(ii).LowerBound,'double') && isinf(maxgain(ii).LowerBound)
      unweightedmaxgain(ii).LowerBound = maxgain(ii).LowerBound;
   end
end
SensStruct = info(1).Sensitivity;
for i=numel(info):-1:2
   SensStruct(i) = info(i).Sensitivity;
end
SensStruct = reshape(SensStruct,size(info));
wcanal = struct('MaximumGain',unweightedmaxgain,...
   'BadUncertainValues',wcunc,'System',ST,'BadSystem',BadSystem,...
   'Sensitivity',SensStruct);


function [STout,scaleout] = LOCALprewt(STin,scale)

% STin is now a UFRD for a specific S and T case
STout = STin;
if ischar(scale) && strncmpi(scale,'Abs',1)
   scaleout = scale;
elseif ischar(scale) && strncmpi(scale,'Rel',1)
   if isa(STin,'FRDModel')
      % "Relative" scaling, or any scaling, implies interested in the PEAK,
      %  since scaling wouldn't matter if you are doing PTWISE.
      scaleout = 1/fnorm(frd(STin,STin.Frequency));
      % Gary XXX - why not just get NominalValue, see next line
      % scaleout = 1/fnorm(STin.NominalValue);
      STout = scaleout*STin;
   else
%      msg = ['P and/or C must be a UFRD or FRD for Relative '...
%             'Scaling of ' STtype ' Sensitivity'];
      ctrlMsgUtils.error('Robust:obsolete:WCSENSRelScl');
   end
elseif isa(scale,'double')
   try
      scaleout = abs(scale);
      STout = scaleout*STin; %scale STin
   catch ME
      %msg = [STtype ' and Scaling array dimensions are inconsistent'];
      %ctrlMsgUtils.error('Robust:obsolete:WCSENSRelScl');
      throw(ME)
   end
elseif isa(scale,'DynamicSystem')
   if STin.Ts == scale.Ts || (STin.Ts == -1 && scale.Ts ~= 0) ...
         || (STin.Ts ~= 0 && scale.Ts == -1)
      szm = size(scale);
      szst = size(STin);
      if szm(1)==szm(2) && szm(1)==1 % correct IO dim
         szml = length(szm);
         if szml==2 || isequal(szm(3:szml),szst(3:szml))
            if isa(scale,'FRDModel')
               %------------ SCALE is an FRD model -------------------%
               if ~isa(STin,'FRDModel')
                  STin = frd(STin,scale.Frequency);
               end
               if isequal(scale.Frequency,STin.Frequency) && ...
                  isequal(scale.FrequencyUnit,STin.FrequencyUnit)
                  scaleout = abs(frd(scale));
               else
                  %msg = [STtype ' and Scaling have inconsistent frequency vectors'];
                  ctrlMsgUtils.error('Robust:obsolete:WCSENSBadFreq');
               end
            else
               %------------ SCALE is a parametric model ---------------%
               if isa(STin,'FRDModel')
                  scaleout = abs(frd(scale,STin.Frequency));
               else
                  scaleout = getValue(scale);
               end
            end
         else
            %msg = [STtype ' and Scaling array dimensions are inconsistent'];
            ctrlMsgUtils.error('Robust:obsolete:WCSENSBadArray');
         end
      else
         %msg = [STtype ' Scaling is not a single-input/single-output system'];
         ctrlMsgUtils.error('Robust:obsolete:WCSENSSISO');
      end
      STout = scaleout*STin; %scale STin
   else
      %msg = [STtype ' and Scaling have different sample times'];
      ctrlMsgUtils.error('Robust:obsolete:WCSENSsamptime');
   end
else
   %msg = ['Invalid ' STtype ' Scaling, must be a SS/TF/PZK/FRD of correct dimension'];
   ctrlMsgUtils.error('Robust:obsolete:WCSENSBadDim');
end
