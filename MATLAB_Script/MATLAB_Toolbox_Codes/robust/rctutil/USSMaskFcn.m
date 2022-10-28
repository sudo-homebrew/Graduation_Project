function varargout = USSMaskFcn(Action,varargin)
%USSMASKFCN  Manages callbacks from USS Block Mask.
%
% See also RCTBLOCKS, USS, UMAT, UREAL, ULTIDYN.

%   Authors: Gary J. Balas, Andy K. Packard, P. Gahinet
%   Copyright 2006-2009 The MathWorks, Inc.

% Turn off warnings
hw = ctrlMsgUtils.SuspendWarnings; %#ok<NASGU>

switch Action
   case 'Initialize'
      % Used both for initializing and updating after Apply
      % RE: The mask variables usys, uval, X0, uX0 are automatically
      % reevaluated in the proper workspace when the mask init callback
      % is executed.

      % Check mask data (already evaluated)
      %   varargin = {CB,usys,uval,X0,uX0}
      %   varargout = {csys,cX0,sysname};
      [varargout{1:nargout}] = LocalCheckData(varargin{:});
            
   case 'MaskUSSCallback'
      % Callback from editing LTI system field
      % Only enables/disables the "Initial states (uncertain dynamics)"
      % edit field. Do not attempt to run mask init (would disable Cancel) 
      % or reset dialog entries here (must wait for Apply)
      CB = varargin{1};
      MaskVal = get_param(CB,'MaskValues');
      try
         % Keep warnings enabled here to 
         usys = slResolve(MaskVal{1},CB);
         usys = LOCALliftUSS(usys);
      catch  %#ok<CTCH>
         usys = [];
      end
      if ~isequal(usys,[]) && any(cellfun(@(x) isa(x,'ultidyn') || isa(x,'umargin'),...
            struct2cell(usys.Uncertainty)))
         % Enable "Initial states (uncertain dynamics)"
         set_param(CB,'MaskEnables',{'on';'on';'on';'on'})
      else
         % Disable "Initial states (uncertain dynamics)"
         set_param(CB,'MaskEnables',{'on';'on';'on';'off'})
      end
end


%-----------------------------Internal Functions---------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [csys,cX0,sysname] = LocalCheckData(CB,usys,uval,X0,uX0)
% Checks system data
MaskVal = get_param(CB,'MaskValues');
sysname = MaskVal{1};

% Convert SYS to USS (may error)
usys = LOCALliftUSS(usys);

% Derive LTI instance used for simulation
if isempty(uval)
   csys = usys.NominalValue;  % ss
elseif ~isstruct(uval)
   error(message('Robust:simulink:USSMask8'))
else
   csys = localInstantiate(usys,uval);
end

% Set initial condition cX0 for LTI block under the mask
nx = length(usys.StateName);  % nominal state dimension
cX0 = LocalInitialConditions(nx,X0,order(csys)-nx,uX0);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function X0 = LocalInitialConditions(nx,X0,nxunc,X0unc)
% Validate Nominal system initial conditions
if ~isnumeric(X0) || ~isreal(X0)
   error(message('Robust:simulink:USSMask6'))
elseif numel(X0)>1 && numel(X0)~=nx
   error(message('Robust:simulink:USSMask7'))
elseif isscalar(X0)
   X0 = X0(ones(nx,1),1);
elseif isempty(X0)
   X0 = zeros(nx,1);
else
   X0 = double(X0(:));
end
% Validate Uncertainty initial conditions if they exist
if nxunc>0
   if ~isnumeric(X0unc) || ~isreal(X0unc)
      error(message('Robust:simulink:USSMask6'))
   elseif numel(X0unc)>1 && numel(X0unc)~=nxunc
      error(message('Robust:simulink:USSMask7'))
   elseif isscalar(X0unc)
      X0unc = X0unc(ones(nxunc,1),1);
   elseif isempty(X0unc)
      X0unc = zeros(nxunc,1);
   else
      X0unc = double(X0unc(:));
   end
   X0 = [X0; X0unc];
end
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function sys = LOCALliftUSS(sys) 
if isempty(sys)
   error(message('Robust:simulink:USSMask1'))
elseif isa(sys,'DynamicSystem') && hasdelay(sys)
   error(message('Robust:simulink:USSMask5'))
end
% Convert to USS
try
   sys = uss(sys);
catch %#ok<CTCH>
   error(message('Robust:simulink:USSMask2',class(sys)))
end
% Check USS attributes
if ~isproper(sys)
   error(message('Robust:simulink:USSMask9'))
elseif ~isreal(sys) || any(cellfun(@(x) isa(x,'udyn'),struct2cell(sys.Uncertainty)))
   error(message('Robust:simulink:USSMask4'))
elseif ndims(sys)>2
   % USS arrays not supported
   error(message('Robust:simulink:USSMask3'))
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function csys = localInstantiate(usys,uSpec)
% Sets the uncertain elements defined in USPEC to their user-defined values
% and the remaining uncertain elements to their nominal values
[~,~,blks] = lftdata(usys);
nblks = length(blks);
blkNames = {blks.Name};
uVals = cell2struct(cell(nblks,1),blkNames,1);
% Fill in user-supplied values
[UserDef,~,ib] = intersect(fieldnames(uSpec),blkNames);
for ct=1:length(UserDef)
   uVals.(UserDef{ct}) = uSpec.(UserDef{ct});
end
% Use nominal value for remaining blocks
blkNames(ib) = [];
for ct=1:length(blkNames)
   uVals.(blkNames{ct}) = usys.Uncertainty.(blkNames{ct}).Nominal;
end
% Replace each block by its value
% Beware that USUBS can return @double value for uncertain gains
csys = ss(usubs(usys,uVals));

