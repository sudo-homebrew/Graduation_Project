function varargout=ussmask(Action,varargin)
%USSMASK  Initialize Mask for USS/UMAT block.
%
%This function is meant to be called only by the USS/UMAT Block mask.
%
%See also RCTBLOCKS, CSTBLOCKS, LTIMASK, USS, UMAT, UREAL, ULTIDYN.

% Authors: Gary J. Balas and Andy K. Packard
%   Copyright 2006-2008 The MathWorks, Inc.

hw = ctrlMsgUtils.SuspendWarnings; %#ok<NASGU>

switch Action,
   case 'Initialize',
      % Used both for initializing and updating after Apply 
      % RE: The mask variable sys, IC (X0), type, logname are 
      % automatically reevaluated in the proper workspace when the mask 
      % init callback is executed.
      [CB,sys,X0,type,logname,X0unc,blkid] = deal(varargin{:});
      % Check mask data (already evaluated)
      [csys,Ts,X0,sysname,NomStateFlg,UncStateFlg] = ...
          LocalCheckData(CB,sys,X0,type,logname,X0unc,blkid);

      % Return data to Mask workspace (for initialization of blocks underneath)
      varargout = {csys,Ts,X0,sysname,NomStateFlg,UncStateFlg};

   case 'UpdateDiagram'
      % Updates diagram (separate from Initialize so that all mask
      % variables are resolved when updating, see g207027)
      LocalUpdateDiagram(varargin{:});

   case 'MaskUSSCallback'
      % Callback from editing LTI system field
      % RE: 1) Only enables/disables the X0 edit field. Do not attempt to 
      %        run mask init (would disable Cancel) or reset dialog entries
      %        here (must wait for Apply)
      %     2) Callback name hardwired in block property 'MaskCallbacks'
      [CB,blkid] = deal(varargin{:});
      
      MaskVal = get_param(CB,'MaskValues');
      type = MaskVal{3};
      sys = LocalEvaluate(MaskVal{1},bdroot(CB));
      hw = [];  %#ok<NASGU> % reenable warnings
      [sys,lerror] = LOCALliftUSS(sys);
      if isempty(lerror)
         maskvec = {'on';'on';'on';'on';'on'};
         maskvisvec = 'on,on,on,on,on';
         if ~isempty(sys)
            nx = length(sys.StateName);
            ultidynflg = LocalAnyUltidyn(sys);
            if strcmp(type,'Nominal')
               maskvec(4:5) = {'off';'off'};
            else
               maskvec(4:5) = {'on';'on'};
            end
            if nx==0 % Nominal system doesn't have states
               maskvec{2} = 'off';
            end
            if ~ultidynflg % no ultidyn blocks
               maskvec{5} = 'off';
            end
            if length(sys.Uncertainty)==0
               maskvec{4} = 'off';
            end
            set_param(CB,'MaskEnables',maskvec)
            set_param(CB,'MaskVisibilityString',maskvisvec)
         end
      end
end


%-----------------------------Internal Functions---------------------------

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% LocalCheckData %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
function [csys,Ts,IC,sysname,NomStateFlg,UncStateFlg] =  ...
   LocalCheckData(CB,sys,X0,type,logname,X0unc,blkid)
% Checks system data

% Defaults
A=[]; B=[]; C=[]; D=1; Ts=0; Tdi=0; Tdo=0;  
csys = [];
IC = zeros(0,1);
%X0 = [];
%X0unc = [];
NomStateFlg = ~isempty(X0);
UncStateFlg = ~isempty(X0unc);
MaskVal = get_param(CB,'MaskValues');
sysname = MaskVal{1};
%set_param(CB,'MaskDisplay',['disp(''' sysname ''')'])

[sys,lerror] = LOCALliftUSS(sys);
if ~isempty(lerror)
    LocalUSSError(lerror);
end

if isempty(sys)
	sysname = '???';
else
    simstat = get_param(bdroot(CB),'SimulationStatus');
    if ~strcmpi(simstat,'stopped')
         nx = length(sys.StateName);
         switch type
            case {'Nominal'}   % use nominal SYS LTI model
               csys = sys.NominalValue;
            case {'User defined'}
               try
                  tmp = usubs(sys,logname);
               catch
                  LocalUSSError(['Some uncertain blocks not specified.']);
               end
               if isuncertain(tmp)
                  LocalUSSError(['Some uncertain blocks not specified.']);
               else
                  csys = tmp;
               end
         end
         if isa(csys,'double')
            csys = ss(csys);
         end
         [A,B,C,D,Ts] = ssdata(csys);
         [IC,NomStateFlg,UncStateFlg] = ...
             LocalInitialConditions(nx,X0,X0unc,type,csys);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% LocalUpdateDiagram %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function LocalUpdateDiagram(CB,Ts,blkid)
% Don't need to do anything because LTI Simulink block handles everything
%disp('Inside LocalUpdateDiagram')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% LocalInitialConditions %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [IC,NomStateFlg,UncStateFlg] = ...
    LocalInitialConditions(nx,X0,X0unc,type,csys)
 % Validate Nominal system initial conditions
 if isempty(X0)
    X0 = zeros(nx,1);
 elseif length(X0)==nx && isa(X0,'double') && isreal(X0)
    X0 = X0;
 elseif length(X0)==1 && isa(X0,'double') && isreal(X0)
    X0 = X0*ones(nx,1); %same initial condition for all states 
 elseif ~any(length(X0)==[1 nx])
    LocalUSSError(...
       'Length of Nominal Initial State vector does not match number of states.')
 elseif ~ireal(X0)
    LocalUSSError('Nominal initial condition vector must be real.');
 else
    LocalUSSError('Nominal initial condition vector is not valid.');
 end
 % Validate Uncertainty initial conditions if they exist
 if strcmp(type,'User defined')
    nxext = length(csys.StateName)-nx;
    if isempty(X0unc)
       X0unc = zeros(nxext,1);
    elseif length(X0unc)==nxext && isa(X0unc,'double') && isreal(X0unc)
       X0unc = X0unc;
    elseif length(X0unc)==1 && isa(X0unc,'double') && isreal(X0unc)
       X0unc = X0unc*ones(nxext,1); %same initial condition for all states 
    elseif ~any(length(X0unc)==[1 nxext])
       LocalUSSError(...
          'Length of Uncertain Initial State vector does not match number of states.')
    elseif ~ireal(X0unc)
       LocalUSSError('Uncertain initial condition vector must be real.');
    else
       LocalUSSError('Uncertain initial condition vector is not valid.');
    end
 else
    X0unc = [];
 end
 IC = [X0; X0unc];
 NomStateFlg = ~isempty(X0);
 UncStateFlg = ~isempty(X0unc);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [sys,lerror] = LOCALliftUSS(sys) 
lerror = [];
if isa(sys,'ureal') || isa(sys,'ultidyn') || isa(sys,'umat') & ~isempty(sys)
   sys = uss(sys);
end

if isa(sys,'lti')         %lift to a USS and run
    sys = uss(sys);
elseif isa(sys,'double') && ~isempty(sys) %lift to a USS and run
    sys = uss(sys);
elseif isa(sys,'ufrd') % UFRDs not supported
    sys = [];
    lerror = ['The USS block does not support UFRD models.'];
% elseif isa(sys,'ucomplex') || isa(sys,'ucomplexm') || isa(sys,'udyn')
%     lerror = ['The USS block does not support UCOMPLEX, UCOMPLEXM and UDYN objects.'];
% elseif isa(sys,'ureal') || isa(sys,'ultidyn') || isa(sys,'umat')
elseif isuncertain(sys)
    sys = uss(sys);
end
if isa(sys,'uss')
   if ndims(sys)>2,
      % USS Arrays not supported
      lerror = ['The USS block does not support multi-dimensional USS arrays'];
   elseif ~isreal(sys)   % all uncertain either UREAL and/or ULTIDYN
      lerror = ['USS blocks do not support UCOMPLEX, UCOMPLEXM and UDYN objects.'];
   end
elseif ~isempty(sys)
      lerror = ['USS blocks do not support this type object.'];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function LocalUSSError(msg)
% Display error messages

InitFlag=strcmp(get_param(bdroot(gcb),'SimulationStatus'),'initializing');
if InitFlag,
   %errordlg(msg,'Simulink Initialization Error','replace');
   error(msg);
else
   %errordlg(msg,'USS Block Error','replace');
   error(msg);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Value = LocalEvaluate(VarName,Model)
% Evaluates model variable in appropriate workspace.
if isempty(VarName)
   Value = [];
else
    try
        Value = slResolve(VarName, Model);
    catch
        Value = [];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Value = LocalAnyUltidyn(sys)

[m,delta,blks] = lftdata(sys);
Value = any(strcmp({blks.Type},'ultidyn'));
