function [uset,pathinfo] = ufind(varargin)
%UFIND  Finds all uncertain variables in a Simulink model.
%
%   UVARS = UFIND('mdl') finds the "Uncertain State Space" blocks in the 
%   Simulink model 'mdl' and returns a structure UVARS summarizing the
%   uncertainty in these blocks. The field names and values of UVARS are
%   the names and characteristics of the uncertain variables. For example, 
%   if a Simulink model contains two Uncertain State Space blocks G and ACT, 
%   G depends on uncertain real parameters K and M, and ACT has uncertain 
%   dynamics UACT, then UVARS is a structure with fields K, M, UACT and  
%   field values of class @ureal, @ureal, and @ultidyn.
% 
%   UFIND can find Uncertain State Space blocks inside Masked Subsystems, 
%   Library Links, and Normal-Mode Model References but not inside 
%   Accelerated Model References. An error is thrown if two blocks use the 
%   same uncertain variable name to describe different uncertain elements.
% 
%   [UVARS,PATHINFO] = UFIND('mdl') also returns a cell array PATHINFO with 
%   the path to each Uncertain State Space block and the uncertain variables
%   that this block references. The first column of PATHINFO lists the block 
%   paths and the second column lists the uncertain variables for each block. 
%   For example,
%          'mymodel/G'                 'K'    
%          'mymodel/G'                 'M'    
%          'mymodel/Subsystem/ACT'    'UACT'    
%   You can use PATHINFO to verify that all Uncertain State Space blocks in
%   'mdl' have been identified.
% 
%   UVARS = UFIND(USYS1,USYS2,ÿ) gathers all uncertain variables referenced
%   by the uncertain USS or UFRD models USYS1,USYS2,... This syntax provides
%   a robust and efficient alternative to querying the Simulink model when 
%   the list of USS variables referenced by the model is readily available.
%
%   See also USAMPLE, RCTBLOCKS, USIM_DEMO.

%   Author(s): P. Gahinet
%   Copyright 2005-2012 The MathWorks, Inc.

% Support strings arguments
[varargin{:}] = convertStringsToChars(varargin{:});

ni = nargin;
if ni<1
   error('UFIND requires one or more input.')
else
   SLFlag = ischar(varargin{1});
   if SLFlag
      % Compile a list of all USS models referenced by MDL
      try
         [USSModels,USSPaths] = localFindUSS(varargin{1});
      catch ME
         throw(ME)
      end
   else
      USSModels = varargin;
      USSPaths = strseq('Input ',1:ni);
   end
end

% Construct the list of uncertain variables
%   Column #1:  atom name
%   Column #2:  atom value
%   Column #3:  path
UList = cell(0,3);
for ct=1:length(USSModels)
   try
      [~,Delta,blk] = lftdata(USSModels{ct});
   catch %#ok<CTCH>
      % Note: Should never happen for models from USS blocks
      if isa(USSModels{ct},'lti') || isa(USSModels{ct},'double')
         continue
      else
         error('Robust:simulink:UFIND1','Input arguments must be of class USS or UFRD.');
      end
   end
   na = length(blk);
   uNames = {blk.Name};
   uPaths = repmat(USSPaths(ct),[na 1]);
   uValues = cell(na,1);
   for j=1:na
      uValues{j} = Delta.Uncertainty.(uNames{j});
   end
   UList = [UList ; uNames(:) uValues uPaths]; %#ok<AGROW>
end

% Check all atoms with the same name are equal
[~,is] = sort(UList(:,1));
UList = UList(is,:);  % sort by names
for ct=1:size(UList,1)-1
   if strcmp(UList{ct,1},UList{ct+1,1}) && ~isequal(UList{ct,2},UList{ct+1,2})
      if SLFlag
         error('Robust:simulink:UFIND2',...
            'Uncertain variable "%s" has inconsistent definitions in the blocks:\n   %s\n   %s',...
            UList{ct,1},UList{ct,3},UList{ct+1,3})
      else
         error('Robust:simulink:UFIND3',...
            'Uncertain variable "%s" has inconsistent definitions in input arguments %s and %s.',...
            UList{ct,1},UList{ct,3}(7),UList{ct+1,3}(7))  % path = 'Input xxx'
      end
   end
end

% Construct outputs
[uNames,iu] = unique(UList(:,1));
uset = cell2struct(UList(iu,2),uNames,1);
pathinfo = sortrows(UList(:,[3 1]));

%---------------- Local Functions -----------------------------------
function [USystems,Paths] = localFindUSS(mdl)
% Finds the Uncertain State Space blocks in a Simulink model and returns:
%   1) The value of the "Uncertain system variable" parameter for each block
%   2) The path to each block.
% USYSTEMS and PATHS are cell arrays of @uss objects and strings, respectively.

% Prepare the model for compilation, disable warnings
ModelParameterMgr = slcontrollib.internal.mdlcfg.ParameterManager(mdl);
ModelParameterMgr.loadModels;
ModelParameterMgr.prepareModels;

% Throw error if there are obsolete USS blocks
models = ModelParameterMgr.getUniqueNormalModeModels;
for ct = 1:numel(models)
   if ~isempty(find_system(models{ct},'FollowLinks','on',...
         'LookUnderMasks','all',...
         'ReferenceBlock','RCTobsolete/USS System'));
      ModelParameterMgr.restoreModels;
      ModelParameterMgr.closeModels;
      error('Robust:simulink:UFIND4',...
         'This model has obsolete versions of the USS block. Use SLUPDATE to upgrade to the latest version.');
   end
end

% Get the list of uncertain state space blocks
Paths = {};
for ct = 1:numel(models)
    Paths = [Paths;find_system(models{ct},'FollowLinks','on',...
                             'LookUnderMasks','all',...
                             'ReferenceBlock','RCTblocks/Uncertain State Space')]; %#ok<AGROW>
end

% Compile the model to get the uncertain state space block USystems
try
    feval(mdl,[],[],[],'compile');
catch Ex
    % Restore the old settings
    ModelParameterMgr.restoreModels;
    ModelParameterMgr.closeModels;
    rethrow(Ex)
end

% Loop over the normal mode model references
npaths = numel(Paths);
USystems = cell(npaths,1);
for ct = 1:npaths
    MaskWorkspace = get_param(Paths{ct},'MaskWSVariables');
    USystems{ct} = MaskWorkspace(strcmp('USystem',{MaskWorkspace.Name})).Value;
end
 
% Terminate the model compilation
feval(mdl,[],[],[],'term')
 
% Restore the old settings
ModelParameterMgr.restoreModels;
ModelParameterMgr.closeModels;

% LocalWords:  UVARS UACT PATHINFO mymodel USYS RCTBLOCKS USIM Gahinet UVARS
% LocalWords:  USYSTEMS RCTobsolete UACT PATHINFO mymodel USYS RCTBLOCKS USIM
% LocalWords:  Gahinet USYSTEMS RCTobsolete RCTblocks UVARS UACT PATHINFO UVARS
% LocalWords:  mymodel USYS RCTBLOCKS USIM Gahinet USYSTEMS RCTobsolete UACT
% LocalWords:  RCTblocks USystems MaskWSVariables PATHINFO mymodel USYS
% LocalWords:  RCTBLOCKS USIM Gahinet USYSTEMS RCTobsolete RCTblocks USystems
% LocalWords:  MaskWSVariables USystem
