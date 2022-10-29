function [cflag,allupaths,allunames,upaths,unames,csumchar,...
syscell,fnames,iiunq,jjunq,fnidx,infocell,ucell,...
diffsysdata,dflag,incompatible,errorstr,syscellname] = ...
usiminfo(sname,silent)
%USIMINFO  Find USS System blocks within a specified Simulink model and
%             checks for consistency.
%
% It is possible to have uncertain atoms of the same name throughout
% a Simulink model.  The helper functions USIMSAMP and USIMFILL assume
% that these represent the same entity.  Hence uncertain atoms of the
% same name should have the same object properties and Uncertainty
% Selection value in the USS System pull down menu.  USIMINFO returns the
% locations of all USS System blocks within a Simulink model and determines
% if these compatibility conditions are satisfied.
%
% [CFLAG,ALLUPATHS,ALLUNAMES,UPATHS,UNAMES,CSUMCHAR] = USIMINFO(SNAME,SILENT)
%
% Inputs:
%  SNAME     - Simulink diagram name
%  SILENT    - Display inconsistencies between uncertain atoms if isn't
%              empty. Default is empty.
%
% Outputs:
%  CFLAG     - Compatibility flag set to 1 if all uncertainties are
%              consistent, set to 0 if an uncertainty definition(s) is
%              inconsistent and set to -1 if common uncertainties in
%              different blocks have different Uncertainty Selection.
%  ALLUPATHS - Path names of USS blocks in the model (cell).
%  ALLUNAMES - Uncertainties names in Simulink model (cell).
%  UPATHS    - Path names associated with each ALLUNAMES entry (cell).
%  UNAMES    - Uncertainty names associated with each ALLUPATHS entry (cell).
%  CSUMCHAR  - Character array with description of uncertainties and their
%               associated block path names. Empty if there is a conflict
%               with UNAMES.
%
% See also: usubs, usimfill, usimsamp


% Undocumented calling arguments
%
% [CFLAG,ALLUPATHS,ALLUNAMES,UPATHS,UNAMES,CSUMCHAR,...
%   SYSCELL,FNAMES,IIUNQ,JJUNQ,FNIDX,INFOCELL,UCELL,
%   DIFFSYSDATA,DFLAG,INCOMPATIBLE,ERRORSTR] =  USIMINFO(SNAME,SILENT)
%
% Additional Outputs Variables:
%  SYSCELL    - Cell of USS/UMAT blocks in the model
%  FNAMES     - All uncertainties names in the model.
%  IIUNQ      - Index of FNUNIQUE in FNAMES
%  JJUNQ      - Index of FNAMES in FNUNIQUE
%  FNIDX      - Index of non-unique uncertainty names in FNAMES
%  INFOCELL   - Index of non-unique uncertainty names in FNAMES
%  UCELL      - For each uncertainty, the block path names
%DIFFSYSDATA  - Block information about consistency of USS/UMAT data and
%               uncertain variable when User Specfied Uncertainty selected.
%  DFLAG      - Compatibility vector associated with each USS/UMAT block.
%               Entry is 1 'User Specified Uncertainty' selected and the
%               uncertain system and uncertain data are consistent or 'Nominal'
%               is selected. Entry set to 0 if uncertain system and
%               uncertain data.
%INCOMPATIBLE - Paths to the USS blocks with incompatible Uncertainty
%               data. Matrix of strings.
%  ERRORSTR   - Cell of length # of uncertain objects containing error
%               messages associated with incorrect assignment of
%               uncertainty. ERRORSTR is [] if there are no errors.
%

% Find all the USS blocks. Note that each USS block's 'Tag' property will
% be set to 'USS'

% Authors: Gary J. Balas and Andy K. Packard
%   Copyright 2005-2011 The MathWorks, Inc.
warning('Robust:obsolete:USIMINFO',...
   'USIMINFO and the "USS System" block are deprecated. Use SLUPDATE to upgrade these blocks and UFIND to extract the uncertain variables.')
if nargin==1
   silent = [];
end
% everything is consistent
cflag = 1;

load_system(sname);

% Disable warnings thrown by the USS block when the uncertainty value is
% not defined
sw = warning('off','Robust:BlockUncertaintyValueNotDefined');
feval(sname,[],[],[],'compile');
allsys = find_system(sname,'LookUnderMasks','all','Tag','USS'); % Finds all USS blocks
mv = get_param(allsys,'MaskWSVariables'); % same size as ALLSYS
feval(sname,[],[],[],'term');
warning(sw);

szall = size(allsys);
syscell     = cell(szall);
syscellname = cell(szall);
TSList      = zeros(szall);
datacell    = cell(szall);
Deltacell   = cell(szall);
infocell    = cell(szall);
diffsysdata = cell(szall);
fnames = [];
fnidx = 1;

% Work out mask variable location in struct array MV{1}
if ~isempty(mv)
   structfields = {mv{1}.Name};
   SYS_IDX = strcmp('sys',structfields);
   SYSNAME_IDX = strcmp('sysname',structfields);
   UNCTYPE_IDX = strcmp('unctype',structfields);
   TS_IDX = strcmp('Ts',structfields);
   LOGNAME_IDX = strcmp('logname',structfields);
   UNCSTATE_IDX = strcmp('UncStateFlg',structfields);
end

for i=1:szall(1)
   % 13Oct08 GJB mods
   % sysname = get_param(allsys{i},'sys');
   % syscell{i} = uss(LocalEvaluate(sysname,sname));
   % TSlist(i) = get(syscell{i},'Ts');
   % uncertainty variable
   % datanamex = get_param(allsys{i},'logname');
   syscell{i} = mv{i}(SYS_IDX).Value; % USS variable SYS VALUE
   syscellname{i} = mv{i}(SYSNAME_IDX).Value; % USS variable SYS Name
   TSList(i)  = mv{i}(TS_IDX).Value; % USS variable Ts
   dataname   = mv{i}(LOGNAME_IDX).Value; % USS variable LOGNAME
   if isa(syscell{i},'uss')
      fn = fieldnames(syscell{i}.Uncertainty);
      [~,del] = lftdata(syscell{i});
   else
      tmpsys = uss(syscell{i});
      fn = fieldnames(tmpsys.Uncertainty);
      [~,del] = lftdata(tmpsys);
   end
   Deltacell{i} = del;
   fnames = [fnames; fn]; %#ok<*AGROW>
   % Uncertainty Names for each USS block
   infocell{i} = fn;
   % Indices to Uncertainty Names
   fnidx(i+1) = fnidx(i)+length(fn);
   % 13 Oct 08 GJB: 
   if ~isempty(dataname)
      if isa(datacell{i},'struct')
         dn = fieldnames(datacell{i});
      else
         dn = [];
      end
   else
      dn = [];
   end
   % Check each USS for uncertain data consistency
   % if ~strcmp(get_param(allsys{i},'unctype'),'Nominal')
   %
   % diffsysdata{i}=[]   if 'Nominal' or 'User defined' uncertainty data
   %                      was in 'datacell' and was consistent
   % diffsysdata{i}=char wasn't 'Nominal', uncertainty name not in 'datacell'
   % diffsysdata{i}=-1   wasn't 'Nominal', uncertainty name in is 'datacell'
   %                      but was not consistent with its definition
   if ~mv{i}(UNCSTATE_IDX).Value  % Is Uncertain State Flag NOT 'Nominal'
       cn = setdiff(fn,dn);          % in FN, missing from DN
       if ~isempty(cn)
           % Uncertainty data missing from 'datacell'
           diffsysdata{i} = cn;
       else %same field names in USYS and uncertain data
           try
              usubs(syscell{i},datacell{i});
           catch
              % Correct uncertainty names, but data wasn't consistent with
              % definition of uncertainty in USS block i
              diffsysdata{i} = -1;
           end
       end
   end
end
% DFLAG is 1 if uncertainty data OK
dflag = cellfun('isempty',diffsysdata);

%Find if there are the same uncertain elements in several USS blocks
[fnunique,iiunq,jjunq] = unique(fnames);
nUnc = length(fnunique); % # of unique uncertainty in Simulink model

% Uncertainty and associated block paths
ucell = cell(nUnc,3);
for i=1:nUnc                  % Go through each uncertainty
   ctmp = cell(0,1);
   dtmp = zeros(0,1);
   for j=1:length(allsys)     % Check each block 1-by-1
      ind = fnidx(j):fnidx(j+1)-1; % Uncertainty name index
      % Compares uncertainty in ALLSYS{i} with unique uncertainty name
      tmp = strcmp(fnunique{i},fnames(ind)); % i'th uncertainty name
      if any(tmp)                            % Uncertain name in this block
         ctmp = [ctmp;allsys(j)];            % Vector of uncertain SYS with same uncertainty
         dtmp = [dtmp;j];                    % Vector of SYS indicies
      end
   end
   ucell{i,1} = ctmp;
   ucell{i,2} = dtmp;
end
pathLeft = cell(0,1);
pathRight = cell(0,1);
errorstr = cell(nUnc,1);

for i=1:nUnc                      % Go through each uncertainty
   Nplaces = length(ucell{i,1});  % # times uncertainty appears in USS blocks
   atomcell = cell(Nplaces,1);
   for k=1:Nplaces
      tmpsys = uss(syscell{ucell{i,2}(k)});
      atomcell{k} = tmpsys.Uncertainty.(fnunique{i});
   end
   ucell{i,3} = evalc('atomcell{1}');    % display string for uncertainty 
   ucheck  = ones(length(ucell{i,1}),length(ucell{i,1}));
   ucheck2 = ones(length(ucell{i,1}),length(ucell{i,1}));
   TScheck = ones(length(ucell{i,1}),1);
   for k=1:length(ucell{i,1})
      TScheck(k) = TSList(ucell{i,2}(k));
      for kk=k+1:length(ucell{i,1})
         ucheck(k,kk) = isequal(atomcell{k},atomcell{kk});
         ucheck2(k,kk) = isequal(mv{ucell{i,2}(k)}(UNCTYPE_IDX),...
                                  mv{ucell{i,2}(kk)}(UNCTYPE_IDX));
      end
   end
   [iloc,jloc] = find(ucheck==0);
   tmpstr = [];
   if ~isempty(iloc)
      pathLeft{i,1}  = ucell{i,1}(iloc);
      pathRight{i,1} = ucell{i,1}(jloc);
      cflag = 0;                      %inconsistent uncertainty definitions
      tmpstr = ['Uncertain atom ' '''' fnunique{i} '''' ...
                ' is defined differently in USS blocks: '];
      for  k=1:length(iloc)
         tmpstr = str2mat(tmpstr,...
             [' ' pathLeft{i}{k} '  *NOT EQUAL TO*  ' pathRight{i}{k}]);
      end
      if ~isempty(silent)
         disp(tmpstr)
      end
   else
      [iloc,jloc] = find(ucheck2==0);
      if ~isempty(iloc)
         pathLeft{i,1} = ucell{i,1}(iloc);
         pathRight{i,1} = ucell{i,1}(jloc);
         cflag = -1;        %inconsistent uncertainty Uncertainty Selection
         tmpstr = ['Uncertain atom ' '''' fnunique{i} '''' ...
           '  Uncertainty Selection is defined differently in USS blocks: '];
         for  k=1:length(iloc)
            tmpstr = str2mat(tmpstr,...
                ['  ' pathLeft{i}{k} '  *NOT EQUAL TO*  ' pathRight{i}{k}]); %#ok<*DSTRMT>
         end
         if ~isempty(silent)
            disp(tmpstr)
         end
      else
         if isa(atomcell{1},'ultidyn')
            [TSU,TSI] = unique(TScheck);
            if length(TSU)>1
               cflag = -2;
               tmpstr = ['ULTIDYN ' '''' fnunique{i} '''' '  SampleTime Inconsistency'];
               for  k=1:length(TSU)
                   tmpstr = str2mat(tmpstr,['  SampleTime in ' ...
                       allsys{TSI(k)} '  equals  ' num2str(TScheck(TSI(k)))]);
               end
               if ~isempty(silent)
                  disp(tmpstr)
               end
            end
         end
      end
   end
   errorstr{i} = tmpstr;
end

allupaths = allsys;
allunames = fnunique;
upaths    = ucell(:,1);
unames    = infocell;
incompatible = [pathLeft pathRight];

sumcell = cell(0,1);
if isempty(incompatible)
   for i=1:size(ucell,1)
     sumcell = [sumcell;ucell{i,3}];
     for k=1:length(ucell{i,1})
        sumcell = [sumcell; {[' - ' ucell{i,1}{k}]}];
     end
   %  sumcell = [sumcell; {[' ']}];
   end
end
csumchar = char(sumcell);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GJB 21Nov06
% function Value = LocalEvaluate(VarName,Model)
% % Evaluates model variable in appropriate workspace.
% if isempty(VarName)
%    Value = [];
% else
%    try
%       Value = evalin('base',VarName);
%    catch
%       try
%          ModelWS = get_param(Model,'ModelWorkspace');
%          Value = ModelWS.evalin(VarName);
%       catch
%          Value = [];
%       end
%    end
% end
