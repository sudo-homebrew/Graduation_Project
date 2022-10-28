function usimfill(sname,str,unctype)
%USIMFILL  Command line manipulation of dialog settings for all USS System
%          blocks in a Simulink model. 
%
% USIMFILL(SNAME,STR) inserts STR in all "Uncertainty Variable Name" dialog 
% boxes associated with the USS System blocks within the Simulink model 
% SNAME.  STR is a character string.
%
% USIMFILL allows the user to book keep all the uncertain instances in an
% uncertain Simulink model. STR is the variable name of a structure whose
% fields contain all the uncertain instances in the Simulink model. This
% ensures that copies of the same uncertainty spread through out the model
% use the same instance. USIM_DEMO provides an example demonstrating the 
% use of USIMFILL.
%
% USIMFILL(MODEL,'Uncertainty value',UNCTYPE) sets the 'Uncertainty 
% value' pull down menu of each USS Simulink block to the value 
% in UNCTYPE. Acceptable values for UNCTYPE are 'Nominal' and 'User defined'. 
% Partial matching is used and hence USIMFILL(MODEL,'U','N') and
% USIMFILL(MODEL,'U','U') are acceptable calls.
%
%See also: usim_demo, usiminfo, usimsamp, usubs.

% Authors: Gary J. Balas and Andy K. Packard
%   Copyright 2007-2009 The MathWorks, Inc.
warning('Robust:obsolete:USIMFILL','USIMFILL is deprecated. See USIM_DEMO for alternatives.')
load_system(sname);
allsys = find_system(sname,'LookUnderMasks','all','Tag','USS'); % Finds all USS blocks

for i=1:length(allsys)
   if nargin ==2
      % Push STR into Uncertainty Variable Name
      set_param(allsys{i},'logname',str)
   elseif nargin ==3 && strncmpi(str,'Uncertainty value',1);
      % Set Uncertainty Selection
      if strncmpi(unctype,'Nominal',1)
         set_param(allsys{i},'unctype','Nominal')
      else
         set_param(allsys{i},'unctype','User defined')
      end
   end
end 
