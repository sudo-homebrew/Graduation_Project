function smultiplot(name,button)
%SMULTIPLOT  Clear plot or save data to workspace from an Simulink 
%  MultiPlot Graph block.
%
% SMULTIPLOT(H,'Clear') clears the MultiPlot Graph Display associated
% with the block handle or block name H. If H is the name of a Simulink
% model then all MultiPlot Graph Displays in the model are cleared.
%
% SMULTIPLOT(H,'Save') exports to the workspace the MultiPlot Graph data 
% associated with the block handle or block name H. The exported data 
% appears in the workspace variable name listed in the "Variable for 
% Export for Workspace" dialog box. If H is the name of a Simulink model 
% then data from all MultiPlot Graph Displays in the model is exported. 

%   Copyright 1986-2020 The MathWorks, Inc.
if isa(name,'char') && strcmp(bdroot(name),name)
   % Name refers to a system 
   blknames = find_system(name,'LookUnderMasks','all','Tag','MultiPlot');
else
   % Name refers to a MultiPlot Graph
   blknames = {name};
end   

for i1 = 1:length(blknames)
   if strncmpi(button,'clear',1)
      sfun_multiplot([],[],[],'Clear',blknames{i1})
   elseif  strncmpi(button,'save',1)
      sfun_multiplot([],[],[],'Save',blknames{i1})
   end
end

