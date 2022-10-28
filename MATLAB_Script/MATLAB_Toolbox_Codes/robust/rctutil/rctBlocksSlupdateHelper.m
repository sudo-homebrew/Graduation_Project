function rctBlocksSlupdateHelper(h)
% Helper function for SLUPDATE.
% Not intended to be called directly from the command line.

%   Copyright 1990-2009 The MathWorks, Inc.
replaceInfo = getReplaceInfo();

context     = getContext(h);
% work our way up from whatever context we get to the model
while strcmpi(get_param(context,'Type'), 'block')
   context = get_param(context,'Parent');
end

for idx=1:size(replaceInfo)
   args = replaceInfo(idx).BlockDesc;
   % @todo update the usage of edit-time filter filterOutInactiveVariantSubsystemChoices(),
   % instead use the post-compile filter activeVariants().
   blocks = find_system(context, 'LookUnderMasks','all', ...
       'MatchFilter', @Simulink.match.internal.filterOutInactiveVariantSubsystemChoices, ... % look only inside active choice of VSS
       args{:});
   
   % if any are found, call the ReplaceFcn
   for blkIdx=1:numel(blocks),
      feval(replaceInfo(idx).ReplaceFcn, blocks{blkIdx}, h);
   end
   
end

end

% This function sets up replacement information for the obsolete blocks in
% Robust Control Toolbox
function ReplaceInfo = getReplaceInfo

ReplaceInfo = struct(...
   'BlockDesc',{{'MaskType','USS Block','ReferenceBlock','RCTobsolete/USS System'}},...
   'ReplaceFcn','ReplaceUSSBlock');

end

%ReplaceLTIBlock Prompt for update to LTI Block
function ReplaceUSSBlock(block, h)

if askToReplace(h, block)
   load_system('RCTblocks')
   MaskVal = get_param(block,'MaskValues');
   if strcmp(MaskVal{3},'Nominal') || isempty(MaskVal{4})
      % Set "Uncertainty value" to [] when old block was in Nominal mode or
      % the "User-defined uncertainty" field was left blank
      UValStr = '[]'; 
   else
      UValStr = MaskVal{4};
   end
   funcSet = uReplaceBlock(h, block,'RCTblocks/Uncertain State Space',...
      'USystem',MaskVal{1},'UValue',UValStr,'X0',MaskVal{2},'uX0',MaskVal{5});
   appendTransaction(h, block, h.ReplaceBlockReasonStr, {funcSet});
end

end % ReplaceSSBlock



