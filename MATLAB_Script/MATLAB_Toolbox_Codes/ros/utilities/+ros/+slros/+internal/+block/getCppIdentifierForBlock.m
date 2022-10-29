function cppId = getCppIdentifierForBlock(block, prefix)
%This function is for internal use only. It may be removed in the future.

%getCppIdentifierForBlock - Get C++ identifier for a Simulink block
%   getCppIdentifierForBlock(BLK, PREFIX) generates a unique C++
%   identifier for a Simulink block BLK and a prefix string PREFIX.

%   Copyright 2014-2018 The MathWorks, Inc.

if ~exist('prefix', 'var')
    prefix = '';
end

% blockId is a block identifier of the format '<modelname>:<num>'
% if the block is a library block, then the identifier has the format
% '<modelname>:<num>:<num>'.
blockId = Simulink.ID.getSID(block);


% Remove all leading digits & whitespace chars, convert all other
% non-numeric non-alpha to '_'. Maximum C++ variable length is at least
% 1024, so there is no risk of overflowing (Simulink model names obey
% NAMELENGTHMAX, which is 63).

cppId = [prefix regexprep(blockId, {'^\d*', '\s*', '\W'}, {'','','_'})];
