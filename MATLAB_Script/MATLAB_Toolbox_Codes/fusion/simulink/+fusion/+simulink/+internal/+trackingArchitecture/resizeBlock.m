function newPos = resizeBlock(blkHndl,height,width)
%   Resize a Simulink block and return the new position of the block.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

p = get_param(blkHndl,'position');
newPos = [p(1),p(2),p(1)+width,p(2)+height];
set_param(blkHndl,'position',newPos);
end