function positionBlock(blkHndl,xpos,ypos)
%   Position block in the model.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

p = get_param(blkHndl,'position');
width =  p(3)-p(1);
height = p(4)-p(2);
set_param(blkHndl,'position',[xpos,ypos,xpos+width,ypos+height]);
end