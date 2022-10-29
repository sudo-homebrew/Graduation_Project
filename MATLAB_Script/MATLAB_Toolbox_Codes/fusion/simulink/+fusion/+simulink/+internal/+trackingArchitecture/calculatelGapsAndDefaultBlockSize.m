function [verticalGap,horizontalGap,blkHeight,blkWidth] = calculatelGapsAndDefaultBlockSize()
%Find default size for blocks and calculate gaps between nodes.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

verticalGap = 150;
fixedOffset = 245;
sys = {'motalgorithmslib','trackingutilitieslib'};
load_system(sys);
%Find position of a node in fusion library.
p = get_param('motalgorithmslib/Track-To-Track Fuser','position');
blkWidth  = p(3) - p(1);
blkHeight = p(4) - p(2);
p = get_param('trackingutilitieslib/Track Concatenation','position');
concatBlkWidth = p(3) - p(1);
horizontalGap = fixedOffset + blkWidth + concatBlkWidth;
close_system(sys);
end