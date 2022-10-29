function blkType = fusionBlocksAndObjectMapping(objClass)
%   Map MATLAB object to equivalent Simulink block.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

switch objClass
    case 'trackerGNN'
        blkType = 'motalgorithmslib/Global Nearest Neighbor Multi Object Tracker';
    case 'trackerJPDA'
        blkType = 'motalgorithmslib/Joint Probabilistic Data Association Multi Object Tracker';
    case 'trackerTOMHT'
        blkType = 'motalgorithmslib/Track-Oriented Multi-Hypothesis Tracker';
    case 'trackerPHD'
        blkType = 'motalgorithmslib/Probability Hypothesis Density Tracker';
    case 'trackFuser'
        blkType = 'motalgorithmslib/Track-To-Track Fuser'; 
    otherwise
        error(message('fusion:simulink:exportToSimulink:ExportNotSupported',objClass));
end
end

