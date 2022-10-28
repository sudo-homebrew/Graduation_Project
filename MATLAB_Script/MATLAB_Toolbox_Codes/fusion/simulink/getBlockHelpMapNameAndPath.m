function [mapName, relativePathToMapFile, found] = getBlockHelpMapNameAndPath(block_type)
%This function is for internal use only. It may be removed in the future.

% Returns the mapName and the relative path to the maps file for this
% block_type

% Copyright 2019-2021 The MathWorks, Inc.

blks = {...
    'fusion.simulink.trackerGNN'                'fusion_gnn_mot_block'          ;...
    'fusion.simulink.trackerJPDA'               'fusion_jpda_mot_block'         ;...
    'fusion.simulink.trackerTOMHT'              'fusion_tomht_mot_block'        ;...
    'fusion.simulink.trackFuser'                'fusion_track_fuser_block'      ;...
    'fusion.simulink.trackOSPAMetric'           'fusion_ospa_metric_block'      ;...
    'fusion.simulink.trackGOSPAMetric'          'fusion_gospa_metric_block'     ;...
    'fusion.simulink.trackerPHD'                'fusion_phd_mot_block'          ;...
    'fusion.simulink.trackingScenarioReader'    'fusion_scenario_reader_block'  ;...
    'fusion.simulink.trackerGridRFS'            'fusion_gridrfs_mot_block'      ;
};

relativePathToMapFile = '/fusion/helptargets.map';

% See whether or not the block is a Sensor Fusion and Tracking Toolbox block
idx = strcmp(block_type, blks(:,1));

if ~any(idx)
    found = false;
    mapName = 'User Defined';
else
    found = true;
    mapName = blks(idx,2);
end