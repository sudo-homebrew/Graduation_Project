function [result] = PyGetMapFile()
%PYGETMAPFILE Summary of this function goes here
%   Detailed explanation goes here
    global slamAlg
    
    maxLidarRange = 8;
    mapResolution = 20;
    
    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
    
    filename = [pwd, '/Maps/map', datestr(now)];

    figure; 
%     h = show(map);

    saveas(show(map), [filename, '.pgm'])
    
    data.image = [filename, '.pgm'];
    data.mode = 'trinary';
    data.resolution = map.Resolution;
    data.origine = map.LocalOriginInWorld;
    data.origine = [data.origine, 0];
    data.negate = 0;
    data.occupied_thresh = map.OccupiedThreshold;
    data.free_thresh = map.FreeThreshold;

    yaml.dumpFile(filename + ".yaml", data)
    

    result = true;
end
