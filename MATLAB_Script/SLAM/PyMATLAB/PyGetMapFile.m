function [result] = PyGetMapFile()
%PYGETMAPFILE Summary of this function goes here
%   Detailed explanation goes here
    global map

    occMatrix = getOccupancy(map);
    filename = [pwd, '/Maps/map_', datestr(now)];

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