function map = importOccupancyMap3D(mapPath)
%IMPORTOCCUPANCYMAP3D Import an octree file as a 3-D occupancy map
%   MAP = importOccupancyMap3D(MAPPATH) imports the octree file specifed at
%   MAPPATH as an occupancyMap3D object MAP. MAPPATH can be an absolute
%   path or relative path to a .ot/.bt file, which is the serailized format
%   of octree.
%
%   Example:
%       map = occupancyMap3D();
%       exportOccupancyMap3D(map, 'map.ot');
%       mapCopy = importOccupancyMap3D('map.ot');

%   Copyright 2019 The MathWorks, Inc.

validateattributes(mapPath, {'string', 'char'}, ...
    {'scalartext', 'nonempty'}, 'importOccupancyMap3D', 'mapPath');
mapPath = robotics.internal.validation.findFilePath(convertStringsToChars(mapPath), 'importOccupancyMap3D', 'mapPath');

map = nav.algs.internal.MapIO.read(mapPath);
end

