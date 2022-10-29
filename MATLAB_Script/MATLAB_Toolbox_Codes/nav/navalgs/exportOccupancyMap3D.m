function exportOccupancyMap3D(map, mapPath)
%EXPORTOCCUPANCYMAP3D Export 3-D occupancy map as an octree or binary tree file
%   exportOccupancyMap3D(MAP, FILENAME) serializes the MAP object into
%   either an octree file (.ot) that contains all the occupancy data, or a
%   binary tree file (.bt) that contains only the maximum-likelihood
%   information at the location specified by FILENAME.
%
%   Example:
%      map = occupancyMap3D();
%      exportOccupancyMap3D(map, 'map.ot');

%   Copyright 2019 The MathWorks, Inc.

validateattributes(map, {'occupancyMap3D'}, {'scalar'}, 'exportOccupancyMap3D', 'map');
validateattributes(mapPath, {'string', 'char'}, ...
    {'scalartext', 'nonempty'}, 'exportOccupancyMap3D', 'mapPath');
if exist(mapPath, 'file')
    error(message('nav:navalgs:occmap3d:FileAlreadyExist', mapPath));
end

[fid,errmsg] = fopen(mapPath, 'w');
if ~isempty(errmsg)
    error(message('nav:navalgs:occmap3d:CannotCreateFile', mapPath, errmsg));
else
    fclose(fid);
end

try
    nav.algs.internal.MapIO.write(map, convertStringsToChars(mapPath));
catch ME
    delete(mapPath);
    rethrow(ME);
end

end

