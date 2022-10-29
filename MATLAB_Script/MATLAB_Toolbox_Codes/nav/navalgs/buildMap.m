function map = buildMap(scans, poses, mapResolution, maxLidarRange, varargin)
%buildMap Build a occupancyMap map object from scans and poses
%   MAP = buildMap(SCANS, POSES, MAPRESOLUTION, MAXLIDARANGE) creates a
%   occupancyMap map MAP by inserting SCANS at the corresponding
%   POSES. SCANS is a cell array of lidarScan objects, and POSES is an
%   N-by-3 matrix, where N is the number of lidarScan objects in SCANS. The
%   resolution of the resultant map MAP is specified by the MAPRESOLUTION
%   input argument as number of grid cells per meter. The maximum
%   acceptable range for each lidar scan is specified by MAXLIDARANGE in
%   meters.
%
%   ___ = buildMap(___, Name, Value) provides additional options
%   specified by one or more Name,Value pair arguments. Name must appear
%   inside single quotes (''). You can specify several name-value pair
%   arguments in any order as Name1, Value1, ..., NameN, ValueN:
%
%   'MapWidth'  - The width of the map in meters. If not provided, the 
%                 width is automatically computed to fit all the scans in 
%                 SCANS.
%
%   'MapHeight' - The height of the map in meters. If not provided, the 
%                 height is automatically computed to fit all the scans in 
%                 SCANS.
%
%   See also lidarScan.

%   Copyright 2017-2020 The MathWorks, Inc.

%#codegen

    narginchk(4, inf);

    validateattributes(scans, {'cell'}, {'nonempty'}, 'buildMap', 'scans');
    N = numel(scans);
    for i = 1:N
        validateattributes(scans{i}, {'lidarScan'}, {'nonempty', 'scalar'},  'buildMap', 'scans');
    end

    validateattributes(poses, {'numeric'}, {'size', [N, 3], 'nonnan', 'finite','nonempty'}, 'buildMap', 'poses')
    mapResolution = robotics.internal.validation.validatePositiveIntegerScalar(mapResolution, 'buildMap', 'mapResolution');
    maxLidarRange = robotics.internal.validation.validatePositiveNumericScalar(maxLidarRange, 'buildMap', 'maxLidarRange');

    %estimate map size
    positions = poses(:,1:2);
    mapSize = max(positions) - min(positions) + 2*[maxLidarRange maxLidarRange];
    mapSize = ceil(mapSize);

    defaults = struct('MapWidth', mapSize(1), ...
                      'MapHeight', mapSize(2));

    names = {'MapWidth', 'MapHeight'};
    defaultValues = {defaults.MapWidth, defaults.MapHeight};

    % Parse name-value pairs
    parser = robotics.core.internal.NameValueParser(names, defaultValues);
    parse(parser, varargin{:});

    mapWidth = parameterValue(parser, 'MapWidth');
    mapHeight = parameterValue(parser, 'MapHeight');

    mapWidth = robotics.internal.validation.validatePositiveNumericScalar(mapWidth, 'buildMap', 'MapWidth');
    mapHeight = robotics.internal.validation.validatePositiveNumericScalar(mapHeight, 'buildMap', 'MapHeight');

    map = occupancyMap(mapWidth, mapHeight, mapResolution);
    map.GridLocationInWorld = min(positions)-[maxLidarRange, maxLidarRange];
    for i = 1:N
        sc = scans{i}.removeInvalidData('RangeLimits', [0.01 maxLidarRange]);
        map.insertRay(poses(i,:), sc, maxLidarRange);
    end

end
