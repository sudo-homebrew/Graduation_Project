function map = mapClutter(varargin)
%mapClutter Generate map with randomly scattered obstacles
%   MAP = mapClutter generates a binaryOccupancyMap, MAP, with a
%   width and height of 50m, and resolution of 5 cells per meter. The
%   map contains 20 randomly distributed obstacles of type "Box" and
%   "Circle". Generated obstacles have random sizes.
%
%   MAP = mapClutter(NUMOBST) generates binaryOccupancyMap, MAP, with
%   default size and resolution. The number of randomly distributed
%   obstacles, NUMOBST, specified as a positive scalar integer.
%
%   MAP = mapClutter(NUMOBST, SHAPES) generates binaryOccupancyMap with
%   the specified number of obstacles, NUMOBST, and shape of the
%   obstacles, SHAPES. The valid shapes are "Box", "Circle", and
%   "Plus", specified either as a string scalar, string array, or cell
%   array of character vectors.
%
%   MAP = mapClutter(..., 'Name', Value) specifies additional options
%   using one or more name-value pair arguments to generate randomly
%   distributed obstacle map.
%
%   Following are valid Name-Value pairs for the function:
%
%       MapSize        - Width and height of the generated map,
%                        specified either as a two-element vector of
%                        [Width,Height] pairs in meters.
%                        Default: [50,50]
%
%       MapResolution  - Resolution of the generated map, specified
%                        as a positive numeric scalar in cells per
%                        meter.
%                        Default: 5
%
%   Example:
%       % Generate one binaryOccupancyMap with 10 obstacles which are
%       % of Box, Circle, and Plus type, placed in random locations.
%       % The generated map is 50m wide and 30m high with a resolution
%       % of 5.
%       map = mapClutter(10, ["Box", "Plus", "Circle"], ...
%           'MapSize', [50 30], 'MapResolution', 5);
%
%       % Visualize the generated cluttered map with 10 obstacles of
%       % Box, Plus, and Circle.
%       show(map);
%
%   See also binaryOccupancyMap, validatorOccupancyMap

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    narginchk(0,6);

    % valid inputs for shapes.
    validShapes = {'box', 'circle', 'plus'};
    % parse for all the allowed inputs and assign appropriate arg value to
    % appropriate variable. While parsing also validate for datatype and
    % allowed shapes, names. Also check for Names without values. If a
    % mistake is detected throw appropriate error.
    [numObst, shapes, mapSz, mapRes] = parseValidateArguments(validShapes,...
                                                      varargin{:});


    % Initialize binaryOccupancyMap object array with desired mapSz and
    % mapRes values with the value
    map = binaryOccupancyMap(mapSz(1), mapSz(2), mapRes);

    % pos is N-by-2. ori and scale are 2-by-2-by-N.
    [pos,ori,scale] = randObstacleInfo(mapSz(1), mapSz(2), numObst,...
                                       mapRes);
    mapShapeSz = numel(shapes);

    % Randomize order of clusters so that same shapes are not always
    % next to each other and randomly distributed.
    % This way we can ensure a uniform distribution of the number of
    % shapes and also randomize their placement.
    randIdx = randperm(size(pos,1));
    pos = pos(randIdx,:);
    ori = ori(:, :, randIdx);
    scale = scale(:, :, randIdx);

    % To avoid holes after rotation of object have the step size as
    % half of resolution.
    ptsStepSz = 1/(2*mapRes);

    % obsPts will contain all the points belonging to all the obstacles
    % in the map. Once all points are collected it will be used
    % to set all the cells as obstacle in one go.
    % Initialize the number of points with number of cells in the map.
    obsPtsSz = prod(map.GridSize);
    obsPts = zeros(2*obsPtsSz,2);
    % To track the position to add the next point.
    obsPtsCount = 1;
    for obsIdx=1:numObst % loop over number of obstacles

        % Properties for this obstacle
        obsPos = pos(obsIdx,:); % 1-by-2
        obsOri = ori(:,:,obsIdx); % 2-by-2
        obsScale = scale(:,:,obsIdx); % 2-by-2

        % Shape of obstacle is picked from the user provided shapes
        % in order. 1,2,3,1,2,3.....
        obsShapeIdx = rem(obsIdx-1,mapShapeSz) + 1;
        shape = shapes(obsShapeIdx);
        coder.varsize('p');
        p = [];
        switch(shape)

          case 1 % validShapes{1}
            p = getOrientedBoxPts(obsPos, obsOri, obsScale, ...
                                          ptsStepSz);

          case 2 % validShapes{2}
            p = getCirclePts(obsPos, obsScale, ptsStepSz);

          case 3 % validShapes{3}
            p = getPlusPts(obsPos, obsOri, obsScale, ptsStepSz);
        end

        % Append the points generated for the current obstacle to the
        % all previous obstacle points.
        obsPtsEndIdx = (obsPtsCount + size(p,1) - 1);
        obsPts(obsPtsCount:obsPtsEndIdx,:) = p;
        obsPtsCount = obsPtsEndIdx + 1;
    end  % loop over number of obstacles

    % Create binaryOccupancyMap and set all cells with obstacle points
    % to 1.
    obsPts(obsPtsCount:end, :) = [];
    obsPtsSz = size(obsPts,1);
    setOccupancy(map, obsPts, ones(obsPtsSz,1), 'world');


end % end mapClutter

function [numObst, orderedShapeIdx, mapSz, mapRes] = parseValidateArguments(...
    validShapes, varargin)
    %parseValidateArguments estimates which optional args are provided by
    %the user and calls functions to parse and validate respective
    %functions.

    % 0. Setup the default/frequently used values.
    % All the inputs to mapClutter would come through varargin, so it's
    % size should be the same as nargin for mapClutter.
    nargs = numel(varargin);

    numMaxOptionalArgs = 2;
    mapSz = [50 50];
    mapRes = 5;
    validnames = {'mapsize', 'mapresolution'}; % valid names
    defaults = {mapSz, mapRes}; % Values

    % 1. Find where the name value pairs start.
    % If name value pairs are provided nameValueIdx would be between 1 to
    % numMaxOptionalArgs+1, as only name value pairs are allowed after all
    % optional argument are mentioned.
    % In case name value pairs are not provided nameValueIdx will be equal
    % to inf. inf as compared to others (0,nan) make it easier to do
    % further comparison operations.
    nameValueIdx = coder.const(firstNameValuePosition(validnames,...
                                                      numMaxOptionalArgs, varargin{:}));

    % 2. Find args to be parsed based on first name value position and
    % number of input arguments.
    % Arg order: number of Obstacles, shapes, extra args.
    argIdx = 1:3;
    % It could be parsed if the argIdx is less than nargin.
    narginCouldParseArg = argIdx <= nargs;
    % It should be parsed if the argIdx is less than nargin.
    nameValueCouldParseArg = argIdx < nameValueIdx;
    % If both say arg could be parsed, then it should be parsed.
    shouldParseArg = and(narginCouldParseArg, nameValueCouldParseArg);

    % 3. Parse Number of Obstacles if it should be.
    if shouldParseArg(1)
        % 3.1 Assign the arg to number of obstacles variable.
        % If numObst is present it has to be first argument.
        numObst = varargin{1};
    else
        % 3.1 Assign a default value if it's not passed by the user.
        numObst = 20;
    end
    % 3.2 Validate the arg as number of obstacles.
    % e.g. mapClutter(42, ...);
    % e.g. mapClutter('Error', ...);
    validateattributes(numObst, {'numeric'}, ...
                       {'nonempty', 'scalar', 'nonnan', 'finite', 'real',...
                        'integer', 'positive', 'nonsparse'},...
                       mfilename, 'Number of Obstacles', 1);
    numObst = double(numObst);

    % 4. Parse shapes if it should be.
    if shouldParseArg(2)
        % 4.1 Assign the arg to shapes variable.
        % If shapes is to be processed then numObst is 1 and shapes is 2nd
        % arg.
        shapesBeforeValidation = varargin{2};
    else
        % 4.1 Assign default value if it's not passed by the user.
        shapesBeforeValidation = {'box', 'circle'};
    end
    % 4.2 Validate arg as shapes.
    orderedShapeIdx = parseValidateShapes(shapesBeforeValidation, validShapes);

    % 5. If extraArgs is present then check if it's a valid shape. If it is
    %    then throw custom error informing user if multiple shapes are
    %    provided then they should be in a cell array and not as multiple
    %    arguments.
    %    If it's true then an error is thrown and execution flow halts.
    %    If it's false then do nothing and let Name value parser deal with
    %    it.
    isExtraShapeThere = shouldParseArg(3) && ...
        isValidString(varargin{3}, validShapes);
    coder.internal.errorIf( isExtraShapeThere,...
                            'nav:navalgs:mapclutter:NonCellMultiShape');

    % 6. Let the Name Value parser parse whatever in the args hasn't been
    %    parsed.
    % If more arguments have been passed then treat the args after optional
    % args as if they were supposed to be NV pair. irrespective of if a NV
    % pair was detected.
    if nargs > numMaxOptionalArgs && nameValueIdx == inf
        nameValueIdx = numMaxOptionalArgs + 1;
    end
    % If namevalueIdx is inf at this then args shouldn't be parsed as name
    % value pair as NV pair wasn't specified by the user. It's parsed in
    % two scenarios:
    % 1. All optional args were not provided
    %    for e.g. ("Map..",...), (10, "Map..",..), (10, "box", "Map..")
    % 2. All optional args are passed, but illegal args are passed after that.
    %    for e.g. (10, "box", 42)
    if nameValueIdx ~= inf
        nvArgs = {varargin{nameValueIdx:end}};
        [mapSz,mapRes] = parseValidateNameValuePairs(validnames, defaults,...
                                                     nvArgs{:});
        mapSz = double(mapSz);
        mapRes = double(mapRes);
    end
end % end parseValidateArguments

function nmPos = firstNameValuePosition(validnames, numMaxOptionalArgs,...
                                        varargin)
    % return inf if no Name Value pair is present.
    nmPos = inf;
    % Ideally first Name at the max should be just after the max number of
    % optional arguments, that is numMaxOptionalArgs+1, o.w. some invalid
    % arguments have been provided. The error messages for those should be
    % provided by the Name Value parser, so can stop processing after the
    % optional inputs have been processed.
    searchEnd = min(numMaxOptionalArgs+1, numel(varargin));
    for idx=1:searchEnd
        foundName = isValidString(varargin{idx}, validnames);
        if foundName
            nmPos = idx;
            break;
        end
    end
end % end firstNameValuePosition

function [isValidStr, isStr, strIdx] = isValidString(str, validNames)
% [ISVALID, ISSTR, IDX] = isValidString(STR, VALIDNAMES) validates if
% is a string and is a member of list of strings. STR is a string
% scalar or character vector. VALIDNAMES is a cell array of string
% scalars or character vectors. ISVALID as true indicates STR matched
% with a member of VALIDNAMES. ISSTR indicates that STR is a character
% vector or string array. STRIDX indicates index of the first match in
% VALIDNAMES.

% Two conditions:
% 1. Is it a string.
% 2. Is it valid, i.e. str is a subset of at least 1 string in
%    validnames.

    isValidStr = false;
    strIdx = inf;

    isStr = coder.internal.isTextRow(str);
    if isStr
        strLower = lower(strtrim(str));
        for idx=1:numel(validNames)
            if contains(validNames{idx}, strLower)
                isValidStr = true;
                strIdx = idx;
                break;
            end
        end
    end
end % end isValidString

function [mapSz, mapRes] = parseValidateNameValuePairs(validnames, ...
                                                      defaults, varargin)
    % Check if values are arranged in NV pair format. Every alternate value
    % should be a valid name.
    parser = robotics.core.internal.NameValueParser(validnames, defaults);
    parse(parser, varargin{:});

    mapSz = parameterValue(parser, validnames{1}); % map width and height.
    validateattributes(mapSz, {'numeric'}, ...
                       {'nonempty', '2d', 'size', [1 2], 'nonnan', 'finite', 'real', ...
                        'nonsparse', 'positive'}, ...
                       mfilename, 'MapSize');

    mapRes = parameterValue(parser, validnames{2}); % map resolution
    robotics.internal.validation.validatePositiveNumericScalar(mapRes,...
                                                      mfilename, 'MapResolution');

end % end parseValidateNameValuePairs

function uniqueShapeIds = parseValidateShapes(ipShapes, validShapes)

    % Check if shapes is a cell array of chars or strings. Just test the
    isShapesTypeValid = ~iscellstr(ipShapes) && ~isstring(ipShapes);
    % If it is not then throw an error.
    coder.internal.errorIf(isShapesTypeValid,...
                           'nav:navalgs:mapclutter:invalidShapesType');

    % Convert shapes to cellstr if passed as string.
    if isStringScalar(ipShapes)
        % If it was then add a layer of cell array to make it consistent to
        % the rest of the code's expectation.
        % e.g. 'Box' => {'Box'}
        shapes = {convertStringsToChars(ipShapes)};
    elseif isstring(ipShapes)
        shapes = convertStringsToChars(ipShapes);
    else
        shapes = ipShapes;
    end

    % Validate attributes of shapes
    validateattributes(shapes, {'cell'}, {'nonempty', '2d', 'vector'},...
                       mfilename, 'shapes', 2);
    coder.internal.errorIf(numel(shapes) > 3,...
                           'nav:navalgs:mapclutter:shapesMaxNumElements');

    % Validate shapes in a consistent manner.
    numShapes = numel(shapes);
    shapesAfterValidation = cell(1, numShapes);
    shapePresent = false(1, numel(validShapes));
    shapeIds = zeros(1, numShapes);
    for idx = 1:numShapes
        shape = lower(strtrim(shapes{idx}));
        % Validate datatype of shape. e.g. 'Box'
        validateattributes(shape, {'char','string'},...
                           {'nonempty', 'scalartext'}, mfilename, 'shapes', 2);

        % Validate if specified shape is valid.
        shapesAfterValidation{idx} = validatestring(shape, validShapes,...
                                                    mfilename, 'shapes', 2);
        [~,~,shapeIdx] = isValidString(shape, validShapes);
        % Throw error if this shape has already been specified before.
        coder.internal.errorIf(shapePresent(shapeIdx),...
                               'nav:navalgs:mapclutter:multipleShapeEntries');
        shapePresent(shapeIdx) = true;
        shapeIds(idx) = shapeIdx;
    end

    uniqueShapeIds = unique(shapeIds, 'stable');

end % end parseValidateShapes

function [pos, ori, scale] = randObstacleInfo(m, n, numCenters, res)

    % Check if numCenter small obstacles can be fit into the map
    % All shapes need to be 3 cells long to be recognizable.
    obsMinCellSize = 3;
    spaceEnoughForObstacles = numCenters * obsMinCellSize > res * sqrt(m*n);
    isMapSmall = (res * sqrt(m*n)) < 10;
    % Throw an error if number of obstacles times min obstacle size is
    % greater than square root of the number of cells available in the map.
    coder.internal.errorIf(isMapSmall && spaceEnoughForObstacles, ...
                           'nav:navalgs:mapclutter:TooManyObstacles');

    % Generate a random sparse points
    % sprand alternate
    density = numCenters/(res*min(m,n));
    coder.internal.errorIf(density >= 0.5,...
                           'nav:navalgs:mapclutter:TooManyObstacles');
    nnzwanted = round(m * n * res * density);
    randX = rand(nnzwanted, 1) * m;
    randY = rand(nnzwanted, 1) * n;
    ij = unique([randX randY],'rows');
    X = ij(:,1);
    Y = ij(:,2);
    coder.varsize('X', 'Y');

    if numCenters == 1
        % 1. Only consider points inside a smaller square of 80% size of
        %    original
        withinBounds = X>0.1*m & X<0.9*m & Y>0.1*n & Y<0.9*n;
        X = X(withinBounds);
        Y = Y(withinBounds);

        % 2. Get the indices for X and Y which will dictate output
        % variables.
        idxs = randi(size(X,1), 3, 1);

        % 3. Get random position.
        pos = [X(idxs(1)) Y(idxs(1))];

        % 4. Get orientation.
        th = atan2(Y(idxs(2)), X(idxs(2)));
        ori = [cos(th) -sin(th); sin(th) cos(th)];

        % 5. Get random scale and ensure it's less that 0.2*dims, and
        %    greater than 0.0*dims
        scale = [max(0.01*m, min(X(idxs(3)), 0.2*m)) 0;...
                 0 max(0.01*n, min(Y(idxs(3)), 0.2*n))];
    else

        % K-means cluster where K is same as number of obstacles
        cidx = nav.algs.internal.kmeans([X Y], numCenters);

        sdwidth = 1; % n sigma to be covered

        pos = zeros(numCenters, 2);
        ori = zeros(2,2,numCenters);
        scale = zeros(2,2,numCenters);
        for ncIdx = 1:numCenters(1,1) % loop num cluster centers
            % Find points belonging to the current cluster
            cIidx = (cidx==ncIdx);
            Xi = [X(cIidx) Y(cIidx)];
            if size(Xi,1)<= 1
                continue;
            end
            % Cluster center is the obstacle position
            pos(ncIdx,:) = mean(Xi);
            sig = cov(Xi);
            [vout,dout]=eig(sig);
            % Make longer axis X-axis by sorting.
            dvec = sqrt(diag(dout));
            [dvec, sortIdx] = sort(dvec, 'descend');
            dout = diag(max(obsMinCellSize/res, dvec));
            d = real(dout);
            % 1st standard deviation of the eigen value provides the span of
            % the obstacle.
            scale(:,:,ncIdx) = sdwidth * d;

            % Eigen vectors provide orientation of the obstacle.
            vout = real(vout(:,sortIdx));
            % Disable orientation if the smaller side is smaller then 7
            % cell size to ensure the shapes are recognizable.
            if any(dvec < 7/res)
                vout = eye(2);
            elseif sign(vout(1,1)) ~= sign(vout(2,2))
                % Ensure it follows right handed system for rotation.
                vout(:,2) = -vout(:,2);
            end
            % To ensure MATLAB and codegen have same output.
            vout = sign(vout(1,1))*vout;
            ori(:,:,ncIdx) = vout;
        end % loop num cluster centers
    end

end % end randObstacleInfo

function pts = getOrientedBoxPts(center, v, d, stepsz)

    % d (output of eig) is used as the span of the object, multiplying d
    % with (1,1) scales the vector.
    sc = d*[1;1];
    % Since the center is on origin and generated points are -x to x,
    % scale is divided by 2.
    sc = sc./2;

    % Generate points to fill the box, where -sc(1) and sc(1) represent the
    % extremities in X, and -sc(2) and sc(2) in Y.
    [mgX, mgY] = meshgrid(-sc(1):stepsz:sc(1), -sc(2):stepsz:sc(2));
    pts = [mgX(:)'; mgY(:)']; % 2-by-N

    % Rotate using the eigen vectors, which is used as the orientation of
    % the points.
    rotPts = (v * pts)'; % N-by-2 = (2-by-N)'

    % Translate points to the cluster center, which is used as the
    % obstacle position.
    repeatedCenter = repmat(center, size(rotPts, 1), 1);
    pts = rotPts + repeatedCenter; % N-by-2

end % end getOrientedBoxPts

function pts = getCirclePts(center, d, radialStep)

    % mean of the eigen values is used as diameter.
    radius = mean(diag(d))/2;
    % Generate points in a square of side length equal to radius.
    [X, Y] = meshgrid(-radius:radialStep:radius);
    % Index of points within or on edge of the circle.
    cidx = find(sqrt(X.^2 + Y.^2)<=radius);

    % Points belonging to the circle.
    pts = [X(cidx) Y(cidx)];
    % Translate points to the cluster center, which is used as the
    % obstacle position.
    repeatedCenter = repmat(center, size(pts, 1), 1);
    pts = repeatedCenter + pts;

end % end getCirclePts

function pts = getPlusPts(center, v, d, step)

    % d (output of eig) is used as the span of the object, multiplying d
    % with (1,1) scales the vector.
    sc = d*[1;1];
    % Since the center is on origin and generated points are -x to x,
    % scale is divided by 2.
    sc = sc./2;
    % Width of the bars in X would be 10% of the smaller bar.
    barWidth = 0.1*min(sc);

    % Generate points to fill the box, where -sc(1) and sc(1) represent the
    % extremities in length for horizontal bar, and -sc(2) and sc(2) in
    % vertical bar.
    [hmgX, hmgY] = meshgrid(-sc(1):step:sc(1), -barWidth:step:barWidth);
    horizBarPts = [hmgX(:)';hmgY(:)']; % 2-by-M
    [vmgX, vmgY] = meshgrid(-barWidth:step:barWidth, -sc(2):step:sc(2));
    vertBarPts = [vmgX(:)';vmgY(:)']; % 2-by-M
    pts = [horizBarPts vertBarPts]; % 2-by-N

    % Rotate using the eigen vectors, which is used as the orientation of
    % the points.
    rotPts = (v * pts)'; % N-by-2 = (2-by-N)'
    % Translate points to the cluster center, which is used as the
    % obstacle position.
    repeatedCenter = repmat(center, size(rotPts, 1), 1);
    pts = rotPts + repeatedCenter; % N-by-2

end % end getPlusPts
