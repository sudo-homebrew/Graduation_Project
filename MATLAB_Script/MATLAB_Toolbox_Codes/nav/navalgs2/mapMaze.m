function map = mapMaze(varargin)
%mapMaze Generate maze map with straight passage, turns, T-junctions
%   MAP = mapMaze generates a random 2-D maze map as a binaryOccupancyMap
%   object map, with a width and height of 50 meters and a resolution of 5
%   cells per meter. The maze map contains straight passage, turns,
%   and T-junctions. The wall width is 1 grid cell and the passage width
%   is 4 grid cells.
%
%   MAP = mapMaze(PASSAGEWIDTH) generates a binaryOccupancyMap of a maze of
%   the default size and resolution, with a specified passage width,
%   PASSAGEWIDTH, in number of grid cells.
%
%   MAP = mapMaze(PASSAGEWIDTH,WALLTHICKNESS) generates a
%   binaryOccupancyMap of a maze of the default size and resolution, with a
%   specified passage width, PASSAGEWIDTH, and wall thickness, WALLTHICKNESS
%   in number of grid cells.
%
%   MAP = mapMaze(___,Name,Value) specifies options using one or more
%   name-value pair arguments in addition to any combination of input
%   arguments from previous syntaxes.
%
%   Name-Value pairs for the function:
%
%       MapSize        - Width and height of generated map, specified as a   
%                        two-element vector of positive real finite numbers 
%                        of the form [width, height]. Specify both values in meter.
%                        Default: [50 50] 
%
%       MapResolution  - Resolution of generated map specified as a positive 
%                        real scalar in cells per meter.
%                        Default: 5
%
%
%   Example:
%       % Generate a binaryOccupancyMap containing a maze.
%       map = mapMaze(1,2,"MapSize",[50 30],"MapResolution",2);
%
%       % Visualize the generated maze map
%       show(map);
%
%   See also mapClutter, binaryOccupancyMap, validatorOccupancyMap.

%   Copyright 2020-2021 The MathWorks, Inc.

%#codegen

narginchk(0, 6);

% parse for all the allowed inputs and assign appropriate arg value to
% appropriate variable. Also check for Names without values. If a
% mistake is detected throw appropriate error.
[passageWidth, wallThickness, mapSize, mapRes] = parseInputArguments(varargin{:});

%Create binaryOccupancyMap
map = binaryOccupancyMap(mapSize(1), mapSize(2), mapRes);

%Take map grid size
numGridRows    = map.GridSize(1);
numGridColumns = map.GridSize(2);

%Number of cells needed to jump from one passage to another passage
skipStep = passageWidth + wallThickness;

NoWALL      = 0;
WALL        = 1;
NotVISITED  = -1;
VISITED     = -2;

%Throw an error if passageWidth+2*wallThickness > (numCellsRows or
%numCellsColumns). wallThickness is multiplied by 2 border of the maze.
if passageWidth+2*wallThickness > numGridColumns
    nav.algs.internal.error('nav:navalgs', 'mapMaze:MapWidthError');
end

if passageWidth+2*wallThickness > numGridRows
    nav.algs.internal.error('nav:navalgs', 'mapMaze:MapHeightError');
end

%Compute number of rows which make sure maze's passageWidth and wallThickness
%are not changing at any place also maze has always a border from left and 
%right side with at least size of wallThickness.
extraRowsBorderPadding = mod(numGridRows-wallThickness, skipStep);
numGridRows = numGridRows - extraRowsBorderPadding;

%Compute number of columns which make sure maze's passageWidth and 
%wallThickness are not changing at any place also maze has always a border 
%from top and bottom side with at least size of wallThickness.
extraColumnsBorderPadding = mod(numGridColumns-wallThickness, skipStep);
numGridColumns = numGridColumns - extraColumnsBorderPadding;

%Take a grid of size numGridRows X numGridColumns. Also, mark grid cells as
%NoVISITED.
grid = NotVISITED(ones(numGridRows, numGridColumns));

%Take left-top passage to start DFS
currentCell = sub2ind(size(grid),wallThickness+1,wallThickness+1);
coder.varsize('currentCell');
grid(currentCell) = VISITED;

offsetsIndices = zeros(4, passageWidth*wallThickness);

x = 1;
y = passageWidth;
for i=1:wallThickness
    %Fill walls vertical and horizontal with size of wallthickness at
    %distance of passage
    grid(i:skipStep:numGridRows,:) = WALL;
    grid(:,i:skipStep:numGridColumns) = WALL;
    
    offsetsIndices(:, x:y) = [...
        -1*i:numGridRows:numGridRows*(passageWidth-1);... %Indices to remove left side wall from the current cell 
        passageWidth+i-1:numGridRows:numGridRows*passageWidth;...%Indices to remove right side wall from the current cell 
        (passageWidth+i-1)*numGridRows:(numGridRows+1)*(passageWidth+i-1)-i; ... %Indices to remove bottom side wall from the current cell 
        -numGridRows*i:-numGridRows*i+passageWidth-1 ...%Indices to remove top side wall from the current cell 
        ];
    x = y+1;
    y = y + passageWidth;
end

%Initialize stack
S = robotics.core.internal.Stack(0,numGridRows*numGridColumns,1);
S.push(currentCell);

[validMovesIndices, validOffsetsIndices, validMovesValues] =...
    computeValidMovesIndicesValues([wallThickness+1, wallThickness+1], grid, offsetsIndices, skipStep);

while (~isempty(S))
    
    %Find valid not visited cells
    unvistedNeigbors = find(validMovesValues==NotVISITED);
    
    %If there are unvisited neighbors, randomly pick one and continue move, 
    %if there are not unvisited neighbor, pop a new location from the stack
    %and start over.
    if (~isempty(unvistedNeigbors))
        
        %Select random cell from all valid not visited cells
        next = unvistedNeigbors(randi(length(unvistedNeigbors),1));
        
        %Clear wall between currentCell and selected random cell using
        %meshgrid "validOffsetsIndices"
        grid(repmat(currentCell, 1, size(validOffsetsIndices,2)) + validOffsetsIndices(next,:)) = NoWALL;
        
        newCell = validMovesIndices(next);
        [movesX, movesY] = ind2sub(size(grid), newCell);
        
        %Find valid moves indices, valid offset indices and valid moves grid values for newCell 
        [validMovesIndices, validOffsetsIndices, validMovesValues] =...
            computeValidMovesIndicesValues([movesX, movesY], grid, offsetsIndices, skipStep);
        
        %push newCell into stack if any valid move is NotVISITED
        if (any(validMovesValues==NotVISITED))
            S.push(newCell);
        end
        
        currentCell = newCell;
        grid(currentCell) = VISITED;
    else
        currentCell = S.pop();
        [movesX, movesY] = ind2sub(size(grid), currentCell);
        
        %Find valid moves indices, valid offset indices and valid moves
        %grid values for currentCell (stack pop value)
        [validMovesIndices, validOffsetsIndices, validMovesValues] =...
            computeValidMovesIndicesValues([movesX, movesY], grid, offsetsIndices, skipStep);
        
    end
end

grid(grid==-2) = 0;
grid(grid==-1) = 0;

%Fill extra border padding with 1
grid = [ones(ceil(extraRowsBorderPadding/2),numGridColumns); grid; ones(floor(extraRowsBorderPadding/2),numGridColumns)];
grid = [ones(numGridRows+extraRowsBorderPadding,ceil(extraColumnsBorderPadding/2))...
    grid ones(numGridRows+extraRowsBorderPadding,floor(extraColumnsBorderPadding/2))];

%Set occupancy values with grid values
[I,J] = meshgrid(1:size(grid,1), 1:size(grid,2));
grid = grid';
values = grid(:);
setOccupancy(map, [I(:), J(:)], values, 'grid');
end

function [validMovesIndices, validOffsetsIndices, validMovesValues] = ...
    computeValidMovesIndicesValues(currentCell, grid, offsetsIndices, skipStep)
%computeValidMovesIndicesValues Find valid moves' indices, valid offset's
%   indices and valid moves' values for given currentCell cell, in given
%   grid.

% Un-const grid dimensions to keep validMovesIndices upper-bound varsize
sz = coder.ignoreConst(size(grid));

offsets = [...
    -skipStep 0; ...%Move left side to the current cell by passageWidth and wallThickness
    skipStep 0; ...%Move right side to the current cell by passageWidth and wallThickness
    0 skipStep; ...%Move bottom side to the current cell by passageWidth and wallThickness
    0 -skipStep ....%Move top side to the current cell by passageWidth and wallThickness
    ];

%Compute all four directions(left, right, bottom, top) subscripts which are 
%moveable from current cell
moves = repmat(currentCell,4,1) + offsets;

%Find valid I, J(which are less than are equal to grid size).
I = moves(:,1) > 0 & moves(:,1)<=sz(1);
J = moves(:,2) > 0 & moves(:,2)<=sz(2);

%valid Moves
validMoves = moves(I&J,:);

%valid offsetsIndices
validOffsetsIndices = offsetsIndices(I&J,:);

%Convert validMoves subscripts to indices
validMovesIndices = validMoves(:,1) + (validMoves(:,2) - 1)*size(grid,1);

%Extract grid values at valid moves
validMovesValues = grid(validMovesIndices);
end

function [passageWidth, wallThickness, mapSize, mapRes] = parseInputArguments(varargin)
    %parseValidateArguments estimates which optional args are provided by
    %the user and calls functions to parse and validate respective
    %functions.

    % 0. Setup the default/frequently used values.
    % All the inputs to mapMaze would come through varargin, so it's
    % size should be the same as nargin for mapMaze.
    nargs = numel(varargin);

    numMaxOptionalArgs = 2;
    mapSize = [50 50];
    mapRes = 5;
    validnames = {'mapsize', 'mapresolution'}; % valid names
    defaults = {mapSize, mapRes}; % Values

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
    % Arg order: passage width, wall thickness, step size, extra args.
    argIdx = 1:3;
    % It could be parsed if the argIdx is less than nargin.
    narginCouldParseArg = argIdx <= nargs;
    % It should be parsed if the argIdx is less than nargin.
    nameValueCouldParseArg = argIdx < nameValueIdx;
    % If both say arg could be parsed, then it should be parsed.
    shouldParseArg = and(narginCouldParseArg, nameValueCouldParseArg);

    % 3. Parse Passage Width if it should be.
    if shouldParseArg(1)        
        passageWidth = varargin{1};
        robotics.internal.validation.validatePositiveIntegerScalar(...
            passageWidth, 'mapMaze', 'passageWidth');
    else
        % Assign a default value if it's not passed by the user.
        passageWidth = 4;
    end
    passageWidth = double(passageWidth);

    % 4. Parse Wall Thickness if it should be.
    if shouldParseArg(2)
        wallThickness = ...
            double(robotics.internal.validation.validatePositiveIntegerScalar(...
            varargin{2}, 'mapMaze', 'WallThickness'));
    else
        % Assign default value if it's not passed by the user.
        wallThickness = 1;
    end

    % 5. Let the Name Value parser parse whatever in the args hasn't been
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
    %    for e.g. ("Map..",...), (10, "Map..",..), (10, 2, "Map..")
    % 2. All optional args are passed, but illegal args are passed after that.
    %    for e.g. (10, 2, 42)
    if nameValueIdx ~= inf
        nvArgs = {varargin{nameValueIdx:end}};
        [mapSize,mapRes] = parseValidateNameValuePairs(validnames, defaults,...
                                                     nvArgs{:});
        mapSize = double(mapSize);
        mapRes = double(mapRes);
    end
end

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
end

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
end

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

end