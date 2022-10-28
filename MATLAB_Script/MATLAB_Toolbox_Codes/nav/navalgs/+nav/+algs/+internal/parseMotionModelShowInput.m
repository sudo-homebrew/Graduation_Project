function inputparam = parseMotionModelShowInput(varargin)
%This class is for internal use only. It may be removed in the future.

%parseMotionModelShowInput parse the input parameters of show method. This
%   has been used by show() of dubinsPathSegment &
%   reedsSheppPathSegment.

% Copyright 2018-2020 The MathWorks, Inc.


    parser = inputParser;

    addParameter(parser, 'Parent', [], ...
                 @(x)robotics.internal.validation.validateAxesUIAxesHandle(x));

    % Add default position value
    defaultPositionValue = ["start", "goal"];
    addParameter(parser, 'Positions', defaultPositionValue);

    % Add default heading value
    defaultHeadingValue = ["start", "goal", "transitions"];
    addParameter(parser, 'Headings', defaultHeadingValue);

    %add default headingLength value
    defaultHeadingLengthValue=0;
    addParameter(parser,'HeadingLength',defaultHeadingLengthValue);

    parser.parse(varargin{:});

    % Parse the positions
    tempPositions = parser.Results.Positions;
    positions = strings(1,numel(tempPositions));

    % To disable positions at all pose input should be only empty cell array({}).
    if  numel(tempPositions) == 0
        validateattributes(tempPositions, {'cell', 'strings'}, {}, 'show', 'Positions');
    end

    % Validate positions
    for i=1:numel(tempPositions)
        positions(i) = validatestring(tempPositions{i}, {'start', 'goal'}, 'show', 'Positions');
    end

    % Parse the headings
    tempHeadings = parser.Results.Headings;
    headings = strings(1,numel(tempHeadings));

    % To disable headings at all pose it should be only empty cell array({}).
    if  numel(tempHeadings) == 0
        validateattributes(tempHeadings, {'cell', 'strings'}, {}, 'show', 'Headings');
    end

    % Validate headings
    for i=1:numel(tempHeadings)
        headings(i) = validatestring(tempHeadings{i}, {'start', 'goal', 'transitions'}, 'show', 'Headings');
    end

    tempHeadingLength=parser.Results.HeadingLength;
    validateattributes(tempHeadingLength,{'numeric'},{'>=',0},'show','HeadingLength')
    headingLength=tempHeadingLength;

    inputparam.axHandle     = parser.Results.Parent;
    inputparam.positions    = positions;
    inputparam.headings     = headings;
    inputparam.headingLength= headingLength;


end
