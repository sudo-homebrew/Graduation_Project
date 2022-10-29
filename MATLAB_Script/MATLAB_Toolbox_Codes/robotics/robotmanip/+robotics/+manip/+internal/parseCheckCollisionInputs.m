function [worldObjects, isExhaustive, ignoreSelfCollision] = parseCheckCollisionInputs(varargin)
%parseCheckCollisionInputs Parse the optional inputs for the rigidBodyTree/checkCollision
%   The function parses the input arguments for the optional worldObjects,
%   and the 'Exhaustive' and 'IgnoreSelfCollision', flag

%   Copyright 2020 The MathWorks, Inc.

%#codegen
    names = {'Exhaustive', 'IgnoreSelfCollision'};
    defaults = {'off', 'off'};

    isExhaustive = defaults{1};
    ignoreSelfCollision = defaults{2};

    parser = robotics.core.internal.NameValueParser(names, defaults);

    if(nargin < 1) %When there are no world objects defined
        worldObjects = {};
    else
        %If there is more than one input argument, and if the first one is a
        %char or string, then the user wants to specify just the flags. If the
        %first argument is a garbage string or char, the error is thrown by the
        %parser itself.
        if(ischar(varargin{1}) || isstring(varargin{1}))
            parser.parse(varargin{:});
            isExhaustive = parser.parameterValue('Exhaustive');
            ignoreSelfCollision = parser.parameterValue('IgnoreSelfCollision');
            worldObjects = {};
        else
            %If the first argument is not a string or char, then we assume that the
            %first argument is denoting the world objects.
            worldObjects = varargin{1};

            %skip the first argument as it is the world objects
            parser.parse(varargin{2:end});
            isExhaustive = parser.parameterValue('Exhaustive');
            ignoreSelfCollision = parser.parameterValue('IgnoreSelfCollision');
        end
    end
end
