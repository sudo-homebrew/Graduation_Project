function plat = generatePlatform(scenario, platID, varargin)

%   This file is for internal use only.  It may be removed in a future
%   release.
%

%   Copyright 2020 The MathWorks, Inc.


plat = fusion.scenario.Platform(scenario, platID, varargin{:});
