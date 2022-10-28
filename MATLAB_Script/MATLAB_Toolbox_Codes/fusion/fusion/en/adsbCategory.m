classdef adsbCategory< uint8
%adsbCategory Enumeration of Automatic Dependent Surveillance - Broadcast categories
%
%   adsbCategory implements the list of categories used by the
%   International Civil Aviation Organization and described in the ICAO
%   Document 9871.
%   The list of entries is shown below:
%
%     0:No Category Information
%     1:Light
%     2:Small
%     3:Large
%     4:High-Vortex Large
%     5:Heavy
%     6:High Performance
%     7:Rotorcraft
%     8:Glider / Sailplane
%     9:Lighter-than-air
%    10:Parachutist / Skydiver
%    11:Ultralight
%    12:Unmanned Aerial Vehicle
%    13:Space Vehicle
%    14:Surface Vehicle
%    15:Obstacle
%
%   See also: adsbReceiver, adsbTransponder

 
    %   Copyright 2020 The MathWorks, Inc.

    methods
        function out=adsbCategory
        end

    end
    enumeration
        Glider_Sailplane;

        Heavy;

        High_Performance;

        High_Vortex_Large;

        Large;

        Light;

        Lighter_than_air;

        No_Category_Information;

        Obstacle;

        Parachutist_Skydiver;

        Rotorcraft;

        Small;

        Space_Vehicle;

        Surface_Vehicle;

        Ultralight;

        Unmanned_Aerial_Vehicle;

    end
end
