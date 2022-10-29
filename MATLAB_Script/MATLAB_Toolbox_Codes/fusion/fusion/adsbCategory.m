classdef adsbCategory < uint8
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
    
    % Reference:
    % ICAO Doc 9871, Technical Provisions for Mode S and Extended Squitter,
    % APPENDIX C, p. C-65

    %   Copyright 2020 The MathWorks, Inc.
    
    enumeration
        No_Category_Information (0)
        Light                   (1)
        Small                   (2)
        Large                   (3)
        High_Vortex_Large       (4)
        Heavy                   (5)
        High_Performance        (6)
        Rotorcraft              (7)
        Glider_Sailplane        (8)
        Lighter_than_air        (9)
        Parachutist_Skydiver    (10)
        Ultralight              (11)
        Unmanned_Aerial_Vehicle (12)
        Space_Vehicle           (13)
        Surface_Vehicle         (14)
        Obstacle                (15)
    end
    
end
