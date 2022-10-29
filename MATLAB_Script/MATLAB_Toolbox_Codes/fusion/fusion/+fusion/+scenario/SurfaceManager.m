classdef SurfaceManager < radarfusion.internal.scenario.SurfaceManager
    % This object contains properties and methods for interacting with a
    % trackingScenario surface.
    %
    % SurfaceManager properties:
    %
    % UseOcclusion - specify whether or not to use line-of-sight occlusion
    %
    % SurfaceManager methods:
    % 
    % height    - Return the height of the scenario surface at a point
    % occlusion - Return the surface occlusion state between two points
    % 
    %   See also trackingScenario/groundSurface.

    %   Copyright 2021 The MathWorks, Inc.

    methods(Access = ?trackingScenario)

        function obj = SurfaceManager(varargin)
            obj@radarfusion.internal.scenario.SurfaceManager(varargin{:});
        end

        function s = groundSurface(obj,varargin)
            s = fusion.scenario.GroundSurface(obj,varargin{:});            
            obj.addSurface(s);
        end

    end

    methods
        
        function h = height( obj,varargin )
            % [ H ] = height( OBJ,POS )
            %   Return the surface height at the given position or
            %   positions.
            %
            %   POS is a 3-by-N matrix where each column is a coordinate
            %   vector in scenario coordinates.

            h = height@radarfusion.internal.scenario.SurfaceManager(obj,varargin{:});
        end
        
        function [ occ, state ] = occlusion( obj,varargin )
            % [ OCC, STATE ] = occlusion( OBJ,POS1,POS2 )
            %   Determine if the line of sight between two points is
            %   occluded by the scenario surface. POS1 and POS2 are points
            %   given in scenario coordinates.
            %
            %   OCC is a logical scalar. If true, the line of sight is
            %   occluded by terrain or the horizon, if false the line of
            %   sight is unoccluded. If either input points contains NaN
            %   values, OCC will be NaN. This will occur if the input
            %   points came from the position of a scenario platform that
            %   is not currently in the scene.
            %
            %   STATE is an enumeration that describes the occlusion state,
            %   and will be one of the following:
            %
            %   Invalid:         One or both of the input points contained
            %                    a NaN.
            %   Unoccluded:      The line of sight between points is
            %                    unobstructed.
            %   TerrainOccluded: The line of sight is obstructed by
            %                    terrain.
            %   HorizonOccluded: The line of sight is obstructed by the
            %                    Earth's horizon (only relevant when
            %                    IsEarthCentered is true)

            [occ,state] = occlusion@radarfusion.internal.scenario.SurfaceManager(obj,varargin{:});
        end
        
        function s = clone(obj)
            s = obj.pClone(fusion.scenario.SurfaceManager(obj));
        end
        
    end

    methods(Static, Access=protected)

        function validateSurfaces( S )
            % In trackingScenario, surfaces are not allowed to overalap

            % Validate the last two elements of S.
            if numel(S) > 1 && S(end-1).overlaps(S(end))
                error(message('shared_radarfusion:surfaces:OverlappingSurfaces',numel(S)-1,numel(S)))
            end

        end

    end

end