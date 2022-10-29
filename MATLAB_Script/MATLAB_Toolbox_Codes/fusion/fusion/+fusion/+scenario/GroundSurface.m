classdef GroundSurface < radarfusion.internal.scenario.StaticSurfaceModel
    %fusion.scenario.GroundSurface create surface object for trackingScenario
    %
    %   fusion.scenario.GroundSurface contains properties and methods that
    %   define static height data over some domain of the scenario world.
    %
    % GroundSurface properties:
    %
    % Terrain         - terrain data associated with the surface
    % ReferenceHeight - the origin height value of the surface
    % Boundary        - the rectangular boundary of the surface
    %
    % GroundSurface methods:
    % 
    % height    - Return the height of the surface at a point
    % occlusion - Return the surface occlusion state between two points
    %
    %   See also trackingScenario/groundSurface.

    %   Copyright 2021 The MathWorks, Inc.
    
    methods(Access = ?fusion.scenario.SurfaceManager)
       
        function obj = GroundSurface( man,varargin )
            obj@radarfusion.internal.scenario.StaticSurfaceModel(man,varargin{:});

            % Parse inputs
            obj.Parser.addOptional('Terrain',[]);
            obj.Parser.parse(varargin{:});
            obj.ReferenceHeight = obj.Parser.Results.ReferenceHeight;
            obj.Boundary = obj.Parser.Results.Boundary;
            obj.parseTerrainSpec;
            
        end
        
    end
    
    methods
        
        function h = height(obj,varargin)
            %height   Return the height of the surface at a point
            %   H = height(srf,P) returns the height in meters of
            %   the surface, srf, at a given point. P is a 2-by-N or 3-by-N
            %   matrix of coordinate points in the coordinate system of the
            %   scenario. H is a length-N vector of height values in
            %   meters.
            %
            %   When the IsEarthCentered property of the scenario is true, 
            %   the coordinate system is geodetic. Otherwise, the
            %   coordinate system is Cartesian. 
            %
            %   % Example:
            %   % Load a height map and use it to determine the heights of
            %   % 2 points on the map. 
            %
            %   % Create a tracking scenario
            %   scene = trackingScenario('IsEarthCentered',false)
            %
            %   % Load height map
            %   load('heightMap.mat')
            %   bnds = [-50 50; -50 50];
            %   
            %   % Create a ground surface
            %   srf = groundSurface(scene,'Terrain',hmap,'Boundary',bnds);
            % 
            %   % Determine the height at points 1 and 2
            %   P1 = [-24.0157; -15.3543]; % Point 1
            %   P2 = [27.9528; -41.3386];  % Point 2
            %   H = height(srf,[P1 P2])    % Heights in meters 
            %
            %   See also trackingScenario, trackingScenario/groundSurface. 
        
            h = height@radarfusion.internal.scenario.StaticSurfaceModel(obj,varargin{:});
        end
        
        function occ = occlusion(obj,varargin)
            %occlusion   Determine if two points are occluded
            %   TF = occlusion(srf,P1,P2) returns a scalar logical
            %   indicating if the line of sight between two points P1 and
            %   P2 is occluded. P1 and P2 are length-3 vectors in the same
            %   units as the trackingScenario coordinate system.
            %
            %   When the IsEarthCentered property of trackingScenario is
            %   true, the coordinate system is geodetic. Otherwise, the
            %   coordinate system is Cartesian.
            %
            %   % Example:
            %   % Load a height map and use it to determine if the path 
            %   % from point 1 to points 2 and 3 are occluded.
            %
            %   % Create a radar scenario
            %   scene = trackingScenario('IsEarthCentered',false)
            %
            %   % Load height map
            %   load('heightMap.mat')
            %   bnds = [-50 50; -50 50];
            %   
            %   % Create a land surface
            %   srf = groundSurface(scene,'Terrain',hmap,'Boundary',bnds);
            % 
            %   % Set points for testing
            %   P1 = [-24.0157; -15.3543; 8]; % Point 1
            %   P2 = [27.9528; -41.3386; 6];  % Point 2
            %   P3 = [21.6535; 21.6535; 10];  % Point 3
            %
            %   % Plot terrain and test points
            %   figure()
            %   x = linspace(bnds(1,1),bnds(1,2),128);
            %   y = linspace(bnds(2,1),bnds(2,2),128);
            %   s = surf(x,y,hmap);
            %   s.EdgeColor = [0.3 0.3 0.3];
            %   hold on
            %   plot3(P1(1),P1(2),P1(3),'o','LineWidth',3,'MarkerSize',6)
            %   plot3(P2(1),P2(2),P2(3),'o','LineWidth',3,'MarkerSize',6)
            %   plot3(P3(1),P3(2),P3(3),'o','LineWidth',3,'MarkerSize',6)
            %   xlabel('X (m)')
            %   ylabel('Y (m)')
            %   title('Height Map')
            %   colorbar
            %   view([40 10])
            %   legend('Terrain','P1','P2','P3','Location','South')
            %
            %   % Determine if the path from P1 to P2 is occluded
            %   tf1 = occlusion(srf,P1,P2)
            %
            %   % Determine if the path from P1 to P3 is occluded
            %   tf2 = occlusion(srf,P1,P3)
            %
            %   See also trackingScenario, trackingScenario/groundSurface.
            
            occ = occlusion@radarfusion.internal.scenario.StaticSurfaceModel(obj,varargin{:});
        end
        
    end
    
end