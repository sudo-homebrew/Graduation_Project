classdef (Hidden) MapInterface < handle

% This is an internal class and may be removed or modified in a future
% release.


%   Copyright 2020 The MathWorks, Inc.
    
%#codegen
    
    properties
        %Resolution Grid resolution in cells per meter
        Resolution
        
        %GridSize Size of the grid in [rows, cols] (number of cells)
        GridSize
    end
    
    properties (Dependent)
        %XLocalLimits Min and max values of X in local frame
        XLocalLimits
        
        %YLocalLimits Min and max values of Y in local frame
        YLocalLimits
    end
    
    properties
        %GridOriginInLocal Location of the grid in local coordinates
        %   A vector defining the [X Y] location of the bottom-left
        %   corner of the grid, relative to the local frame.
        %   Default: [0 0]
        GridOriginInLocal
    end
    
    properties (Hidden, Constant)
        %FPECorrFactor Tolerance for considering grid-based numbers 'whole'
        %   A scalar which defines floating-point error tolerance when 
        %   flooring/ceiling values that have been converted from xy to cells.
        %   A value of 2 is used to counteract error caused by summation of
        %   floating point numbers, larger tolerances can be used if other 
        %   operations may have been performed on the input
        FPECorrFactor = 2;
    end
    
    properties (Hidden)
        %Length length of the map along X direction
        Length
        
        %Width length of the map along Y direction
        Width
    end
    
    methods
        function set.GridOriginInLocal(obj, gridOrig)
            validateattributes(gridOrig, {'numeric'}, {'nonempty', 'real',...
                'nonnan', 'finite', 'vector', 'numel', 2}, 'EgoCentricMap','GridOriginInLocal');
            obj.GridOriginInLocal = gridOrig;
        end
        
        function val = get.XLocalLimits(obj)
            val = [obj.GridOriginInLocal(1),obj.GridOriginInLocal(1)+obj.Length];
        end
        
        function val = get.YLocalLimits(obj)
            val = [obj.GridOriginInLocal(2),obj.GridOriginInLocal(2)+obj.Width];
        end
    end
    
    methods (Access = protected)
        function localXY = grid2localImpl(obj, gridInd)
        %grid2localImpl Convert grid coordinates to local coordinates
            xlimit = [obj.GridOriginInLocal(1),obj.GridOriginInLocal(1)+obj.Length];
            ylimit = [obj.GridOriginInLocal(2),obj.GridOriginInLocal(2)+obj.Width];
            localXY = [xlimit(1) + (gridInd(:,2)-1)/obj.Resolution,...
                ylimit(1) + (obj.GridSize(1) - gridInd(:,1))/obj.Resolution] + (1/(2*obj.Resolution));
        end

        function gridInd = local2gridImpl(obj,localXY)
        %local2gridImpl Convert local coordinates to grid coordinates
            xlimit = [obj.GridOriginInLocal(1),obj.GridOriginInLocal(1)+obj.Length];
            ylimit = [obj.GridOriginInLocal(2),obj.GridOriginInLocal(2)+obj.Width];
            gridInd = ceil([-ylimit(1) + localXY(:,2),-xlimit(1) + localXY(:,1)]*obj.Resolution);
            
            % GridLocation is always cell [1 1]
            originIdx = abs(gridInd) < eps;
            if any(originIdx(:))
                gridInd(originIdx) = 1;
            end
            
            % Set grid index increase along -ve y direction
            gridInd(:,1) = obj.GridSize(1) + 1 - gridInd(:,1);
        end
        
        function [xCell, yCell, cellWidth] = calculateMapProperties(map)
            [i, j] = ind2sub(map.GridSize, (1:prod(map.GridSize))');
            p = grid2localImpl(map, [i j]);
            xCell = p(:,1);
            yCell = p(:,2);
            cellWidth = (1/map.Resolution);
        end
    end 
end