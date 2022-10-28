classdef CustomizedLayouts < handle
%CUSTOMIZEDLAYOUTS Mixin class that provides utility to create
%   customized layouts for working area in SLAM app

% Copyright 2018-2021 The MathWorks, Inc.

    properties (Constant, Access = private)
        FigureDocumentTags = ["MapFigure_document1","IncrementalFigure_document3","OdomFigure_document7",...
            "InfoFigure_document4","InfoWSVarsFigure_document5","ScanFigure_document6","LoopClosureFigure_document2"];
    end

    methods


        function layout = createLayout3F(obj,style)
        %createLayout Generate layouts for 3 figures
        
            assert(ismember(style, {'main', 'main1'}));
            
            % App container document area is considered as a set of 4
            % tiles. Tile 1 occupy all of the first column. Tile 2 and Tile
            % 3 occupy row1 column 2, row 2 column 2 respectively.
            gridDimensions = [2,2];
            tileCount = 3;
            
            if strcmp(style, 'main')
                columnWeights = [0.6668 0.3332];
            else
                columnWeights = [0.6188, 0.3812];
            end
            rowWeights = [1/2, 1/2];
            tileCoverage = [1,2;1,3];
            
            layout = createLayoutStruct(obj, gridDimensions, tileCount, ...
                columnWeights, rowWeights, tileCoverage);
            
            % Define which documents appear in which tile
            % Id of a document is defined as <GroupTag>_<DocumentTag>
            documentState = repmat(struct,7,1);
            for k = 1:length(obj.FigureDocumentTags)
                documentState(k).id = char(obj.FigureDocumentTags(k));
            end
            
            tile1.children = documentState([1,6]);
            tile2.children = documentState([2,4,5]);
            tile3.children = documentState([7,3]);
            
            layout.documentLayout.tileOccupancy = [tile1,tile2,tile3];
        end

        function layout = createLayout1F(obj, style)
        %createLayout1F
            assert(ismember(style, {'loop', 'incremental'}));
            % only one document will be visible in 1 tile. all others will
            % be invisible
            gridDimensions = [1,1];
            tileCount = 1;
            columnWeights = 1;
            rowWeights = 1;
            tileCoverage = 1;
            
            layout = createLayoutStruct(obj, gridDimensions, tileCount, ...
                columnWeights, rowWeights, tileCoverage);
            
            % Define which documents appear in which tile
            % Id of a document is defined as <GroupTag>_<DocumentTag>
            documentState = repmat(struct,7,1);
            for k = 1:length(obj.FigureDocumentTags)
                documentState(k).id = char(obj.FigureDocumentTags(k));
            end
            
            tile1.children = documentState;
            
            % tile occupancy specifies the tile/grid of each figure
            % document and also specifies width and hight of each tile. 
            layout.documentLayout.tileOccupancy = tile1;
        end
        
        function layout = createLayoutStruct(~, gridDimensions, tileCount, ...
                columnWeights, rowWeights, tileCoverage)
            %createLayoutStruct creates common app container figure
            %   document layout for slam map builder app
            
            documentLayout = struct;
            
            documentLayout.gridDimensions.w = gridDimensions(1);
            documentLayout.gridDimensions.h = gridDimensions(2);
            
            % total number of tiles in the layout
            documentLayout.tileCount = tileCount;
            
            
            % Set colum and row weights
            documentLayout.columnWeights = columnWeights;
            documentLayout.rowWeights = rowWeights;
            
            % specify which tile parent for each grid in the layout. For
            % example tile coverage of [1,1;2,3] means that grids (1,1) and
            % (1,2) together represent tile 1, grid (2,represents tile 2
            % and grid (2,2) represents tile 3.
            documentLayout.tileCoverage = tileCoverage;
            
            documentLayout.tileOccupancy = [];
            layout = struct;
            layout.documentLayout = documentLayout;
            layout.panelLayout = struct;
            layout.majorVersion = 2;
            layout.minorVersion = 1;
        end
    end
end
