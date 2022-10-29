classdef UIElementFactory < handle
    %This class if for internal use only and may be removed in a future release
    
    %UIElementFactory Methods to construct common UI and matlab.ui.* components
    
    %   Copyright 2021 The MathWorks, Inc.
    
    methods (Static)
        function newLabel = createUILabel(parent, row, col, text)
            newLabel = uilabel(parent, 'Text', text);
            newLabel.Layout.Row = row;
            newLabel.Layout.Column = col;
        end
        
        function newLabel = createUIField(parent, row, col, text)
            newLabel = uieditfield(parent, 'Value', text);
            newLabel.Layout.Row = row;
            newLabel.Layout.Column = col;
        end
        
        function button = createButtonInGrid(parentGrid, row, col, varargin)
            button = uibutton(parentGrid, varargin{:});
            button.Layout.Row = row;
            button.Layout.Column = col;
        end
        
        function [view, panel] = createPanelView(container, viewHandle, varargin)
            %createPanelView Add a panel containing a view in its figure
            
            panelOptions = struct(varargin{:});
            panel = matlab.ui.internal.FigurePanel(panelOptions);
            add(container, panel);
            view = viewHandle(panel.Figure);
        end
    end
    
end