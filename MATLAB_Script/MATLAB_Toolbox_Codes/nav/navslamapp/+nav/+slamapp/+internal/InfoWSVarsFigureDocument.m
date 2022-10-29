classdef InfoWSVarsFigureDocument < nav.slamapp.internal.FigureDocument
%This class is for internal use only. It may be removed in the future.

%INFOWSVARSFIGUREDOCUMENT Figure document for display of workspace variable
%   information

% Copyright 2020-2021 The MathWorks, Inc.


    properties
        %ScansVarNameLabel
        ScansVarNameLabel

        %ScansVarContent
        ScansVarContent

        %PosesVarNameLabel
        PosesVarNameLabel

        %PosesVarContent
        PosesVarContent

        %PercentageLabel
        PercentageLabel

        %PercentageContent
        PercentageContent

    end

    methods
        function obj = InfoWSVarsFigureDocument(tag)
        %InfoWSVarsFigureDocument Constructor
            obj@nav.slamapp.internal.FigureDocument(tag);

            obj.ScansVarNameLabel = obj.addLabel('ScansVarNameLabelName');
            obj.ScansVarContent = obj.addTextField();

            obj.PosesVarNameLabel = obj.addLabel('PosesVarNameLabelName');
            obj.PosesVarContent = obj.addTextField();

            obj.PercentageLabel = obj.addLabel('PercentageLabelName');
            obj.PercentageContent = obj.addTextField();

            addlistener(obj.Figure, 'SizeChanged', @(src, evt) obj.repositionUIControlsInFigure);
        end

        function repositionUIControlsInFigure(obj)
        %repositionUIControlsInFigure
            figPos = obj.Figure.Position;
            indentWidth = 35;
            obj.ScansVarNameLabel.Position(2) = figPos(4) - 50;
            obj.ScansVarContent.Position(2) = figPos(4) - 75;
            obj.ScansVarContent.Position(1) = indentWidth;

            obj.PosesVarNameLabel.Position(2) = figPos(4) - 100;
            obj.PosesVarContent.Position(2) = figPos(4) - 125;
            obj.PosesVarContent.Position(1) = indentWidth;

            obj.PercentageLabel.Position(2) = figPos(4) - 150;
            obj.PercentageContent.Position(2) = figPos(4) - 175;
            obj.PercentageContent.Position(1) = indentWidth;
        end

        function h = addLabel(obj, msgID)
        %addLabel
             h = uilabel(obj.Figure, 'Position', [15 2 800 20], ...
                          'HorizontalAlignment', 'left', ...
                          'FontSize', 10, ...
                          'Text', obj.retrieveMsg(msgID), 'FontWeight', 'bold');
        end

        function h = addTextField(obj)
        %addTextField
            h = uilabel(obj.Figure, 'Position', [15 2 800 20], ...
                          'HorizontalAlignment', 'left', ...
                          'FontSize', 10, 'FontName', 'Consolas', ...
                          'Text', '');
        end
        
        function refreshTextFields(obj, infoStrings)
            %refreshTextFields
            
            obj.ScansVarContent.Text = infoStrings{1};
            obj.PosesVarContent.Text = infoStrings{2};
            obj.PercentageContent.Text = infoStrings{3};
        end

    end
end
