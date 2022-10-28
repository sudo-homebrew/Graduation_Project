classdef InfoFigureDocument < nav.slamapp.internal.FigureDocument

%This class is for internal use only. It may be removed in the future.

%INFOFIGUREDOCUMENT Figure document for display of sensor data info

% Copyright 2018-2021 The MathWorks, Inc.


    properties
        %BagNameLabel
        BagNameLabel

        %BagNameContent
        BagNameContent

        %StartTimeLabel
        StartTimeLabel

        %StartTimeContent
        StartTimeContent

        %EndTimeLabel
        EndTimeLabel

        %EndTimeContent
        EndTimeContent

        %AvailableTopicsLabel
        AvailableTopicsLabel

        %AvailableTopicsContent
        AvailableTopicsContent

    end

    methods
        function obj = InfoFigureDocument(tag)
        %InfoFigureDocument Constructor
            obj@nav.slamapp.internal.FigureDocument(tag);

            obj.BagNameLabel = obj.addLabel('BagNameLabelName');
            obj.BagNameContent = obj.addTextField();

            obj.StartTimeLabel = obj.addLabel('StartTimeLabelName');
            obj.StartTimeContent = obj.addTextField();

            obj.EndTimeLabel = obj.addLabel('EndTimeLabelName');
            obj.EndTimeContent = obj.addTextField();

            obj.AvailableTopicsLabel = obj.addLabel('AvailableTopicsLabelName');
            obj.AvailableTopicsContent = obj.addTextField();
            
            addlistener(obj.Figure, 'SizeChanged', @(src, evt) obj.repositionUIControlsInFigure);
        end

        function repositionUIControlsInFigure(obj)
        %repositionUIControlsInFigure
            figPos = obj.Figure.Position;
            indentWidth = 35;
            obj.BagNameLabel.Position(2) = figPos(4) - 50;
            obj.BagNameContent.Position(2) = figPos(4) - 75;
            obj.BagNameContent.Position(1) = indentWidth;

            obj.StartTimeLabel.Position(2) = figPos(4) - 100;
            obj.StartTimeContent.Position(2) = figPos(4) - 125;
            obj.StartTimeContent.Position(1) = indentWidth;

            obj.EndTimeLabel.Position(2) = figPos(4) - 150;
            obj.EndTimeContent.Position(2) = figPos(4) - 175;
            obj.EndTimeContent.Position(1) = indentWidth;

            numTopics = length(obj.AvailableTopicsContent.Text);
            obj.AvailableTopicsLabel.Position(2) = figPos(4) - 200;
            obj.AvailableTopicsContent.Position(2) = figPos(4) - 200 - 25*numTopics;
            obj.AvailableTopicsContent.Position(1) = indentWidth;
            obj.AvailableTopicsContent.Position(4) = 25*numTopics;
        end

        function h = addLabel(obj, msgID)
        %addLabel
             h = uilabel(obj.Figure,  'Position', [15 2 800 20], ...
                          'HorizontalAlignment', 'left', ...
                          'FontSize', 10, ...
                          'Text', obj.retrieveMsg(msgID), 'FontWeight', 'bold');
        end

        function h = addTextField(obj)
        %addTextField
            h = uilabel(obj.Figure, 'Position', [15 2 800 20], ...
                          'HorizontalAlignment', 'left', ...
                          'FontSize', 10, 'Text', '');          
        end
        
        function refreshTextFields(obj, infoStrings)
            %refreshTextFields
            
            obj.BagNameContent.Text = infoStrings{1};
            obj.StartTimeContent.Text = infoStrings{2};
            obj.EndTimeContent.Text = infoStrings{3};
            obj.AvailableTopicsContent.Text = infoStrings(4:end);
        end

    end
end
