classdef Tab < handle
%   This class is for internal use and may be removed in a future release

%TAB Superclass for tab objects in the toolstrip view

%   Copyright 2021-2022 The MathWorks, Inc.

    properties (SetAccess = immutable)
        % The properties here are defined once in the constructor and then
        % never changed, so their access is limited to enforce mutability

        %Parent Object that stores the tab
        Parent

        %TabHandle Handle to the <matlab ui>.Tab object
        TabHandle

        %TabGroup Handle to the toolstrip's tabgroup with which the tab may be associated
        TabGroup
    end

    methods (Sealed)
        function obj = Tab(toolstrip, tabName, tabTag)
        %Tab Constructor

            obj.TabHandle = matlab.ui.internal.toolstrip.Tab(tabName);
            obj.TabHandle.Tag = tabTag;
            obj.Parent = toolstrip;
            obj.TabGroup = toolstrip.TabGroup;

        end
    end

    methods
        function show(obj)
        %show Make the tab visible in the toolstrip

            obj.TabGroup.add(obj.TabHandle);
        end

        function hide(obj)
        %hide Remove the tab from those visible in the toolstrip

        % If the tab group does not contain this tab object, don't
        % attempt to remove it. It's already hidden from view, and
        % trying to remove it will result in an error.
            if ~isempty(obj.TabGroup.contains(obj.TabHandle.Tag))
                obj.TabGroup.remove(obj.TabHandle);
            end
        end
    end

    methods (Sealed)
        function enableAll(obj)
        %enableAll Enable all ui components in the tab

            obj.TabHandle.enableAll();
        end

        function disableAll(obj)
        %disableAll Disable all ui components in the tab

            obj.TabHandle.disableAll();
        end

        function select(obj)
        %select Bring the tab into focus in the toolstrip

            obj.TabGroup.SelectedTab = obj.TabHandle;
        end

        function sendNotification(obj, eventName, eventData)
        %sendNotification Send notification
        %   This method tells the parent to send the notification. This
        %   ensures that the notifications are only ever sent by the
        %   main toolstrip object.

            obj.Parent.sendNotification(eventName, eventData);
        end

        function updateToolstripMode(obj, toolstripModeEnum)
        %updateToolstripMode Update the toolstrip mode
        %   This method tells the parent to set the toolstrip mode.
        %   This ensures that mode switches are only ever made by the
        %   top-level toolstrip.

            obj.Parent.updateToolstripMode(toolstripModeEnum);
        end

        function updateToolstripTabDisplay(obj, tabTagsToHide, tabTagsToShow, tabTagToSelect, isVisibleTabAConstraint)
        %updateToolstripTabDisplay Update the displayed tabs in the toolstrip
        %   This method tells the parent to set the visibility of tabs
        %   in the toolstrip, which ensures that this visibility is
        %   governed at the top-level toolstrip view. This method takes
        %   three inputs: the cell array of tab tags to be hidden, the
        %   cell array of tab tags to be shown, and if specified, the
        %   tab tag (a string) to be selected. The last value is a
        %   boolean indicating whether the resultant visible tab is a
        %   constraint tab or not.

            obj.Parent.updateToolstripTabDisplay(tabTagsToHide, tabTagsToShow, tabTagToSelect, isVisibleTabAConstraint);
        end
    end

    methods (Access = protected)
        function uialert(obj, errMsg)
            evt = robotics.ikdesigner.internal.event.MessageEvent(errMsg);
            obj.sendNotification("RequestUIAlert", evt);
        end

        function [isValid, numericValue] = validateAndSetNumericInput(obj, src, evt, fieldObj, fieldName, fieldAttributes)
            %validateAndSetNumericInput Validate inputs for a numeric toolstrip edit field
            %   This method takes in the source & event data from an edit
            %   action and then verifies that the relevant attributes are
            %   met. If they are, the field is updated; it not, the
            %   validateattributes error is passed to the error dialog.

            try
                numericValue = str2num(src.Text); %#ok<ST2NM> 
                if isempty(numericValue) || any(isnan(numericValue), 'all')
                    % When the input cannot be converted to numeric data,
                    % it returns empty or sometimes NaN
                    error(message('robotics:ikdesigner:toolstrip:ExpectedNumericInputs'));
                end
                validateattributes(numericValue, {'double'}, ...
                                   fieldAttributes, ...
                                   'ikdesigner',fieldName);
                fieldObj.Value = src.Text;
                isValid = true;
            catch ME
                obj.uialert(ME.message);
                fieldObj.Value = evt.EventData.OldValue;
                isValid = false;
            end
        end
    end

    methods (Static, Access = protected)
        function initializeBodyNameDropdown(dropdown, robot, rigidBodyKeyMap, startIdx)
        %initializeBodyNameDropdown Initialize a dropdown of body names associated with a rigid body tree object
        %   There are several instances of dropdowns that refer to a
        %   list of the bodies in the rigid body tree and require the
        %   user to select one. This method initializes that list. The
        %   list displays the names of the bodies, but the values
        %   corresponding with each selection are the keys in the scene
        %   objects map that are associated with that rigid body.
        %   Additionally, the start index assigns the body that will be
        %   selected by default. Typically this value is either 1
        %   (likely the base) or the total number of bodies (which is
        %   likely the end effector). The index associations are not
        %   guaranteed though, as every rigid body tree is different.

        % Create the list of bodies (labels) and associate it with the
        % corresponding list of rigid body keys (values)
            bodyNameArray = [robot.BaseName robot.BodyNames]';
            bodyKeyArray = cell(size(bodyNameArray));
            for i = 1:numel(bodyNameArray)
                bodyKeyArray{i} = rigidBodyKeyMap(bodyNameArray{i});
            end

            % Assign the list to the dropdown and set the initial value
            dropdown.replaceAllItems([bodyKeyArray bodyNameArray]);
            dropdown.Value = bodyKeyArray{startIdx};
        end
    end

    methods (Static, Access = protected)

        function populatePopupList(button, popupListButtonsCellArray)
        %populatePopupList Populate a pop-up list of all buttons using a cell array of inputs
        %   This helper method accepts a cell array where each input
        %   defines a button that will appear in a pop-up list. The
        %   format of the input cell array is {name icon tag callback}.
        %   This method does not cover non-button list items, which must be
        %   added separately.

            for i = 1:numel(popupListButtonsCellArray)
                listButtonData = popupListButtonsCellArray{i};
                listItemName = listButtonData{1};
                listItemIcon = listButtonData{2};
                listItemTag = listButtonData{3};
                listItemCallback = listButtonData{4};
                listItemTooltip = listButtonData{5};

                listButtonItem = matlab.ui.internal.toolstrip.ListItem(listItemName, listItemIcon);
                listButtonItem.Tag = listItemTag;
                listButtonItem.ItemPushedFcn = listItemCallback;
                listButtonItem.Description = listItemTooltip;
                button.Popup.add(listButtonItem);
            end

        end

        function addColumnPlaceholderLabel(col, objTag)
        %addColumnPlaceholderLabel Add an empty label to an existing column

            label = matlab.ui.internal.toolstrip.Label(" ");
            label.Tag = objTag;
            label.Description = " ";
            col.add(label);

        end
    end
end
