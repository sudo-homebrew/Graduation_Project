classdef SolverController < robotics.manip.internal.InternalAccess
%This class is for internal use only and may be removed in a future release

%SolverController Controller that listens to SolverModel
%   The SolverController has listeners on the solver model and views
%   that can trigger to solver model. It facilitates communication
%   regarding the solver actions and solutions, and contains the solver
%   state. This differs from the scene model, which pertains to
%   physical objects in the scene and the absolute state of those
%   objects.

%   Copyright 2021 The MathWorks, Inc.

    properties (Access = ?matlab.unittest.TestCase)

        %SceneModel Model containing scene contents and state data
        SceneModel

        %SolverModel Model containing solver and solution data
        SolverModel

        %ConfigModel Model containing stored configurations and associated state data
        ConfigModel

        %SceneCanvasView Handle to the scene canvas view
        SceneCanvasView

        %ToolstripView Handle to the toolstrip view
        ToolstripView

        %ConstraintsBrowserView Handle to the constraints browser view
        ConstraintsBrowserView
    end

    methods
        function obj = SolverController(models, views)
        %SolverController Constructor

        % The solver controller only has model event listeners on the
        % solver model
            obj.SolverModel = models.SolverModel;

            % The solver controller also directs state updates on the scene
            % model
            obj.SceneModel = models.SceneModel;

            % Since the solver controller affects solver & solution state,
            % it is necessary to update the configurations model at times
            obj.ConfigModel = models.ConfigModel;

            % Views that have listeners
            obj.SceneCanvasView = views.SceneCanvasView;
            obj.ToolstripView = views.ToolstripView;
            obj.ConstraintsBrowserView = views.ConstraintsBrowserView;
        end

        function initialize(obj)
            obj.addViewListeners;
            obj.addModelListeners;
        end
    end

    % Construction methods
    methods (Access = private)

        function addViewListeners(obj)
        %addViewListeners Add listeners for events that occur on views

            addlistener(obj.ToolstripView, 'RequestLastSolutionInfo', @(~,~)obj.updateSolverReport());
            addlistener(obj.SceneCanvasView, 'MarkerButtonUp', @(source, event)obj.updateIKSolution(event) );
            addlistener(obj.ToolstripView, 'SolverSettingsEdited', @(source, event)obj.updateSolverSettings(event) );
            addlistener(obj.ToolstripView, 'RequestUpdatedIKSolution', @(source, event)obj.refreshIKSolution() );
            addlistener(obj.ToolstripView, 'ConstraintEdited', @(source, event)obj.updateSolverConstraints(event) );
            addlistener(obj.ToolstripView, 'RequestAddConstraint', @(source, event)obj.requestAddNewConstraint() );
            addlistener(obj.ToolstripView, 'RequestEditConstraint', @(source, event)obj.requestEditCurrentConstraint() );
            addlistener(obj.ToolstripView, 'RequestEditMarkerPoseConstraint', @(source, event)obj.requestEditMarkerPoseConstraint() );
            addlistener(obj.ToolstripView, 'RequestDisableMarker', @(source, event)obj.disableMarkerPoseVisuals() );
            addlistener(obj.ToolstripView, 'RequestEnableMarker', @(source, event)obj.enableMarkerPoseVisuals() );
            addlistener(obj.ToolstripView, 'RequestUpdateMarkerPose', @(source, event)obj.updateMarkerPose(event) );
            addlistener(obj.ConstraintsBrowserView, 'NodeSelectionChange', @(source, event)obj.updateSelectedConstraint(event) );
            addlistener(obj.ConstraintsBrowserView, 'ConstraintDeleteRequest', @(source, event)obj.deleteConstraintFromModel(event) );
            addlistener(obj.ConstraintsBrowserView, 'ConstraintEditRequest', @(source, event)obj.requestEditCurrentConstraint() );
            addlistener(obj.ConstraintsBrowserView, 'RequestConstraintStateChange', @(source, event)obj.requestToggleConstraintEnable(event) );
        end

        function addModelListeners(obj)
        %addModelListeners Add listeners for events that occur on models

            addlistener(obj.SolverModel, 'IKSolutionUpdated', @(source, event)obj.updateRBTState() );
            addlistener(obj.SolverModel, 'EEBodyChanged', @(source, event)obj.updateEEStateVisuals() );
            addlistener(obj.SolverModel, 'IKPropertiesChanged', @(source, event)obj.refreshIKSolution() );
            addlistener(obj.SolverModel, 'IKConstraintStateUpdated', @(source, event)obj.refreshIKConstraintViews(event) );
            addlistener(obj.SolverModel, 'InstructEnableConstraintEditing', @(source, event)obj.allowConstraintEditing(true) );
            addlistener(obj.SolverModel, 'InstructDisableConstraintEditing', @(source, event)obj.allowConstraintEditing(false) );
            addlistener(obj.SolverModel, 'InstructUpdateSolverTab', @(source, event)obj.updateSolverTabValues() );
            addlistener(obj.SolverModel, 'MarkerPoseTargetDisabled', @(source, event)obj.disableMarkerPoseVisuals() );
            addlistener(obj.SolverModel, 'MarkerPoseTargetEnabled', @(source, event)obj.enableMarkerPoseVisuals() );
        end

        function updateIKSolution(obj, event)
        %updateIKSolution Compute a new configuration given the latest pose data

            obj.SolverModel.updateMarkerPoseTarget(event.Pose);
            currConfig = obj.SceneModel.Config;
            obj.SolverModel.updateIK(currConfig);

        end

        function refreshIKSolution(obj)
        %refreshIKSolution Update the computed IK solution
        %   Sometimes it is necessary to re-compute the solution, e.g.
        %   when settings change, or when the user simply requests
        %   another update. In that case, refresh the solution using
        %   the current marker pose and joint configuration.

            obj.SolverModel.updateMarkerPoseTarget(obj.SceneCanvasView.MarkerPose);
            currConfig = obj.SceneModel.Config;
            obj.SolverModel.updateIK(currConfig);
        end

        function updateRBTState(obj)
        %updateRBTState Update the stored configuration on the scene model
        %   The solver model computes configuration and stores the most
        %   recent solution, but the scene model stores the actual
        %   configuration of the robot in the scene (which may be
        %   obtained in some other way). This method updates
        %   configuration state of the robot in the scene using the
        %   solution from the IK solver.

            newConfig = obj.SolverModel.LastSolution;

            % Update the scene model
            obj.SceneModel.updateConfig(newConfig);
            obj.ToolstripView.updateSolverReportFromInfo(obj.appendConstraintNameFieldToSolutionInfo());

        end
        function updateSolverReport(obj)
        %updateSolverReport Updates solver report with last solution info
        %   The callback updates the solver info view with the details from the
        %   last solution stored in the solver model.
            obj.ToolstripView.updateSolverReportFromInfo(obj.appendConstraintNameFieldToSolutionInfo());
        end

        function updateSolverSettings(obj, event)
        %updateSolverSettings Modify the solver settings in the model

            obj.SolverModel.updateIKSettings(event);
        end

        function updateSolverTabValues(obj)
        %updateSolverTabValues Update the solver tab so it displays the current solver

            solverObj = obj.SolverModel.IKSolver;
            obj.ToolstripView.updateSolverTabValues(solverObj);

        end

        function updateSolverConstraints(obj, event)
        %updateSolverConstraints Modify the solver constraints in the model

            obj.SolverModel.updateIKConstraintsMap(event);
        end

        function updateSelectedConstraint(obj, event)
        %updateSelectedConstraint Update the selected constraint in the model

            obj.SolverModel.updateSelectedConstraint(event);

        end

        function deleteConstraintFromModel(obj, deleteEvent)
        %deleteConstraintFromModel Delete constraint from the solver model

            obj.SolverModel.removeConstraintsFromSolverModel(deleteEvent);
        end

        function requestAddNewConstraint(obj)
        %requestAddNewConstraint Request a new constraint draft in the solver model
        %   This method indicates to the solver model that it should
        %   supply the inputs for a new constraint. This method does
        %   NOT add a new constraint; that can happen only after the
        %   constraint has been applied.

            [constraintKey, constraintName] = obj.SolverModel.getNewConstraintData();
            obj.ToolstripView.editNewConstraint(constraintKey, constraintName);
        end

        function requestEditCurrentConstraint(obj)
        %requestEditCurrentConstraint Request the solver model to send out an edit constraint event + data

            [constraintKey, constraintName, constraintType, constraintData] = obj.SolverModel.getActiveConstraintData();
            obj.ToolstripView.editExistingConstraint(constraintKey, constraintName, constraintType, constraintData);

        end

        function requestEditMarkerPoseConstraint(obj)
        %requestEditMarkerPoseConstraint Request the solver model to send out an edit constraint event + data for the marker pose constraint

            [constraintKey, constraintName, constraintType, constraintData] = obj.SolverModel.getMarkerPoseConstraintData();
            obj.ToolstripView.editExistingConstraint(constraintKey, constraintName, constraintType, constraintData);
        end

        function requestToggleConstraintEnable(obj, event)
        %requestToggleConstraintEnable Toggle whether or not a constraint is active in the solver model

            obj.SolverModel.toggleConstraintEnable(event);

        end

        function disableMarkerPoseVisuals(obj)
        %disableMarkerPoseVisuals Hide the marker in the canvas & disable associated toolstrip visuals

            obj.SceneCanvasView.hideMarker();

        end

        function enableMarkerPoseVisuals(obj)
        %enableMarkerPoseVisuals Show the marker in the canvas & enable associated toolstrip visuals

            obj.SceneCanvasView.showMarker;

        end

        function updateMarkerPose(obj, evt)
        %updateMarkerPose Move the marker to a specific target pose

            poseTarget = evt.Pose;
            obj.SceneCanvasView.updateMarkerPose(poseTarget);

        end

        function allowConstraintEditing(obj, isConstraintEditable)
        %makeConstraintEditable Enable/Disable the constraint button in the toolstrip

            obj.ToolstripView.updateConstraintButtonEnabledState(isConstraintEditable);

        end

        function refreshIKConstraintViews(obj, evt)
        %refreshIKConstraintViews Refresh the views that display constraint data
        %   This method is called when the constraint state is updated,
        %   and updates the associated visuals that display information
        %   about constraint state (e.g. whether or not the constraint
        %   is satisfied).

            obj.ConstraintsBrowserView.updateViewContent(evt);

            % Update stored configuration data associated with any changed
            % constraints
            obj.ConfigModel.resetConfigConstraintData(evt);

        end

        function updateEEStateVisuals(obj)
        %updateEEStateVisuals Update views that are dependent on the end-effector state

            eeBodyPose = obj.SolverModel.LastSolutionEEPose;
            obj.SceneCanvasView.updateMarkerPose(eeBodyPose);
            obj.ToolstripView.updateMarkerBodySelection(obj.SceneModel.MarkerBodyKey);

            %TODO: Update the piece of the toolstrip that displays the marker pose

        end
    end
    methods(Access=private)
        function info=appendConstraintNameFieldToSolutionInfo(obj)
        %appendConstraintNameFieldToSolutionInfo
        %   The solution info which the model holds doesn't have the name
        %   of the constraint a violation is associated with. It is the
        %   unique identifier of the user-defined constraint in the
        %   Constraints Browser View. Thus, since this info is required by
        %   the Toolstrip to generate the info view and report violations,
        %   the struct adds a field called "Name" atop of existing fields
        %   in the strcut in the struct-array "ConstraintViolations".
            info=obj.SolverModel.LastSolutionInfo;
            if(~isempty(info))
                lastsolconstraintkeys=obj.SolverModel.LastSolutionConstraintKeys;
                for i=1:size(info.ConstraintViolations,2)
                    info.ConstraintViolations(i).Name=...
                        obj.SolverModel.ConstraintsMap(lastsolconstraintkeys{i}).Name;
                end
            end
        end
    end

end
