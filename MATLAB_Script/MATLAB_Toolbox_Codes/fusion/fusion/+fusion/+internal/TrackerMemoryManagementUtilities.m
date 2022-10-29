classdef TrackerMemoryManagementUtilities < handle
    % This is an internal class and may be removed or modified in a future
    % release.

    % Copyright 2021 The MathWorks, Inc.
    
    % This class calculates the bounds used for clusters and number of
    % detections for memory management. It assumes that the following
    % nontunable properties exist for the trackers
    %
    % MaxNumTracks
    % MaxNumDetectionsPerSensor
    % MaxNumDetectionsPerCluster
    % MaxNumTracksPerCluster
    % ClusterViolationHandling
    
    %#codegen
    methods (Access = protected, Abstract)
        tf = hasAssignmentClustering(tracker);
        out = getMaxNumInputDetections(tracker);
    end
    
    methods (Access = protected)
        function type = getClusterViolationHandlingType(obj)
            switch obj.ClusterViolationHandling
                case 'Terminate'
                    type = fusion.internal.ClusterViolationHandlingType(1);
                case 'Split and warn'
                    type = fusion.internal.ClusterViolationHandlingType(2);
                case 'Split'
                    type = fusion.internal.ClusterViolationHandlingType(3);
                otherwise
                    assert(false); % Never happens
            end
        end

        function [maxNumDetsPerCluster, maxNumTracksPerCluster] = getClusterBounds(tracker)
            isStaticMemCodegen = matlabshared.tracking.internal.fusion.isCodegenWithStaticMemory();
            hasClustering = hasAssignmentClustering(tracker);
            enabledMemManagement = tracker.EnableMemoryManagement;
            if ~hasClustering || (hasClustering && ~enabledMemManagement && ~isStaticMemCodegen) % Clustering off or on without memory management & dynamic allocation is allowed.
                maxNumDetsPerCluster  = inf;
                maxNumTracksPerCluster = inf;
            elseif ~enabledMemManagement && isStaticMemCodegen % Clustering on without memory management but dynamic allocation is disabled
                maxNumDetsPerCluster = getMaxNumInputDetections(tracker);
                maxNumTracksPerCluster = tracker.MaxNumTracks;
            elseif enabledMemManagement % Clustering on and memory management is on
                maxNumDetsPerCluster = tracker.MaxNumDetectionsPerCluster;
                maxNumTracksPerCluster = tracker.MaxNumTracksPerCluster;
            end
        end

        function maxNumDetectionsPerSensor = getMaxNumDetectionsPerSensor(tracker)
            if tracker.EnableMemoryManagement
                maxNumDetectionsPerSensor = tracker.MaxNumDetectionsPerSensor;
            else
                maxNumDetectionsPerSensor = getMaxNumInputDetections(tracker);
            end
        end

        function flag = isInactivePropertyImpl(tracker, prop)
            flag = (~tracker.EnableMemoryManagement || ~hasAssignmentClustering(tracker)) && any(strcmpi(prop,{'MaxNumDetectionsPerCluster','MaxNumTracksPerCluster','ClusterViolationHandling'}));
            flag = flag || ~tracker.EnableMemoryManagement && strcmpi(prop,'MaxNumDetectionsPerSensor');
        end

        function groups = getPropertyGroups(tracker)
            enableMemGroup = matlab.mixin.util.PropertyGroup(...
                {'EnableMemoryManagement'});

            memManagementProperties = {'MaxNumDetectionsPerSensor','MaxNumDetectionsPerCluster','MaxNumTracksPerCluster','ClusterViolationHandling'};

            if tracker.EnableMemoryManagement
                memGroup = matlab.mixin.util.PropertyGroup(...
                   memManagementProperties,getString(message('fusion:internal:TrackerMemoryManagementUtilities:MemoryManagementDisplay')));
            else
                memGroup = [];
            end

            groups = [enableMemGroup memGroup];
        end

        function validateMaxNumDetectionsPerSensor(obj, val)
            validateIntegerOrInf(obj, val, 'MaxNumDetectionsPerSensor');
        end

        function validateMaxNumTracksPerCluster(obj, val)
            validateIntegerOrInf(obj, val, 'MaxNumTracksPerCluster');
        end

        function validateMaxNumDetectionsPerCluster(obj, val)
            validateIntegerOrInf(obj, val, 'MaxNumDetectionsPerCluster');
        end

        function validateIntegerOrInf(obj, val, propName)
            validateattributes(val,{'numeric'},...
                {'real','positive','scalar'},class(obj),propName);
            if isfinite(val)
                validateattributes(val,{'numeric'},{'integer'},class(obj),propName);
            end
        end

        function validatePropertiesImpl(obj)
            % Check tha MaxNumDetectionsPerSensor <= MaxNumDetections
            if obj.EnableMemoryManagement
                validateattributes(obj.MaxNumDetectionsPerSensor,{'numeric'},{'scalar','<=',getMaxNumInputDetections(obj)},class(obj),'MaxNumDetectionsPerSensor');
            end
        end
    end

    methods (Static, Access = protected)
        function memManagementGroup = getPropertyGroupsImpl
            propMemManagement = {'MaxNumDetectionsPerSensor',...
                'MaxNumDetectionsPerCluster','MaxNumTracksPerCluster',...
                'ClusterViolationHandling'};
            memManagementSection = matlabshared.tracking.internal.getDisplaySection('fusion',...
                    'internal:TrackerMemoryManagementUtilities','', propMemManagement);
            memManagementGroup =  matlab.system.display.SectionGroup( ...
                    'Title', getString(message('fusion:internal:TrackerMemoryManagementUtilities:GroupMemoryManagementSection')), ...
                    'Sections', memManagementSection);
        end

        function msg = getDetectionClusterViolationMsg()
            msg = 'fusion:internal:TrackerMemoryManagementUtilities:DetectionClusterViolation';
        end

        function msg = getTrackClusterViolationMsg()
            msg = 'fusion:internal:TrackerMemoryManagementUtilities:TrackClusterViolation';
        end
    end
end
