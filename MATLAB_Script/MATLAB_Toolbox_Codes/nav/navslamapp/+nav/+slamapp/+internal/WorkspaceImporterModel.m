classdef WorkspaceImporterModel < handle & ...
        matlab.mixin.SetGet & ...
        robotics.appscore.internal.mixin.MsgCatalogHelper & ...
        robotics.appscore.internal.mixin.SaveLoadTools
    %This class is for internal use only. It may be removed in the future.

    %WorkspaceImporterModel This class encapsulates the Model portion of the
    %   Model/View/Controller design pattern used in the SLAM app for
    %   importing sensor data from MATLAB workspace

    % Copyright 2020 The MathWorks, Inc.
    
    properties
        %Scans
        Scans
        
        %Poses
        Poses
        
        %LidarRange
        LidarRange
    end
    
    properties
        %AvailablePercentagesToKeep
        AvailablePercentagesToKeep

        %AvailableScansVars
        AvailableScansVars
        
        %AvailablePosesVars
        AvailablePosesVars 
    end
    
    properties
        %Applied Applied workspace importer tab settings
        Applied

        %SelectedScansVarIndex
        SelectedScansVarIndex
        
        %SelectedPosesVarIndex
        SelectedPosesVarIndex
        
        %SelectedPercentageToKeepIndex
        SelectedPercentageToKeepIndex
        
    end
    
    properties (SetObservable)
        %Tentative Tentative workspace importer tab settings (not yet applied)
        Tentative
        
        %InfoStrings
        InfoStrings
    end

    properties (SetObservable, Transient)
        %TabRefreshTrigger
        TabRefreshTrigger = 1
    end    
    
    
    events
        WorkspaceImporterModel_RequestUpdateWSImporterTab
        WorkspaceImporterModel_MaxLidarRangeEstimateAvailable
    end
    
    methods
        function obj = WorkspaceImporterModel()
            %WorkspaceImporterModel Constructor
            obj.AvailablePercentagesToKeep = {'100'; '50'; '33.3'; '25'; '20'; '12.5'; '10'; '5'; '1'};
            obj.MsgIDPrefix = 'nav:navslamapp:wsimportermodel';
            
            obj.resetUserConfig();
        end
        
        function resetUserConfig(obj)
            %resetUserConfig
            obj.SelectedScansVarIndex = 1;
            obj.Applied.SelectedScansVar = '';
            obj.SelectedPosesVarIndex = 1;
            obj.Applied.SelectedPosesVar = '';
            obj.SelectedPercentageToKeepIndex = 1;
            obj.Applied.SelectedPercentageToKeep = '100';
            obj.InfoStrings = {'','',''};
            
            obj.Tentative = obj.Applied;
            
            obj.AvailableScansVars = {''};
            obj.AvailablePosesVars = {''};
        end
        
        function [varStr, varStrScans, varStrPoses] = getWorkspaceVariables(obj) %#ok<MANU>
            %getWorkspaceVariables Read variables from workspace and
            %   convert the variable information into displayable char arrays 

            wsVars = evalin('base','whos');
            wsVars = orderfields(wsVars);
            
            % initial filtering
            mask = zeros(0);
            maskScans = zeros(0);
            maskPoses = zeros(0);
            for k = 1:numel(wsVars)
                varType = wsVars(k).class;
                varSize = wsVars(k).size;
                % throw away any data in dimension higher than 2
                if numel(varSize) > 2
                    continue;
                end
                
                % keep: a) 1-D nonempty cell array, OR
                %       b) 1-D nonempty array of lidarScan objects
                %       c) Nx3 double matrices
                if (strcmp(varType, 'cell') && all(varSize) && min(varSize) == 1 )
                    maskScans(k) = 1;
                    maskPoses(k) = 1;
                elseif (strcmp(varType, 'lidarScan') && all(varSize) && min(varSize) == 1 )
                    maskScans(k) = 1;
                    maskPoses(k) = 0;
                elseif (strcmp(varType, 'double') && varSize(2) == 3 && all(varSize))
                    maskScans(k) = 0;
                    maskPoses(k) = 1;
                else
                    maskScans(k) = 0;
                    maskPoses(k) = 0;
                end
                
                mask(k) = maskScans(k) | maskPoses(k);
            end
            wsVars = wsVars(logical(mask));
            [~, idxScans] = ismember(find(maskScans), find(mask));
            [~, idxPoses] = ismember(find(maskPoses), find(mask));
            
            numVars = length(wsVars);

            varStr = cell(numVars,2);
            varNames = {wsVars.name};
            longestVarName = max(cellfun('length',varNames)); 
            formatVarName = sprintf('%%-%ds',longestVarName+2); % left-justify
            formatSizeClass = sprintf('%%-12s %%-6s');
            formatAll = sprintf('%s%s', formatVarName, formatSizeClass);

            for n = 1:numVars
                if length(wsVars(n).size) == 3
                    szStr = sprintf('%dx%dx%d', wsVars(n).size);
                elseif length(wsVars(n).size) < 3
                    szStr = sprintf('%dx%d', wsVars(n).size);
                else
                    szStr = sprintf('%d-D', length(wsVars(n).size));
                end
                
                tmpStr = sprintf(formatAll,wsVars(n).name,...
                                 szStr, wsVars(n).class);

                varStr{n,1} = sprintf('%s', wsVars(n).name);
                varStr{n,2} = sprintf('%s',tmpStr);
            end
            
            
            varStrScans = varStr(idxScans, :);
            varStrPoses = varStr(idxPoses, :);
            
            varStr = [{'', ''}; varStr];
            varStrScans = [{'', ''}; varStrScans];
            varStrPoses = [{'', ''}; varStrPoses];
        end %getWorkspaceVariables
        
        
        function updateAvailableScansAndPosesVariables(obj)
            %updateAvailableScansAndPosesVariables
            [~, varStrScans, varStrPoses] = getWorkspaceVariables(obj);
            
            obj.AvailableScansVars = varStrScans;
            obj.AvailablePosesVars = varStrPoses;
            
        end
        
        function [isSuccess, errorTag, errorArgs] = extractData(obj)
            %extractData
            import robotics.appscore.internal.eventdata.*
            
            isSuccess = true;
            errorTag = [];
            errorArgs = {}; 

            % load scans
            if isempty(obj.Tentative.SelectedScansVar)
                isSuccess = false;
                errorTag = 'ScansVarNotSelected';
                return;
            end
            
            try
                scans = evalin('base', obj.Tentative.SelectedScansVar);
            catch % the variable might be deleted from workspace after it's selected
                isSuccess = false;
                errorTag = 'VarNotFound';
                errorArgs = {obj.Tentative.SelectedScansVar};
                return;
            end
            if iscell(scans)
                checkScans = cellfun(@(x) isa(x, 'lidarScan') && ~isempty(x), scans);
                if ~all(checkScans)
                    isSuccess = false;
                    errorTag = 'InvalidScans';                
                    errorArgs = {obj.Tentative.SelectedScansVar};
                    return;
                end
                obj.Scans = scans;
            else
                checkScans = arrayfun(@(x) ~isempty(x), scans);
                if ~all(checkScans)
                    isSuccess = false;
                    errorTag = 'InvalidScans';
                    errorArgs = {obj.Tentative.SelectedScansVar};
                    return;
                end
                obj.Scans = arrayfun(@(x) {x}, scans); % convert it to cell arrays                
            end
            
            % load poses (optional, could be empty)
            if ~isempty(obj.Tentative.SelectedPosesVar)
            
                try
                    poses = evalin('base', obj.Tentative.SelectedPosesVar);
                catch % the variable might be deleted from workspace after it's selected
                    isSuccess = false;
                    errorTag = 'VarNotFound';
                    errorArgs = {obj.Tentative.SelectedPosesVar};
                    return;                
                end

                if iscell(poses)
                    if numel(poses) ~= numel(obj.Scans)
                        isSuccess = false;
                        errorTag = 'NumPosesMismatch';
                        return;                    
                    end
                    checkPoses = cellfun(@(x) isa(x,'double') && isreal(x) && all(isfinite(x(:))) && numel(x) == 3 && ~isempty(x), poses);
                    if ~all(checkPoses)
                        isSuccess = false;
                        errorTag = 'InvalidPoses';
                        errorArgs = {obj.Tentative.SelectedPosesVar};
                        return;                   
                    end

                    obj.Poses = poses;
                    
                else % Nx3 matrix
                    if size(poses,1) ~= numel(obj.Scans)
                        isSuccess = false;
                        errorTag = 'NumPosesMismatch';
                        return;                        
                    end
                    
                    if isa(poses, 'double') && isreal(poses) && all(all(isfinite(poses))) && size(poses, 2) == 3
                        obj.Poses = mat2cell(poses, ones(size(poses,1),1), 3); % convert it to cell arrays
                    else
                        isSuccess = false;
                        errorTag = 'InvalidPoses';
                        errorArgs = {obj.Tentative.SelectedPosesVar};
                        return;                     
                    end
                end
            else
                obj.Poses = {};
            end
            
            % estimate lidar range
            upperBound = max(cellfun(@(x) max(x.Ranges), obj.Scans));
            lowerBound = min(cellfun(@(x) min(x.Ranges), obj.Scans));

            obj.LidarRange = [lowerBound, upperBound];
                        
            obj.Applied = obj.Tentative;

            percentStr = obj.Applied.SelectedPercentageToKeep;
            percent = str2double(percentStr);
            skip = round(1/percent*100);

            numOriginal = numel(obj.Scans);
            obj.Scans = obj.Scans(1:skip:end);
            obj.Poses = obj.Poses(1:skip:end);
            numNew = numel(obj.Scans);

            obj.InfoStrings = {obj.Applied.SelectedScansVar,...
                               obj.Applied.SelectedPosesVar, ...
                               sprintf('%s%% (%d/%d)', percentStr, numNew, numOriginal)};
            

            
            obj.notify('WorkspaceImporterModel_MaxLidarRangeEstimateAvailable', VectorEventData(obj.LidarRange) );
            
        end %extractData
        
        
        function clean(obj)
            %clean Return the WorkspaceImporterModel object to a pristine state
            obj.Scans = [];
            obj.Poses = [];
            obj.LidarRange = [];
            obj.AvailableScansVars = {''};
            obj.AvailablePosesVars = {''};
            
            obj.Applied = [];
            obj.SelectedScansVarIndex = [];
            obj.SelectedPosesVarIndex = [];
            obj.SelectedPercentageToKeepIndex = [];
        end
        
        function [infoStruct, extraInfoStruct] = saveProperties(obj)
            %saveProperties

            % basic properties
            infoStruct = saveProperties@robotics.appscore.internal.mixin.SaveLoadTools(obj);

            % observable properties
            extraInfoStruct.Tentative = obj.Tentative;
            extraInfoStruct.InfoStrings = obj.InfoStrings;
        end

        function loadProperties(obj, infoStruct1, infoStruct2)
            %loadProperties
            loadProperties@robotics.appscore.internal.mixin.SaveLoadTools(obj, infoStruct1);

            % load additional properties to update importer tab content
            if ~isempty(infoStruct2.Tentative) % as not all app sessions go through import stage
                obj.Tentative = infoStruct2.Tentative;
                obj.InfoStrings = infoStruct2.InfoStrings;
                obj.TabRefreshTrigger = 1;
            end
        end        
    end
    
end
