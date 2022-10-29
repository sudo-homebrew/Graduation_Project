classdef rigidBodyTreeImportInfo < robotics.manip.internal.InternalAccess
    %rigidBodyTreeImportInfo Object for storing rigidBodyTree import information
    %   This object is created by the IMPORTROBOT function when converting
    %   a Simulink model using Simscape Multibody components. Get import
    %   information for specific bodies, joints, or blocks using the object
    %   functions. Changes to the Simulink model are not reflected in this
    %   object after calling IMPORTROBOT.
    %
    %   rigidBodyTreeImportInfo properties
    %      SourceModelName      - Name of the imported Simulink model
    %      RigidBodyTree        - Rigid body tree object 
    %      BlockConversionInfo  - Struct containing lists of blocks that were converted to preserve compatibility
    %
    %      The BlockConversionInfo structure is divided into three
    %      sub-structs, which each contain specific fields:
    %
    %         AddedBlocks: Blocks that were added to the model during the
    %         import process. This struct contains the following fields:
    %            ImplicitJoints     - Cell array of implicit joints added during conversion process
    %
    %         RemovedBlocks: Blocks that were removed from the Simulink
    %         model to enable the import process. This structure contains
    %         the following fields:
    %            ChainClosureJoints - Cell array of Simscape Multibody joint blocks that were removed to open closed chains
    %            SMConstraints      - Cell array of Simscape Multibody constraint blocks that were removed to preserve compatibility
    %            VariableInertias   - Cell array of Simscape Multibody variable inertia blocks that were removed to preserve compatibility
    %
    %         ConvertedBlocks: Blocks that were converted from the original
    %         Simulink model to enable the import process. This structure
    %         contains the following fields:
    %            Joints             - Cell array of Simscape Multibody joint blocks that were converted to fixed joints
    %            JointsSourceType   - Containers.Map object that maps the converted joints to their type in the original Simulink model. 
    %
    %   rigidBodyTreeImportInfo methods
    %      bodyInfo             - Get rigid body import information given a rigid body name
    %      bodyInfoFromJoint	- Get rigid body import information given a joint name
    %      bodyInfoFromBlock	- Get rigid body import information given a block path or substring
    %      showdetails          - Display details of the imported robot
    %
    %   See also importrobot
    
    %   Copyright 2018-2019 The MathWorks, Inc.
    
    properties (SetAccess = ?robotics.manip.internal.InternalAccess)
        %SourceModelName Name of the imported Simulink model
        SourceModelName
        
        %RigidBodyTree Associate RigidBodyTree model
        RigidBodyTree

        %BlockConversionInfo Struct containing lists of blocks that were converted to preserve compatibility
        %It has the following fields:
        %   AddedBlocks.ImplicitJoints       - Cell array of implicit joints added during conversion process
        %   RemovedBlocks.ChainClosureJoints - Cell array of Simscape Multibody joint blocks that were removed to open closed chains
        %   RemovedBlocks.SMConstraints      - Cell array of Simscape Multibody constraint blocks that were removed to preserve compatibility
        %   RemovedBlocks.VariableInertias   - Cell array of Simscape Multibody variable inertia blocks that were removed to preserve compatibility
        %   ConvertedBlocks.Joints           - Cell array of Simscape Multibody joint blocks that were converted to fixed joints
        %   ConvertedBlocks.JointsSourceType - Containers.Map object that maps the converted joints to their type in the original Simulink model. 
        BlockConversionInfo
    end
    
    properties (SetAccess = private, GetAccess = ?robotics.manip.internal.InternalAccess)
        %BodyInfoMap Map that associates bodies with contributing blocks
        BodyInfoMap
        
        %JointInfoMap Map that associates joints with contributing blocks
        JointInfoMap
        
        %BlocksInfoMap Map that associates blocks with associated bodies & joints
        BlocksInfoMap
        
        %ImportData Internal object storage for importdata inputs
        ImportData = struct()
        
        %ModelBlocks List of blocks in the model
        ModelBlocks
        
        %NonSearchableBlocks List of non-searchable blocks in the model
        NonSearchableBlocks
    end
    
    methods (Access = ?robotics.manip.internal.InternalAccess)
        function obj = rigidBodyTreeImportInfo(tree, modelName, bodyMap, jointMap, importData, allModelBlocks)
            %rigidBodyTreeImportInfo constructor
            
            obj.SourceModelName = modelName;
            obj.RigidBodyTree = tree;
            
            %Convert empty elements to empty cells for output consistency
            bodyCells = obj.convertEmptyCells(bodyMap.values);
            jointCells = obj.convertEmptyCells(jointMap.values);
            
            %Assign the containers.Map objects in the constructor so that
            %they are new and always independent of the input maps
            obj.BodyInfoMap = containers.Map(bodyMap.keys, bodyCells);
            if isempty(jointMap)
                %Trees with just a base have no associated joints
                obj.JointInfoMap = containers.Map.empty;
            else
                obj.JointInfoMap = containers.Map(jointMap.keys, jointCells);
            end
               
            %Store key information from translation struct
            obj.ImportData.CutJoints = importData.CutJoints;
            obj.ImportData.ConvertedJoints = importData.ConvertedJoints;
            obj.ImportData.ConvertedJointTypes = importData.ConvertedJointTypes;
            obj.ImportData.Constraints = importData.Constraints;
            obj.ImportData.VariableInertias = importData.VariableInertias;
            obj.ImportData.ImplicitJoints = importData.ImplicitJoints;
            
            %Generate conversion struct
            obj.BlockConversionInfo = obj.generateBlockConversionInfo;
            
            %Generate Blocks info map given the data in the other maps
            obj.BlocksInfoMap = obj.generateBlocksMap;
            
            %Generate list of blocks that are in model but do not map to
            %anything in the generated Rigid Body Tree
            obj.ModelBlocks = allModelBlocks;
            obj.NonSearchableBlocks = obj.generateOtherBlocksList;
        end
    end
    
    methods        
        function info = bodyInfo(obj, varargin)
            %bodyInfo Get body import information given body name or substring
            
            narginchk(1,2);
            [varargin{:}] = convertStringsToChars(varargin{:});
            
            %Get indices of matching body names based on input
            bodyNames = obj.BodyInfoMap.keys;
            foundBodyNames = obj.searchCellArray(bodyNames, varargin{:});
            
            %Get info from body names
            info = obj.getBodyInfo(foundBodyNames);
        end
        
        function info = bodyInfoFromJoint(obj, varargin)
            %bodyInfoFromJoint Get body import information given joint name or substring
            
            narginchk(1,2);
            [varargin{:}] = convertStringsToChars(varargin{:});
            
            %Get indices of matching body names based on input
            jointNames = obj.JointInfoMap.keys;
            foundJointNames = obj.searchCellArray(jointNames, varargin{:});
            foundBodyNames = obj.getBodyNamesFromJointNames(foundJointNames);
            
            %Get info from body names
            info = obj.getBodyInfo(foundBodyNames);
        end
        
        function info = bodyInfoFromBlocks(obj, varargin)
            %bodyInfoFromBlocks Get body import information given joint path or substring
            
            narginchk(1,2);
            [varargin{:}] = convertStringsToChars(varargin{:});
            
            %Get indices of matching body names based on input
            blockNames = obj.BlocksInfoMap.keys;
            foundBlockPaths = obj.searchCellArray(blockNames, varargin{:});
            foundBodyNames = obj.getBodyNamesFromBlockPaths(foundBlockPaths);
            
            %Get info from body names
            info = obj.getBodyInfo(foundBodyNames);
            
            %If info is empty, output the appropriate associated warning
            if isempty(info)
                %Search blocks in model that are not user-searchable in
                %traceability object
                foundOtherBlockPaths = obj.searchCellArray(obj.NonSearchableBlocks, varargin{:});
                
                if isempty(varargin)
                    %The user has not provided any inputs (varargin is
                    %empty), and info is empty. This implies that the model
                    %does not contain any joint or body blocks. This is
                    %typically the scenario when the model consists of just
                    %enough Simscape components so that the model compiles,
                    %and the output robot consists of a single body (its
                    %base). In this case, return empty.
                    return;
                
                elseif isempty(foundOtherBlockPaths)
                    %The user searched a string that did not appear
                    %anywhere in the model. Return the appropriate warning.
                    
                    warning(message('robotics:robotmanip:urdfimporter:NoMatchingBlocksInModel', ...
                        obj.SourceModelName, varargin{1}));
                else
                    %The user searched a string that corresponds to blocks
                    %in the model, but not a searchable block. Provide an
                    %error message that indicates how the user can obtain a
                    %list of searchable blocks.
                    
                    %Create executable code to add to warning message
                    objName = inputname(1);
                    getBodiesLink = sprintf('<a href="matlab:bodyInfoArray = bodyInfo(%s)">bodyInfoArray = bodyInfo(%s)</a>', ...
                    objName, objName);
                    jointBlocksAction = 'jointBlocksArray = cellfun(@(x)(cellstr(x.JointBlocks)),bodyInfoArray,''UniformOutput'',false); vertcat(jointBlocksArray{:})';
                    bodyBlocksAction = 'bodyBlocksArray = cellfun(@(x)(cellstr(x.BodyBlocks)),bodyInfoArray,''UniformOutput'',false); vertcat(bodyBlocksArray{:})';
                    
                    %Convert actions to links
                    bodyBlocksLink = sprintf('<a href="matlab:%s">%s</a>', bodyBlocksAction, bodyBlocksAction);
                    jointBlocksLink = sprintf('<a href="matlab:%s">%s</a>', jointBlocksAction, jointBlocksAction);
                    
                    %Output warning detailing what blocks are searchable
                    warning(message('robotics:robotmanip:urdfimporter:NoMatchingBlocksInTree', ...
                        obj.SourceModelName, varargin{1}, getBodiesLink, bodyBlocksLink, jointBlocksLink));
                end
            end
        end
        
        function highlightBlocks(~, blocks)
            %highlightBlocks Highlight blocks given a cell array of block paths
            blocks = cellstr(blocks);

            for i = 1:length(blocks)
               hilite_system(blocks{i});
            end
        end
        
        function showdetails(obj)
            %showdetails Display details of the imported robot
            %   showdetails(IMPORTINFO) displays details of each body in the 
            %   robot including the body name, associated joint name and
            %   type, and its parent name and children names. Additionally,
            %   the method provides links to the associate methods that can
            %   be used to more clearly define how the Simscape Multibody
            %   blocks from the Simulink source model were imported and
            %   mapped to the associated rigidBodyTree bodies and
            %   joints.
            %
            %   Example:
            %       % Load a model with Simscape Multibody components
            %       sm_double_pendulum
            %
            %       % Import system to rigidBodyTree
            %       [robot, importInfo] = importrobot(gcs);
            %
            %       % Display import details.
            %       showdetails(importInfo);
            
            objName = inputname(1);
            
            %showdetails Display details of the robot
            tree = obj.RigidBodyTree;
            
            breakLine = sprintf('--------------------\n');
            headerText = sprintf('Robot: (%d bodies)\n\n', int32(tree.NumBodies));
            
            widMaxBodyName = 10;
            widSLInfo = 24;
            widMaxJointName = 10;
            widJointType = 10;
            
            for i = 1:tree.NumBodies
                widMaxBodyName = ...
                    max(widMaxBodyName, length(tree.Bodies{i}.Name)+5);
                widMaxJointName = ...
                      max(widMaxJointName, length(tree.Bodies{i}.Joint.Name)+5);
            end  
            
            titlesText = sprintf('%4s   %*s   %*s   %*s   %*s   %*s   %*s(Idx)   Children Name(s)\n', ...
                     'Idx', ...
                     widMaxBodyName, 'Body Name',...
                     widSLInfo, 'Simulink Source Blocks',...
                     widMaxJointName, 'Joint Name',...
                     widSLInfo, 'Simulink Source Blocks',...
                     widJointType, 'Joint Type',...
                     widMaxBodyName+2, 'Parent Name' );
            titlesLines = sprintf('%4s   %*s   %*s   %*s   %*s   %*s   %*s-----   ----------------\n', ...
                     '---', ...
                     widMaxBodyName, '---------',...
                     widSLInfo, '----------------------',...
                     widMaxJointName, '----------',...
                     widSLInfo, '----------------------',...
                     widJointType, '----------',...
                     widMaxBodyName+2, '-----------');
            
             entriesText = cell(tree.NumBodies, 1);
            for i = 1:tree.NumBodies
                
                jointName = tree.Bodies{i}.Joint.Name;
                jointType = tree.Bodies{i}.Joint.Type;
                bodyName = tree.Bodies{i}.Name;
                
                bodyBlocks = sprintf('%s.bodyInfo(''%s'').BodyBlocks', objName, bodyName);
                jointBlocks = sprintf('%s.bodyInfoFromJoint(''%s'').JointBlocks', objName, jointName);
                
                bodyInfoLink = sprintf('<a href="matlab:bodyInfo(%s, ''%s'')">Info</a>', ...
                    objName, bodyName);
                bodyListLink = sprintf('<a href="matlab:%s">List</a>', ...
                    bodyBlocks);
                bodyHighlightLink = sprintf('<a href="matlab:%s.highlightBlocks(%s)">Highlight</a>', ...
                    objName, bodyBlocks);
                jointInfoLink = sprintf('<a href="matlab:bodyInfoFromJoint(%s, ''%s'')">Info</a>', ...
                    objName, jointName);
                jointListLink = sprintf('<a href="matlab:%s">List</a>', ...
                    jointBlocks);
                jointHighlightLink = sprintf('<a href="matlab:%s.highlightBlocks(%s)">Highlight</a>', ...
                    objName, jointBlocks);
                
                
                %Links have more fprintf length than they take up on screen
                widBodyInfoLinks = 1+length(bodyInfoLink);
                widJointInfoLinks = 1+length(jointInfoLink);
                indexText = sprintf('%4d', int32(tree.TreeInternal.Bodies{i}.Index));
                bodyNameText = sprintf('   %*s', widMaxBodyName, bodyName);
                bodyLinkText = sprintf('   %*s | %s | %s', widBodyInfoLinks, bodyInfoLink, bodyListLink, bodyHighlightLink);
                jointNameText = sprintf('   %*s', widMaxJointName, jointName);
                jointLinkText = sprintf('   %*s | %s | %s', widJointInfoLinks, jointInfoLink, jointListLink, jointHighlightLink);
                jointTypeText = sprintf('   %*s', widJointType, jointType);
                
                
                pid = tree.TreeInternal.Bodies{i}.ParentIndex;
                if pid > 0 
                    parent = tree.Bodies{pid};
                else
                    parent = tree.Base; 
                end
                
                % Estimate the number of digits for a body index
                p = pid;
                widID = 0;
                while p > 0.2
                    p = floor(p/10);
                    widID = widID+1;
                end
                widID = max(1, widID);
                parentNameText = sprintf('%*s(%*d)   ', widMaxBodyName+8-widID, parent.Name, widID, int32(pid));
               
                cids = find(int32(tree.TreeInternal.Bodies{i}.ChildrenIndices));
                childrenNamesText = cell(length(cids),1);
                for j = 1:length(cids)
                    c = tree.Bodies{cids(j)};
                    childrenNamesText{j,1} = sprintf('%s(%d)  ', c.Name, int32(cids(j)) ); 
                end
                 
                entriesText{i,1} = [indexText bodyNameText bodyLinkText ...
                    jointNameText jointLinkText jointTypeText ...
                    parentNameText childrenNamesText{:} newline];
            end
                
            detailsText = [breakLine ...
                headerText ...
                titlesText ...
                titlesLines ...
                entriesText{:} ...
                breakLine];
            disp(detailsText);
        end
        
        function newObj = copy(obj)
            %Copy Copy the info object
            
            newObj = rigidBodyTreeImportInfo(obj.RigidBodyTree, obj.SourceModelName, ...
                obj.BodyInfoMap, obj.JointInfoMap, obj.ImportData, obj.ModelBlocks);
        end
        
    end
    
    methods (Access = ?robotics.manip.internal.InternalAccess)
        function formattedCellArray = convertEmptyCells(~, cellArray)
            %convertEmptyCells Ensure that empty values are of type cell
            
            formattedCellArray = cellArray;
            for i = 1:numel(cellArray)
                if isempty(cellArray{i})
                    formattedCellArray{i} = {};
                end
            end
        end
        
        function cStruct = generateBlockConversionInfo(obj)
            %generateBlockConversionInfo Generate a struct with fields that specify conversion details
            
            importData = obj.ImportData;
            
            %Create a user-facing block conversion info struct
            cStruct = struct;
            cStruct.AddedBlocks = struct;
            cStruct.ConvertedBlocks = struct;
            cStruct.RemovedBlocks = struct;
            
            %Fill out any fields for any blocks that were added to the model
            cStruct.AddedBlocks.ImplicitJoints = importData.ImplicitJoints;
            
            %Fill out any fields for any blocks that were converted to
            %preserve compatibility
            cStruct.ConvertedBlocks.Joints = importData.ConvertedJoints;
            if ~isempty(importData.ConvertedJoints)
                cStruct.ConvertedBlocks.JointsSourceType = containers.Map(importData.ConvertedJoints, importData.ConvertedJointTypes);
            else
                cStruct.ConvertedBlocks.JointsSourceType = containers.Map.empty;
            end
            
            %Fill out any fields for any blocks that were removed from the
            %imported model to preserve compatibility
            cStruct.RemovedBlocks.ChainClosureJoints = importData.CutJoints;
            cStruct.RemovedBlocks.SMConstraints = importData.Constraints;
            cStruct.RemovedBlocks.VariableInertias = importData.VariableInertias;
            
        end
        
        function blocksMap = generateBlocksMap(obj)
            %generateBlocksMap Generate a map that has path names as keys and body names as values
            
            blocksMap = containers.Map.empty;
            bodyNames = obj.BodyInfoMap.keys;
            jointNames = obj.JointInfoMap.keys;
            
            % Assign keys & values for all blocks mapped from bodies
            for i = 1:numel(bodyNames)
                bodyBlocks = obj.BodyInfoMap(bodyNames{i});
                if ~iscell(bodyBlocks) && ~isempty(bodyBlocks)
                    %Ensure that blocks are always a cell so later indexing
                    %is consistent
                    bodyBlocks = {bodyBlocks};
                end
                for j = 1:numel(bodyBlocks)
                    blocksMap(bodyBlocks{j}) = bodyNames{i};
                end
            end
            
            % Assign keys and values for all blocks mapped from joints
            for i = 1:numel(jointNames)
                %Since the aim is to map blocks to body names, it is
                %necessary to get the joint blocks from the joint map, and
                %then relate the joint name to a body name. Then the joint
                %blocks can be assigned to that body name.
                jointBlocks = obj.JointInfoMap(jointNames{i});
                if ~iscell(jointBlocks) && ~isempty(jointBlocks)
                    %Ensure that blocks are always a cell so later indexing
                    %is consistent
                    jointBlocks = {jointBlocks};
                end
                bodyName = obj.getBodyNamesFromJointNames(jointNames{i});
                for j = 1:numel(jointBlocks)
                    blocksMap(jointBlocks{j}) = bodyName{1};
                end
            end
        end
        
        function otherBlocksList = generateOtherBlocksList(obj)
            %generateOtherBlocksList Generate list of blocks in model that do not map to tree
            %   This method outputs the list of block paths that are in the
            %   model but do not contribute to imported model dynamics, and
            %   therefore will not appear in the BlocksInfoMap.
            
            blocksInTree = obj.BlocksInfoMap.keys;
            blocksInModel = obj.ModelBlocks;
            otherBlocksList = setdiff(blocksInModel, blocksInTree);
        end
        
        function strMatches = searchCellArray(~, inputCell, varargin)
            %searchCellArray Search a cell array by substring and return a cell array of values that contain that substring
            
            if nargin > 2
                %Search the cell array inputCell to find entries that
                %contain the string in varargin{1}
                logicalCell = strfind(inputCell, varargin{1});
                logicalVec = cellfun(@(s) ~isempty(s), logicalCell);
                strMatches = inputCell(logicalVec);
            else
                %No input argument; output returns input
                strMatches = inputCell;
            end
        end
        
        function info = getBodyInfo(obj, foundBodyNames)
            %getBodyInfo Get body info (or array of info struct) from a cell array of body names
                        
            N = numel(foundBodyNames);
            info = cell(N,1);
            for i = 1:N
                rbName = foundBodyNames{i};
                if strcmp(rbName, obj.RigidBodyTree.BaseName)
                    %Rigid body is the base (it has no joint)
                    rbjName = [];
                    bodySourceBlocks = obj.BodyInfoMap(rbName);
                    jointSourceBlocks = {};
                else
                    rbjName = obj.RigidBodyTree.getBody(rbName).Joint.Name;
                    bodySourceBlocks = obj.BodyInfoMap(rbName); 
                    jointSourceBlocks = obj.JointInfoMap(rbjName);
                end
                
                %Assign to output struct
                info{i}.BodyName = rbName;
                info{i}.JointName = rbjName;
                info{i}.BodyBlocks = bodySourceBlocks;
                info{i}.JointBlocks = jointSourceBlocks;
            end
            
            if N == 1
                info = info{:};
            end
        end
        
        function bodyNames = getBodyNamesFromJointNames(obj, jointNames)
            %getBodyNamesFromJointNames Get cell array of body names given the associated joint names
            
            % Ensure that jointNames can be a single char
            if iscell(jointNames)
                N = numel(jointNames);
            else
                N = 1;
                jointNames = {jointNames};
            end
            
            bodyNames = cell(1, N);
            for i = 1:N
                bodyIdx = obj.RigidBodyTree.TreeInternal.findBodyIndexByJointName(jointNames{i});
                bodyNames{i} = obj.RigidBodyTree.BodyNames{bodyIdx};
            end
        end
        
        function bodyNames = getBodyNamesFromBlockPaths(obj, blockPaths)
            %getBodyNamesFromBlockPaths Get cell array of body names given the associated block paths
            
            % Ensure that blockPaths can be a single char
            if iscell(blockPaths)
                N = numel(blockPaths);
            else
                N = 1;
                blockPaths = {blockPaths};
            end
            
            bodyNames = cell(1, N);
            for i = 1:N
                bodyNames{i} = obj.BlocksInfoMap(blockPaths{i});
            end
            
            %Only return unique values
            bodyNames = unique(bodyNames);
        end
        
    end
    
    
end
