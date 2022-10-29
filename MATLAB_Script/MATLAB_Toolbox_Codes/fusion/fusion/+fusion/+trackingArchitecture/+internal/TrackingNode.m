classdef TrackingNode < handle
    %TrackingNode  An adapter that manages the tracking architecture node
    %  fusion.trackingArchitecture.internal.TrackingNode is an adapter that
    %  connects the trackingArchitecture to Trackers and TrackFusers and
    %  manages how they are triggered by the trackingArchitecture
    %
    % TrackingNode properties:
    %   Name        - Name for the node
    %   ManagedNode - The tracker or track fuser managed by this object
    %   Inputs      - A list of source indices to this node
    %   UpdateRate  - The rate to update the managed node
    %   ToOutput    - A flag. True if the node outputs are part of the architecture output
    %   Index       - Node index in the architecture
    %   IsFuser     - A flag. True if the node is a fuser
    %   IsTracker   - A flag. True if the node is a tracker
    %
    % TrackingNode methods:
    %   nextUpdateTime - The next time in which this node needs to be updated
    %   step           - Update the managed node 
    %   summary        - Provides the node summary
    %   
    % See also: trackingArchitecture, fusion.trackingArchitecture.Tracker,
    % fusion.trackingArchitecture.TrackFuser
    
    properties
        %Name A user readable name for the node
        % Define the name of the node. The name is used when displaying the
        % node in the summary or show methods
        %
        % Default: if not given by the user, the name is the class of the
        % ManagedNode
        Name
        
        %ManagedNode The tracker or track fuser managed by this object
        % Define the tracker or track fuser that the TrackingNode manages.
        % A tracker must satisfy the fusion.trackingArchitecture.Tracker
        % interface. A track fuser must satisfy the
        % fusion.trackingArchitecture.TrackFuser interface
        %
        % Default: none. Must be set on construction
        ManagedNode
        
        %Inputs A list of source indices to this node
        % Define the list of source indices to this node. Source indices
        % must be real, positive, integer, and unique values. They must
        % also be different than the index defined for the ManagedNode.
        Inputs
        
        %UpdateRate The rate to update the managed node
        % Define the update rate of the managed node:
        % A value of -1 means that the managed node is updated with every
        % update to the trackingArchitecture. If not set to -1, the value
        % means how many times per second the managed object is expected to
        % be updated.
        %
        % Default: -1
        UpdateRate = -1
        
        %ToOutput Node provides tracks to the architecture output
        % Specify whether tracks from this node are provided in the output
        % of the trackingArchitecture step. 
        %
        % Default: true
        ToOutput = true
    end
    
    % Consider making these properties accessible only to
    % trackingArchitecture
    properties
        Index
        IsFuser
        IsTracker
    end
    
    properties(Access = private)
        %pLastTime The last timestamp this node was stepped
        pLastTime
    end
    
    methods
        function obj = TrackingNode(varargin)
            p = inputParser;
            addRequired(p, 'ManagedNode');
            addRequired(p, 'Inputs');
            addParameter(p,'Name', '');
            addParameter(p,'UpdateRate',-1);
            addParameter(p,'ToOutput',true);
            parse(p,varargin{:});
            
            % Validate name
            validateattributes(p.Results.Name, {'char','string'},{},...
                'trackingArchitecture','Name');
            
            % Validate inputs
            validateattributes(p.Results.Inputs,{'numeric'},...
                {'real','finite','positive','integer','vector'},...
                'trackingArchitecture','Inputs');
            
            % Validate UpdateRate
            validateattributes(p.Results.UpdateRate,{'numeric'},...
                {'real','finite','scalar'},...
                'trackingArchitecture','UpdateRate');
            
            % Validate ToOutput
            validateattributes(p.Results.ToOutput,{'binary','logical'},...
                {'scalar'},...
                'trackingArchitecture','ToOutput');
            
            obj.ManagedNode = p.Results.ManagedNode;
            obj.Inputs = p.Results.Inputs;
            obj.UpdateRate = p.Results.UpdateRate;
            obj.ToOutput = p.Results.ToOutput;
            
            % Internal management
            ind = nodeIndex(obj);
            if isFuser(obj)
                validateattributes(ind,{'numeric'},{'real','positive','integer','scalar'},...
                    'trackingArchitecture/addTrackFuser','FuserIndex')
            elseif isTracker(obj)
                validateattributes(ind,{'numeric'},{'real','positive','integer','scalar'},...
                    'trackingArchitecture/addTracker','TrackerIndex')
            end
            if strcmp(char(p.Results.Name),'')
                obj.Name = class(obj.ManagedNode);
            else
                obj.Name = char(p.Results.Name);
            end
            
            obj.Index = ind;
            obj.IsFuser = isFuser(obj);
            obj.IsTracker = isTracker(obj);
            
            % Initialize last time information
            initializeLastTime(obj);
        end
        
        function set.Inputs(obj,value)
            coder.internal.assert(numel(value) == numel(unique(value)),...
                'fusion:trackingArchitecture:expectedUniqueInputs');
            obj.Inputs = value;
        end
        
        function set.UpdateRate(obj,value)
            if value ~= -1
                validateattributes(value,{'numeric'},...
                {'positive'},...
                'trackingArchitecture','UpdateRate');
            end
            obj.UpdateRate = value;
        end
        
        function t = nextUpdateTime(obj)
            if obj.UpdateRate == -1 && obj.pLastTime < 0
                t = 0;
            elseif obj.UpdateRate == -1
                t = obj.pLastTime + eps(obj.pLastTime);
            else
                t = obj.pLastTime + 1/obj.UpdateRate;
            end
        end
        
        function [tracks,dataToNode] = step(obj, data, time)
            %STEP Update the managed node
            
            % From the input data, collect all the data that needs to
            % update the ManagedNode
            dataToNode = collectData(obj, data);
            
            % Update the ManagedNode and report the tracks
            if isLocked(obj.ManagedNode) || ~isempty(dataToNode)
                trks = step(obj.ManagedNode, dataToNode, time);
                tracks = convertTracksFromPHD(obj,trks);
            else
                tracks = objectTrack.empty(0,1);
            end
            
            % Update the last time stamp for this node
            obj.pLastTime = time;
        end
        function s = summary(obj)
            %SUMMARY Return a struct with summary of this node
            if obj.IsFuser
                type = 'F';
                ins = obj.Inputs; 
            elseif obj.IsTracker
                type = 'T';
                ins = []; % No fuser inputs for a tracker
            end
            s = struct(...
                'System', strcat(type,num2str(obj.Index),':',obj.Name),...
                'ArchitectureInputs',obj.Inputs,...
                'FuserInputs',ins,...
                'ArchitectureOutput',double(obj.ToOutput));
        end
        
        function s = saveobj(obj)
            s.Name = obj.Name;
            s.ManagedNode = obj.ManagedNode;
            s.Inputs = obj.Inputs;
            s.UpdateRate = obj.UpdateRate;
            s.ToOutput = obj.ToOutput;
            s.Index = obj.Index;
            s.IsFuser = obj.IsFuser;
            s.IsTracker = obj.IsTracker;
            s.pLastTime = obj.pLastTime;
        end
        
        function clonedNode = clone(obj)
            s = saveobj(obj);
            clonedNode = fusion.trackingArchitecture.internal.TrackingNode.loadobj(s);
        end
        
        function release(obj)
            release(obj.ManagedNode);
            initializeLastTime(obj);
        end
        
        function reset(obj)
            reset(obj.ManagedNode);
            initializeLastTime(obj);
        end
    end
    
    methods(Static)
        function obj = loadobj(s)
            obj = fusion.trackingArchitecture.internal.TrackingNode(...
                s.ManagedNode, s.Inputs);
            obj.Name = s.Name;
            obj.UpdateRate = s.UpdateRate;
            obj.ToOutput = s.ToOutput;
            obj.Index = s.Index;
            obj.IsFuser = s.IsFuser;
            obj.IsTracker = s.IsTracker;
            obj.pLastTime = s.pLastTime;
        end
        
        function s = sampleSummary
            s = repmat(struct2table(struct(...
                'System', ' ',...
                'ArchitectureInputs',' ',...
                'FuserInputs',' ',...
                'ArchitectureOutput',0)),0,1);
        end
    end
    
    methods(Access = private)
        function initializeLastTime(obj)
            if obj.UpdateRate > 0
                obj.pLastTime = -1/obj.UpdateRate;
            else
                obj.pLastTime = -cast(eps(1),'like',obj.UpdateRate);
            end
        end
        function ind = nodeIndex(obj)
            if isTracker(obj)
                ind = obj.ManagedNode.TrackerIndex;
            else
                ind = obj.ManagedNode.FuserIndex;
            end
        end
        function tf = isFuser(obj)
            tf = isa(obj.ManagedNode,'fusion.internal.FuserManager') || ...
                    isa(obj.ManagedNode,'fusion.trackingArchitecture.TrackFuser');
        end
        function tf = isTracker(obj)
            tf = isa(obj.ManagedNode,'matlabshared.tracking.internal.fusion.TrackManager') || ...
                    isa(obj.ManagedNode,'fusion.trackingArchitecture.Tracker') || ...
                    isa(obj.ManagedNode,'trackerPHD');
        end
        function dataToNode = collectData(obj, data)
            numData = numel(data);
            toNode = false(numData,1);
            for i = 1:numData
                toNode(i) = any(dataSource(obj,data,i)==obj.Inputs);
            end
            dataToNode = data(toNode);
        end
        function sourceInd = dataSource(~,data,i)
            if iscell(data)
                thisData = data{i};
            else
                thisData = data(i);
            end
            if isa(thisData, 'objectDetection')
                sourceInd = thisData.SensorIndex;
            elseif isa(thisData, 'objectTrack')
                sourceInd = thisData.SourceIndex;
            else % struct
                if isfield(thisData, 'SensorIndex')
                    sourceInd = thisData.SensorIndex;
                else
                    sourceInd = thisData.SourceIndex;
                end
            end
        end
        
        function tracks = convertTracksFromPHD(obj,trks)
            if isa(obj.ManagedNode,'trackerPHD')
                if isempty(trks)
                    tracks = objectTrack.empty(0,1);
                else
                    tracks = repmat(objectTrack(trks(1)),numel(trks),1);
                    for i = 1:numel(trks)
                        tracks(i) = objectTrack(trks(i));
                    end
                end
            else
                tracks = trks;
            end
        end
    end
end