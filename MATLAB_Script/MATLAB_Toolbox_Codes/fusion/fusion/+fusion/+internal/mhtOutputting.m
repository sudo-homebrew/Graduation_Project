classdef mhtOutputting < matlab.System
% mhtOutputting A class that provides the output from an MHT tracker
%   outputter = mhtOutputting returns a multiple-hypotheses tracking track
%   outputting class with default properties.
%
%   outputter = mhtOutputting('Name',value) returns a mhtOutputting object
%   by specifying its properties as name-value pair arguments. Unspecified
%   properties have default values. See the list of properties below.
%
%   mhtOutputting properties:
%       OutputRepresentation - Choice of output representation
%       HypothesesToOutput   - An array of hypotheses to output

% Copyright 2018 The MathWorks, Inc.

%#codegen

% Public, tunable properties
    % Public, non-tunable properties
    properties(Hidden,Constant)
        %OutputRepresentationSet - A set of all possible methods to use for output.
        OutputRepresentationSet = matlab.system.StringSet({'Hypothesis','Tracks','Clusters'});
    end
    
    properties(Nontunable)
        %OutputRepresentation - Choice of output representation
        %   Choose the output method from a list of the following methods:
        %     'Hypothesis'  - Outputs each hypothesis separately.
        %     'Tracks'      - Outputs the centroid of each track based on 
        %                     its track branches.
        %     'Clusters'    - Outputs the centroid of each cluster. Similar
        %                     to 'Tracks' output, but includes all the
        %                     tracks within a cluster.
        %
        % Default: 'Tracks'
        OutputRepresentation = 'Tracks';
    end
    
    properties
        %HypothesesToOutput - An array of hypotheses to output
        %   Choose which hypotheses to output as an array of indices. The
        %   indices must all be less than or equal to the maximum number of
        %   hypotheses provided by the tracker.
        %
        % Default: 1
        HypothesesToOutput = 1;
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private, Nontunable)
        %pOutputRepresentation - A scalar that represents which output method was
        %   chosen. 1 = 'Hypothesis', 2 = 'Coordinated', 3 = 'Continuous'
        pOutputRepresentation
    end

    methods
        % Constructor
        function obj = mhtOutputting(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            if strcmpi(obj.OutputRepresentation,'Hypothesis')
                obj.pOutputRepresentation = 1;
            elseif strcmpi(obj.OutputRepresentation,'Clusters')
                obj.pOutputRepresentation = 2;
            else % Tracks output
                obj.pOutputRepresentation = 3;
            end
        end

        function [confirmedTracks, tentativeTracks, allTracks] = stepImpl(obj,tracks,hypotheses,probs,clusters)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            numTracks = numel(tracks);
            allFlag = true(numTracks,1);
            switch obj.pOutputRepresentation 
                case 1
                    allTracks = hypsOutput(obj,tracks,hypotheses,allFlag);
                case 2
                    allTracks = clustersOutput(obj,tracks,allFlag,probs,clusters);
                case 3
                    allTracks = tracksOutput(obj,tracks,allFlag,probs);
            end
            numTracks = numel(allTracks);
            confFlag = false(numTracks,1);
            for i = 1:numTracks
                confFlag(i) = allTracks(i).IsConfirmed;
            end
            confirmedTracks = allTracks(confFlag,1);
            tentativeTracks = allTracks(~confFlag,1);
        end

        %% Methods used by the stepImpl method
        function outTracks = hypsOutput(obj,tracks,hypotheses,list)
            % Output the tracks in each hypothesis. This option requires
            % HypothesesToOutput to be a scalar. It is a tunable property
            % so it could be modified from one call to the next
            toOutput = any(and(list,hypotheses(:,obj.HypothesesToOutput)),2);
            outTracks = tracks(toOutput);
        end
        
        function outTracks = clustersOutput(~,tracks,list,probs,clusters)
            trs = tracks(list);
            if isempty(trs)
                outTracks = trs;
                return
            end
            cls = clusters(list,:);
            numClusters = size(cls,2);
            for i = numClusters:-1:1
                if ~any(cls(:,i))
                    cls(:,i) = [];
                    numClusters = numClusters - 1;
                end
            end
            outTracks = repmat(trs(1),[numClusters,1]);
            for i = 1:numClusters
                clusterProbs = probs(cls(:,i));
                clusterTracks = trs(cls(:,i));
                
                if isempty(clusterTracks) || sum(clusterProbs)==0
                    continue
                end
                
                [xC,PC,isConfirmed,minID] = calcCluster(clusterTracks,clusterProbs);                
                outTracks(i) = clusterTracks(1);
                outTracks(i).State = xC;
                outTracks(i).StateCovariance = PC;
                outTracks(i).IsConfirmed = isConfirmed;
                outTracks(i).TrackID = minID;
            end
        end
        
        function outTracks = tracksOutput(~,tracks,list,probs)
            trs = tracks(list);
            if isempty(trs)
                outTracks = trs;
                return
            end
            numTracks = numel(tracks);
            trIDs = zeros(numTracks,1,'like',tracks(1).TrackID);
            for i = 1:numTracks
                trIDs(i) = tracks(i).TrackID;
            end
            uniqIDs = unique(trIDs);
            numIDs = numel(uniqIDs);
            outTracks = repmat(trs(1),[numIDs,1]);
            for i = 1:numIDs
                thisID = uniqIDs(i);
                inds = (thisID == trIDs);
                trackProbs = probs(inds);
                trackBranches = trs(inds);
                
                if isempty(trackBranches) || sum(trackProbs)==0
                    continue
                end
                
                [xC,PC,isConfirmed] = calcCluster(trackBranches,trackProbs);                
                outTracks(i) = trackBranches(1);
                outTracks(i).State = xC;
                outTracks(i).StateCovariance = PC;
                outTracks(i).IsConfirmed = isConfirmed;
            end
        end

        %% Backup/restore functions
        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@matlab.System(obj);

            % Set private and protected properties
            s.pOutputRepresentation = obj.pOutputRepresentation;
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s

            % Set private and protected properties
            obj.pOutputRepresentation = s.pOutputRepresentation; 

            % Set public properties and states
            loadObjectImpl@matlab.System(obj,s,wasLocked);
        end

        %% Advanced functions
        function validatePropertiesImpl(obj)
            % Validate related or interdependent property values
            validateattributes(obj.HypothesesToOutput, {'numeric'}, ...
                {'real','positive','integer','row'},mfilename,'HypothesesToOutput');
        end

        function flag = isInputSizeMutableImpl(~,~)
            % Return false if input size is not allowed to change while
            % system is running
            flag = true;
        end

        function flag = isInactivePropertyImpl(~,~)
            % Return false if property is visible based on object 
            % configuration, for the command line and System block dialog
            flag = false;
        end
    end
end

function [xC,PC,isConfirmed,minID] = calcCluster(clusterTracks,clusterProbs)
nT = sum(clusterProbs);
isConfirmed = false;
minID = intmax(class(clusterTracks(1).TrackID));

% Calculate cluster centroid
numTracks = numel(clusterTracks);
states = zeros(numel(clusterTracks(1).State),numTracks,'like',clusterTracks(1).State);
for k = 1:numTracks
    states(:,k) = clusterTracks(k).State;
    isConfirmed = isConfirmed | clusterTracks(k).IsConfirmed;
    minID = min(minID, clusterTracks(k).TrackID);
end
xC = (states * clusterProbs)/nT;

% Calculate cluster covariance
PC = -xC*xC';
for k = 1:numTracks
    xk = clusterTracks(k).State;
    Pk = clusterTracks(k).StateCovariance;
    PC = PC + clusterProbs(k) * (Pk + xk*xk') / nT;
end
end
