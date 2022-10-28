classdef PoseGraphBase < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%POSEGRAPHBASE Base class for pose graph representation

%   Copyright 2017-2021 The MathWorks, Inc.

%#codegen

    properties (Abstract, Constant, Access = ?nav.algs.internal.InternalAccess)
        %PoseLength Length of the pose vector in this pose graph
        PoseLength

        %PoseDeltaLength Length of the vector that represents a infinitesimal pose change
        PoseDeltaLength
        
        %PointLength Length of vector that represents a point landmark
        PointLength 

        %TformSize Size of the homogeneous coordinates of a pose
        TformSize

        %PoseInfoMatSize Size of the information matrix associated with
        %   pose-pose measurement
        PoseInfoMatSize
        
        %PointInfoMatSize Size of the information matrix associated with 
        %   pose-point measurement
        PointInfoMatSize

        % Compact Information Matrix:
        % The upper half (including the diagonal) of the information
        % matrix is flattened along the row direction into a vector. 
        % This vector is referred to as "Compact Information Matrix"
        
        %CompactPoseInfoMatLength Length of the compact form of the information
        %   matrix associated with pose-pose measurement
        CompactPoseInfoMatLength

        %CompactPointInfoMatLength Length of the compact form of the information
        %   matrix associated with pose-point measurement
        CompactPointInfoMatLength        
        
        %DefaultCompactPoseInfoMat Default information matrix for pose-pose
        %   measurement in a compact vector form
        DefaultCompactPoseInfoMat
        
        %DefaultCompactPointInfoMat Default information matrix for pose-point
        %   measurement in a compact vector form
        DefaultCompactPointInfoMat
         
    end

    properties (SetAccess = ?nav.algs.internal.InternalAccess)
        %NumNodes Number of nodes in pose graph
        %
        %   Default: 1
        NumNodes

        %NumEdges Number of Edges in pose graph
        %
        %   Default: 0
        NumEdges

        %NumLoopClosureEdges Number of loop closure edges in pose graph
        %
        %   Default: 0
        NumLoopClosureEdges
    end

    properties (Dependent, SetAccess = ?nav.algs.internal.InternalAccess)

        %LoopClosureEdgeIDs Loop closure edge IDs
        %
        %   Default: [1x0 double]
        LoopClosureEdgeIDs
        
        %LandmarkNodeIDs A vector of landmark node IDs in the pose graph
        %
        %   Default: [1x0 double]
        LandmarkNodeIDs

    end

    properties (Access = ?nav.algs.internal.InternalAccess)

        %NodeEstimates
        NodeEstimates
        
        %NodeMap
        NodeMap
        
        %NodeDims
        NodeDims
        
        %IsLandmarkNode A vector of 0's or 1's
        IsLandmarkNode

        %EdgeNodePairs
        EdgeNodePairs

        %LoopClosureEdgeNodePairs
        LoopClosureEdgeNodePairs

        % edge constraints

        %EdgeMeasurements
        EdgeMeasurements

        %EdgeInfoMatrices
        EdgeInfoMatrices

        %LoopClosureEdgeIDsInternal
        LoopClosureEdgeIDsInternal

    end


    properties (Access = protected)
        MaxNumEdges
        MaxNumNodes

    end

    methods (Abstract, Access = protected)
        defaultObj = getDefaultObject(obj,maxNumEdges,maxNumNodes)
        h = drawEdge(~, ax, n1, n2, varargin)
        h = drawNode(~, ax, n, varargin)
        drawID(~, ax, loc, str, varargin)
    end
    
    
    methods (Hidden)
                
        function nodePairs = edges(obj, varargin)
            %edges Return node pairs that correspond to edges with specified EDGEIDs.
            %   EDGES will be removed in future releases. Use edgeNodePairs
            %   instead.
            nodePairs = obj.edgeNodePairs(varargin{:});
        end
        
        function nodeEsts = nodes(obj, varargin)
            %nodes Returns current node estimates in pose graph
            %   NODES will be removed in future releases. Use nodeEstimates
            %   instead.
            nodeEsts = obj.nodeEstimates(varargin{:});
        end
    end


    methods
        function obj = PoseGraphBase(varargin)
            %PoseGraphBase Constructor
                        
            names = {'MaxNumEdges', 'MaxNumNodes'};
            defaultValues = {uint32(0), uint32(0)};
            
            % parse name-value pairs
            parser = robotics.core.internal.NameValueParser(names, defaultValues);
            parse(parser, varargin{:});
            
            if coder.target('MATLAB')
                maxNumE = 1;
                maxNumN = 1;
            else
                maxNumE = parameterValue(parser, 'MaxNumEdges');
                maxNumN = parameterValue(parser, 'MaxNumNodes');
                coder.internal.assert(~isnan(maxNumE) || ~isnan(maxNumN), 'nav:navalgs:posegraph:MaxNumEdgesAndNodesRequiredForCodegen');

                coder.internal.assert(isnumeric(maxNumE) && isnumeric(maxNumN) && ...
                                      isscalar(maxNumE) && isscalar(maxNumN) && ...
                                      ~isempty(maxNumE) && ~isempty(maxNumN) && ...
                                      maxNumE>0 && maxNumN>0,  'nav:navalgs:posegraph:InvalidMaxNumEdgesOrNodes');
            end
            obj.MaxNumEdges = maxNumE;
            obj.MaxNumNodes = maxNumN;

            obj.NumEdges = 0;
            obj.NumNodes = 1;
            obj.NumLoopClosureEdges = 0;

            obj.NodeEstimates = robotics.core.internal.BlockMatrix(obj.MaxNumNodes, 1, obj.TformSize);
            obj.NodeEstimates.replaceBlock(1,1, eye(obj.TformSize));
            
            obj.NodeMap = zeros(obj.MaxNumNodes,1);
            obj.NodeDims = zeros(obj.MaxNumNodes,1);
            obj.IsLandmarkNode = false(obj.MaxNumNodes,1);
            obj.NodeMap(1) = 1;
            obj.NodeDims(1) = obj.PoseDeltaLength;
            
            obj.EdgeNodePairs = zeros(obj.MaxNumEdges, 2);
            obj.LoopClosureEdgeNodePairs = zeros(obj.MaxNumEdges, 2);
            obj.LoopClosureEdgeIDsInternal = zeros(1, obj.MaxNumEdges);
            obj.EdgeMeasurements = robotics.core.internal.BlockMatrix(obj.MaxNumEdges, 1, obj.TformSize);
            obj.EdgeInfoMatrices = robotics.core.internal.BlockMatrix(obj.MaxNumEdges, 1, obj.PoseInfoMatSize);
        end

        function newObj = copy(obj)
            %copy Creates a copy of the object
            %   NEWOBJ = COPY(OBJ) creates a deep copy of the
            %   pose graph object with the same properties.

            newObj = obj.getDefaultObject(obj.MaxNumEdges, obj.MaxNumNodes);

            newObj.EdgeNodePairs = obj.EdgeNodePairs;
            newObj.LoopClosureEdgeNodePairs = obj.LoopClosureEdgeNodePairs;
            newObj.LoopClosureEdgeIDsInternal = obj.LoopClosureEdgeIDsInternal;

            newObj.NodeEstimates = copy(obj.NodeEstimates);
            newObj.EdgeMeasurements = copy(obj.EdgeMeasurements);
            newObj.EdgeInfoMatrices = copy(obj.EdgeInfoMatrices);


            newObj.NumNodes = obj.NumNodes;
            newObj.NodeMap = obj.NodeMap;
            newObj.NodeDims = obj.NodeDims;
            newObj.IsLandmarkNode = obj.IsLandmarkNode;
            
            newObj.NumEdges = obj.NumEdges;
            newObj.NumLoopClosureEdges = obj.NumLoopClosureEdges;

            if coder.target('MATLAB')
                newObj.MaxNumEdges = obj.MaxNumEdges;
                newObj.MaxNumNodes = obj.MaxNumNodes;
            end
        end

        function updatePoseGraph(obj,newObj)
            %updatePoseGraph Updates the pose graph with optimization results
            obj.NodeEstimates.updateBlockMatrix(newObj.NodeEstimates);
            obj.EdgeInfoMatrices.updateBlockMatrix(newObj.EdgeInfoMatrices);
            obj.EdgeMeasurements.updateBlockMatrix(newObj.EdgeMeasurements);
        end
        
        function [nodePair, edgeId] = addPointLandmark(obj, varargin)
            %addPointLandmark Add point landmark to pose graph
            %   addPointLandmark(POSEGRAPH, MEASUREMENT) creates a new
            %   landmark node and connects it to the last pose node in 
            %   POSEGRAPH with a new edge. Default value of information  
            %   matrix is used while creating the new edge constraint.
            %
            %   MEASUREMENT  - The landmark observation (point measurement)
            %                  with respect to a pose node in POSEGRAPH
            %
            %   addPointLandmark(POSEGRAPH, MEASUREMENT, INFOMAT) specifies
            %   the information matrix associated with the measurement in
            %   INFOMAT.
            %
            %   INFOMAT - The information matrix corresponding to MEASUREMENT.
            %
            %             Default: Upper triangular matrix of eye(poseLength)
            %             in a compact vector form
            %
            %   addPointLandmark(POSEGRAPH, MEASUREMENT, INFOMAT, FROMNODEID)
            %   adds new landmark node and connects it to node FROMNODEID, 
            %   which must already exist in POSEGRAPH and is a pose node.
            %
            %   addPointLandmark(POSEGRAPH, MEASUREMENT, INFOMAT, FROMNODEID,
            %   TONODEID) adds a new edge between node FROMNODEID and
            %   node TONODEID. Node FROMNODEID must already exist in
            %   POSEGRAPH and is a pose node. For TONODEID, node TONODEID
            %   either already exists in POSEGRAPH and is a landmark node,
            %   or TONODEID is equal to the number of nodes in POSEGRAPH
            %   plus one, in the latter case, a new landmark node is added
            %   accordingly.
            %
            %   [EDGENODEPAIR, EDGEID] = addPointLandmark(POSEGRAPH, ___) 
            %   returns EDGENODEPAIR, the node pair corresponding to the newly 
            %   added edge (a 2-vector composed of two node IDs), and
            %   the unique edge ID, EDGEID, for the new edge (a positive 
            %   integer).

            [measurement, measurementInfoMat, fromNodeId, toNodeId, isNewLandmark] = validateAddPointLandmarkInputs(obj, varargin{:});

            nodePair = [fromNodeId, toNodeId];
            Omega = measurementInfoMat;
            Trel = eye(obj.TformSize);
            n = obj.TformSize(2);
            Trel(1:obj.PointLength, n) = measurement(:);
            
            obj.EdgeNodePairs(obj.NumEdges+1, :) = nodePair;
            
            obj.EdgeMeasurements.replaceBlock(obj.NumEdges+1, 1, Trel);
            obj.EdgeInfoMatrices.replaceBlock(obj.NumEdges+1, 1, Omega);

            obj.NumEdges = obj.NumEdges + 1;
            edgeId = obj.NumEdges;
            
            if isNewLandmark
                % provide an initial node estimate for the new point landmark node
                Tx = obj.NodeEstimates.extractBlock(fromNodeId,1)*Trel;
                T = eye(obj.TformSize);
                T(1:obj.PointLength, n) = Tx(1:obj.PointLength, n);
                obj.NodeEstimates.replaceBlock(obj.NumNodes+1, 1, T);

                obj.NumNodes = obj.NumNodes + 1;

                % meta data related to the new node
                obj.NodeDims(obj.NumNodes) = obj.PointLength;
                obj.NodeMap(obj.NumNodes) = obj.NodeMap(obj.NumNodes-1) + obj.NodeDims(obj.NumNodes-1);
                obj.IsLandmarkNode(obj.NumNodes) = true;
            end
            
            if coder.target('MATLAB')
                obj.MaxNumNodes = obj.NumNodes;
                obj.MaxNumEdges = obj.NumEdges;
            end
        end


        function [nodePair, edgeId] = addRelativePose(obj, varargin)
            %addRelativePose Add relative pose to pose graph
            %   addRelativePose(POSEGRAPH, MEASUREMENT) creates a new pose node and
            %   connects it to the last pose node in POSEGRAPH with a
            %   new edge. Default value of information matrix is used while
            %   creating a new edge constraint.
            %
            %   MEASUREMENT  - Relative pose measurement with respect to
            %                  the last pose node in POSEGRAPH 
            %
            %   addRelativePose(POSEGRAPH, MEASUREMENT, INFOMAT) specifies the
            %   information matrix associated with the measurement.
            %
            %   INFOMAT - Information matrix corresponding to MEASUREMENT.
            %
            %             Default: Upper triangular matrix of eye(poseLength)
            %             in a compact vector form
            %
            %   addRelativePose(POSEGRAPH, MEASUREMENT, INFOMAT, FROMNODEID)
            %   adds new node and connects it to node FROMNODEID, which
            %   must already exist in POSEGRAPH and is a pose node.
            %
            %   addRelativePose(POSEGRAPH, MEASUREMENT, INFOMAT, FROMNODEID,
            %   TONODEID) adds a new edge between node FROMNODEID and
            %   node TONODEID. At least one of the node IDs has to exist in
            %   POSEGRAPH. In such case, the nonexistent node ID must equal
            %   the number of nodes in POSEGRAPH plus one, and a new node
            %   will be added accordingly.
            %
            %   [EDGENODEPAIR, EDGEID] = addRelativePose(POSEGRAPH, ___) returns
            %   EDGENODEPAIR, the node pair corresponding to the newly 
            %   added edge (a 2-vector composed of two node IDs), and
            %   the unique edge ID, EDGEID, for the new edge (a positive 
            %   integer).
            
            [relPose, infoMat, fromNodeId, toNodeId, needNewPoseNode, constraintNeedsInversion] = obj.validateAddRelativePoseInputs(varargin{:});
            
            nodePair = [fromNodeId, toNodeId];

            id = obj.findEdgeID(nodePair);
            isLoopClosure = false;
            if ~needNewPoseNode && (isempty(id) || ismember(id(1), obj.LoopClosureEdgeIDsInternal))
                isLoopClosure = true;
            end
            
            Trel = obj.poseToTform(relPose);
            if any(isnan(Trel),'all')
                nav.algs.internal.error('nav:navalgs', 'posegraph:InvalidRelativePose');
            end
            Omega = infoMat;
            

            obj.EdgeNodePairs(obj.NumEdges+1, :) = nodePair;

            if constraintNeedsInversion % this flag should only be true for non-loop-closure edges
                [Trel, Omega] = invertConstraint(obj, Trel, Omega);
            end
            obj.EdgeMeasurements.replaceBlock(obj.NumEdges+1, 1, Trel);
            obj.EdgeInfoMatrices.replaceBlock(obj.NumEdges+1, 1, Omega);

            obj.NumEdges = obj.NumEdges + 1;
            edgeId = obj.NumEdges;

            if needNewPoseNode % a new node to be added for a new non-LC edge
                T = obj.NodeEstimates.extractBlock(fromNodeId,1)*Trel;
                obj.NodeEstimates.replaceBlock(obj.NumNodes+1, 1, T);

                obj.NumNodes = obj.NumNodes + 1;

                % meta data related to the new node
                obj.NodeDims(obj.NumNodes) = obj.PoseDeltaLength;
                obj.NodeMap(obj.NumNodes) = obj.NodeMap(obj.NumNodes-1) + obj.NodeDims(obj.NumNodes-1);
                obj.IsLandmarkNode(obj.NumNodes) = false;
            else % no new node will be added for a new LC edge
                if isLoopClosure
                    obj.LoopClosureEdgeNodePairs(obj.NumLoopClosureEdges + 1, :) = nodePair;
                    obj.LoopClosureEdgeIDsInternal(1, obj.NumLoopClosureEdges + 1) = edgeId;
                    obj.NumLoopClosureEdges = obj.NumLoopClosureEdges + 1;
                end
                
            end


            if coder.target('MATLAB')
                obj.MaxNumNodes = obj.NumNodes;
                obj.MaxNumEdges = obj.NumEdges;
            end
            
        end


        function nodePairs = edgeNodePairs(obj, edgeIds)
            %edgeNodePairs Return the node pairs that form the designated edges
            %   NODEPAIRS = edgeNodePairs(POSEGRAPH) returns the list of node
            %   pairs associated with each individual edge that presents in
            %   POSEGRAPH. NODEPAIRS is an n-by-2 matrix, where n is the
            %   number of edges in POSEGRAPH.
            %   
            %   NODEPAIRS = edgeNodePairs(POSEGRAPH, EDGEIDS) returns node pairs
            %   corresponding to the specified EDGEIDS. EDGEIDS is a vector
            %   of positive integers. Edges with different IDs may have the
            %   same node pairs.
            if nargin > 1
                validateattributes(edgeIds, {'numeric'},{'nonnan', 'finite', 'integer', 'positive', 'nonempty', 'vector', '<=', obj.NumEdges}, 'edgeNodePairs', 'edgeIds');
                L = length(edgeIds);
                nodePairs = zeros(L,2);
                for i = 1:L
                    nodePairs(i,:) = obj.EdgeNodePairs(edgeIds(i),:);
                end
            else
                nodePairs = obj.EdgeNodePairs(1:obj.NumEdges, :);
            end
        end

        function [measurements, measurementInfoMats] = edgeConstraints(obj, edgeIds)
            %edgeConstraints Returns edge constraints in pose graph
            %   [MEASUREMENTS, MEASUREMENTINFOMATS] = edgeConstraints(POSEGRAPH)
            %   returns all the edge constraints. The edge constraints are 
            %   returned in two parts:
            %
            %   MEASUREMENTS        - Relative pose measurements (for
            %                         pose-pose edges) OR landmark
            %                         measurements (for pose-point edges)
            %   MEASUREMENTINFOMATS - Information matrices corresponding
            %                         to MEASUREMENTS in compact form
            %
            %   [MEASUREMENTS, MEASUREMENTINFOMATS] = edgeConstraints(POSEGRAPH, EDGEIDS)
            %   returns the edge constraints corresponding to specified
            %   edges with the specified EDGEIDS. EDGEIDS is a vector of
            %   positive integers.
            if nargin > 1
                validateattributes(edgeIds, {'numeric'},{'nonnan', 'finite', 'integer', 'positive', 'nonempty', 'vector', '<=', obj.NumEdges}, 'edgeConstraints', 'edgeIds');
                L = length(edgeIds);
            else
                L = obj.NumEdges;
                edgeIds = 1:L;
            end

            measurements = zeros(L, obj.PoseLength);

            for i = 1:L
                nodePair = obj.EdgeNodePairs(edgeIds(i),:);
                if obj.IsLandmarkNode(nodePair(2))
                    measurements(i,:) = obj.tformToPoint(obj.EdgeMeasurements.extractBlock(edgeIds(i), 1));
                else
                    measurements(i,:) = obj.tformToPose(obj.EdgeMeasurements.extractBlock(edgeIds(i), 1));
                end
            end

            if nargout > 1
                measurementInfoMats = zeros(L, obj.CompactPoseInfoMatLength);
                for i = 1:L
                    measurementInfoMats(i,:) = obj.serializeInformationMatrix(obj.EdgeInfoMatrices.extractBlock(edgeIds(i), 1));
                end
            end

        end

        function nodeEsts = nodeEstimates(obj, nodeIds)
            %nodeEstimates Returns current node estimates in pose graph
            %   NODEESTS = nodeEstimates(POSEGRAPH) returns the estimated 
            %   states for all the nodes in POSEGRAPH. A node estimate
            %   could be either for a pose or for a position corresponding 
            %   to a point landmark (both with respect to global reference frame).
            %
            %   NODEESTS = nodeEstimates(POSEGRAPH, NODEIDS) returns the
            %   current estimated node states for the specified NODEIDS. 
            %   NODEIDS is a vector of positive integers.

            if nargin > 1
                validateattributes(nodeIds, {'numeric'},{'nonnan', 'finite', 'integer', 'positive', 'nonempty', 'vector', '<=', obj.NumNodes}, 'nodeEstimates', 'nodeIds');
                L = length(nodeIds);
            else
                L = obj.NumNodes;
                nodeIds = 1:L;
            end
            nodeEsts = zeros(L, obj.PoseLength);

            for i = 1:L
                if obj.IsLandmarkNode(nodeIds(i))
                    nodeEsts(i,:) = obj.tformToPoint(obj.NodeEstimates.extractBlock(nodeIds(i), 1));
                else
                    nodeEsts(i,:) = obj.tformToPose(obj.NodeEstimates.extractBlock(nodeIds(i), 1));
                end
            end
        end

        function removeEdges(obj, edgeIds)
            %removeEdges Removes edges from pose graph
            %   removeEdges(POSEGRAPH, EDGEIDS) removes edges from
            %   POSEGRAPH corresponding to EDGEIDS. Only loop closure edges
            %   can be removed.

            validateattributes(edgeIds, {'numeric'},{'nonnan', 'finite', 'integer', 'positive', 'nonempty', 'vector', '<=', obj.NumEdges}, 'removeEdges', 'edgeIds');
            edgeIdsUnique = sort(unique(edgeIds), 'descend');

            lcIds = obj.LoopClosureEdgeIDs;

            li = ismember(edgeIdsUnique, lcIds);
            if all(li)
                % removing edges one by one
                for k = 1:numel(edgeIdsUnique)
                    obj.removeEdgeInternal(edgeIdsUnique(k));
                end
            else
                invalidIds = edgeIdsUnique(~li);
                % num2str doesn't support codegen, so using
                % convertNumToString.
                nav.algs.internal.error('nav:navalgs', 'posegraph:CanOnlyRemoveLoopClosureEdges', robotics.core.internal.num2str(invalidIds));
            end
        end

        function edgeID = findEdgeID(obj, nodePair)
            %findEdgeID Find edge ID for a given node pair
            %   ID = findEdgeId(POSEGRAPH, NODEPAIR) returns the edge IDs
            %   corresponding to the given node pair if they exist in 
            %   POSEGRAPH. NODEPAIR is a 2-vector, each element corresponds
            %   to a node ID in POSEGRAPH. The return value, EDGEID, is a 
            %   vector that contains edge IDs. Each found edge shares the
            %   same node pair.
            %   
            validateattributes(nodePair, {'numeric'},{'nonnan', 'finite', 'integer', 'positive', 'nonempty', 'vector', 'numel', 2}, 'findEdgeID', 'nodePair');

            edgeID = find(ismember(obj.EdgeNodePairs, nodePair, 'rows'));
        end
        
        
        function residuals = edgeResidualErrors(obj)
            %edgeResidualErrors Compute edge residual errors
            %   RESVEC = edgeResidualErrors(POSEGRAPH) returns the residual
            %   error for each edge in the pose graph given the current pose 
            %   node estimates. The residual errors are ordered in RESVEC by
            %   edge IDs.
            
            nodePairs = obj.EdgeNodePairs;
            
            % NE - internal representation for node estimates
            % EI - internal representation for edge information matrices
            % EM - internal representation for edge measurements
            [NE, EI, EM ] = obj.dump();
            nodeEstimates = robotics.core.internal.BlockMatrix(NE, obj.TformSize);
            edgeMeasurements = robotics.core.internal.BlockMatrix(EM, obj.TformSize);
            edgeInfoMats = robotics.core.internal.BlockMatrix(EI, obj.PoseInfoMatSize);
            
            residuals = zeros(obj.NumEdges, 1);
            for k = 1:obj.NumEdges

                i = nodePairs(k, 1);
                j = nodePairs(k, 2);
                
                Tij = edgeMeasurements.extractBlock(k, 1);
                OmegaIn = edgeInfoMats.extractBlock(k, 1);
                                
                if obj.IsLandmarkNode(j)
                    Omega = OmegaIn(1:obj.NodeDims(j), 1:obj.NodeDims(j));
                else
                    Omega = OmegaIn;
                end
                
                Toi = nodeEstimates.extractBlock(i, 1);
                Toj = nodeEstimates.extractBlock(j, 1);
                
                residuals(k) = nav.algs.internal.PoseGraphHelpers.costBetweenTwoNodes(Toi, Toj, Tij, Omega, obj.IsLandmarkNode(j));
            end
        end

        function ax = show(obj, varargin)
            %show Plot pose graph
            %   show(POSEGRAPH) plots pose graph in MATLAB figure
            %
            %   AX = show(POSEGRAPH) returns the axes handle under which
            %   the pose graph is plotted.
            %
            %   show(___, Name, Value) provides additional options specified
            %   by one or more Name, Value pair arguments. Name must appear
            %   inside single quotes (''). You can specify several name-value
            %   pair arguments in any order as Name1, Value1, ..., NameN, ValueN:
            %
            %   'Parent' - Handle of the axes in which the pose graph
            %              is rendered
            %
            %   'IDs'    - Display of IDs on pose graph plot
            %              Possible values:
            %              'all'          : plot all node and edge IDs
            %              'nodes'        : plot all node IDs and loop
            %                               closure edge IDs.
            %              'loopclosures' : plot only loop closure edge IDs
            %                               and node IDs that loop closure
            %                               edges connect
            %              'off'          : No IDs are plotted
            %              Default: 'loopclosures'

            parser = inputParser;
            parser.StructExpand = false;

            parser.addParameter('Parent', [], ...
                                @(x)robotics.internal.validation.validateAxesUIAxesHandle(x));
            parser.addParameter('IDs', 'loopclosures', ...
                                @(x)any(validatestring(x, {'all', 'nodes', 'loopclosures', 'off'})));
            parser.parse(varargin{:});

            ph = parser.Results.Parent;
            idDispMode = parser.Results.IDs;

            if isempty(ph)
                ax = newplot;
            else
                ax = newplot(ph);
            end

            if strcmp(ax.NextPlot, 'replace') % when hold is off
                obj.resetScene(ax);
            end

            obj.plotGraph(ax, idDispMode);

        end
    end

    methods (Access = ?nav.algs.internal.InternalAccess)
        function [measurement, measurementInfoMat, fromNodeId, toNodeId, isNewLandmark] = validateAddPointLandmarkInputs(obj, varargin)
            %validateAddPointLandmarkInputs
            
            isNewLandmark = true;
            
            if nargin <= 3
                % pg.addPointLandmark(measurement)
                % pg.addPointLandmark(measurement, measurementInfoMat)
                fromNodeId = obj.NumNodes;
                while obj.IsLandmarkNode(fromNodeId) % find the latest pose node (that is not a landmark) %%
                    fromNodeId = fromNodeId - 1;
                end
                toNodeId = obj.NumNodes + 1;

            elseif nargin == 4
                % pg.addPointLandmark(measurement, measurementInfoMat, fromNodeId)
                validateattributes(varargin{3}, {'numeric'},{'scalar','nonnan', 'finite', 'integer', 'nonempty', 'positive', '<=', obj.NumNodes}, 'addPointLandmark', 'fromNodeId');
                fromNodeId = varargin{3};
                if obj.IsLandmarkNode(fromNodeId)
                    nav.algs.internal.error('nav:navalgs', 'posegraph:FromPoseToPointOnly'); %%
                end
                toNodeId = obj.NumNodes + 1;

            else
                % pg.addPointLandmark(measurement, measurementInfoMat, fromNodeId, toNodeId)
                validateattributes(varargin{3}, {'numeric'},{'nonnan', 'finite', 'integer', 'nonempty', 'positive','scalar','<=', obj.NumNodes}, 'addPointLandmark', 'fromNodeId');
                toNodeId = robotics.internal.validation.validatePositiveIntegerScalar(varargin{4}, 'addPointLandmark', 'toNodeId');
                fromNodeId = varargin{3};
                if fromNodeId == toNodeId
                    nav.algs.internal.error('nav:navalgs', 'posegraph:TwoNodeIdsCannotBeEqual');
                elseif obj.IsLandmarkNode(fromNodeId)
                    nav.algs.internal.error('nav:navalgs', 'posegraph:FromPoseToPointOnly');
                elseif (toNodeId <= obj.NumNodes) && (~obj.IsLandmarkNode(toNodeId))
                    nav.algs.internal.error('nav:navalgs', 'posegraph:FromPoseToPointOnly');
                elseif toNodeId - obj.NumNodes > 1
                    nav.algs.internal.error('nav:navalgs', 'posegraph:UnsupportedNodeIdPairs');
                else
                    if toNodeId <= obj.NumNodes
                        isNewLandmark = false;
                    end
                end
            end
            
            % extract measurement and associated information matrix
            measurement = varargin{1};
            
            if nargin == 2
                im = obj.DefaultCompactPointInfoMat; %%
            elseif isempty(varargin{2})
                im = obj.DefaultCompactPointInfoMat;
            else
                im = varargin{2};
            end

            % validate measurement and its corresponding information matrix
            validateattributes(measurement, {'numeric'},{'nonnan', 'finite', 'real', 'nonempty', 'vector', 'numel', obj.PointLength}, 'addPointLandmark', 'measurement');
            measurement = double(measurement);

            validateattributes(im, {'numeric'},{'nonnan', 'finite', 'real', 'nonempty', 'vector', 'numel', obj.CompactPointInfoMatLength}, 'addPointLandmark', 'measurementInfoMat');
            
            imPadded = nan(1, obj.CompactPoseInfoMatLength);
            imPadded(1:obj.CompactPointInfoMatLength) = im;
            measurementInfoMat = obj.deserializeInformationMatrix(imPadded);
            
            % information matrix has to be PD
            if ~robotics.core.internal.isPositiveDefinite(measurementInfoMat(1:obj.PointInfoMatSize(1), 1:obj.PointInfoMatSize(2)) )
                nav.algs.internal.error('nav:navalgs', 'posegraph:InformationMatrixNotPD');
            end
        end
        
        
        function [relPose, infoMat, fromNodeId, toNodeId, needNewPoseNode, constraintNeedsInversion] = validateAddRelativePoseInputs(obj, varargin)
            %validateAddRelativePoseInputs
            narginchk(2,5);

            constraintNeedsInversion = false;

            if nargin <= 3 % inexplicitly add a new incremental edge
                fromNodeId = obj.NumNodes;
                while obj.IsLandmarkNode(fromNodeId) % find the latest pose node (that is not a landmark)
                    fromNodeId = fromNodeId - 1;     % the first node is always a pose node
                end
                toNodeId = obj.NumNodes+1;
                needNewPoseNode = true;
            elseif nargin == 4 % branching
                validateattributes(varargin{3}, {'numeric'},{'scalar','nonnan', 'finite', 'integer', 'nonempty', 'positive', '<=', obj.NumNodes}, 'addRelativePose', 'fromNodeId');
                fromNodeId = varargin{3};
                if obj.IsLandmarkNode(fromNodeId)
                    nav.algs.internal.error('nav:navalgs', 'posegraph:FromPoseToPoseOnly');
                end
                toNodeId = obj.NumNodes+1;
                needNewPoseNode = true;
            else
                validateattributes(varargin{3}, {'numeric'},{'nonnan', 'finite', 'integer', 'nonempty', 'positive','scalar'}, 'addRelativePose', 'fromNodeId');
                validateattributes(varargin{4}, {'numeric'},{'nonnan', 'finite', 'integer', 'nonempty', 'positive','scalar'}, 'addRelativePose', 'toNodeId');
                nodeId1 = varargin{3};
                nodeId2 = varargin{4};
                if nodeId1 == nodeId2
                    nav.algs.internal.error('nav:navalgs', 'posegraph:TwoNodeIdsCannotBeEqual');
                elseif (nodeId1 <= obj.NumNodes && obj.IsLandmarkNode(nodeId1)) || (nodeId2 <= obj.NumNodes && obj.IsLandmarkNode(nodeId2))
                    nav.algs.internal.error('nav:navalgs', 'posegraph:FromPoseToPoseOnly');
                elseif (nodeId1 <= obj.NumNodes) && (nodeId2 <= obj.NumNodes)
                    % if both nodes exist but no edge exists in between, or
                    % there are already loop closure edges in between them,
                    % then this new edge to be added is a loop closure.
                    if isempty(obj.findEdgeID([nodeId1, nodeId2])) || ismember([nodeId1, nodeId2], obj.LoopClosureEdgeNodePairs, 'rows')
                        fromNodeId = nodeId1;
                        toNodeId = nodeId2;
                        needNewPoseNode = false;
                    else % otherwise this has to be an incremental multi-edge
                        fromNodeId = min(nodeId1, nodeId2);
                        toNodeId = max(nodeId1, nodeId2);
                        needNewPoseNode = false;
                        if fromNodeId ~= nodeId1
                            constraintNeedsInversion = true;
                        end
                    end
                elseif (min(nodeId1, nodeId2) <= obj.NumNodes) && (max(nodeId1, nodeId2)-obj.NumNodes == 1) % explicitly add a new incremental edge
                    fromNodeId = min(nodeId1, nodeId2);
                    toNodeId = obj.NumNodes+1;
                    needNewPoseNode = true;
                    if fromNodeId ~= nodeId1
                        constraintNeedsInversion = true;
                    end
                else
                    nav.algs.internal.error('nav:navalgs', 'posegraph:UnsupportedNodeIdPairs');
                end
            end

            rp = varargin{1};
            if nargin == 2
                im = obj.DefaultCompactPoseInfoMat;
            elseif isempty(varargin{2})
                im = obj.DefaultCompactPoseInfoMat;
            else
                im = varargin{2};
            end
            
            validateattributes(rp, {'numeric'},{'nonnan', 'finite', 'real', 'nonempty', 'vector', 'numel', obj.PoseLength}, 'addRelativePose', 'measurement');
            relPose = double(rp);
            validateattributes(im, {'numeric'},{'nonnan', 'finite', 'real', 'nonempty', 'vector', 'numel', obj.CompactPoseInfoMatLength}, 'addRelativePose', 'measurementInfoMat');
            infoMat = obj.deserializeInformationMatrix(im);

            if ~robotics.core.internal.isPositiveDefinite(infoMat)
                nav.algs.internal.error('nav:navalgs', 'posegraph:InformationMatrixNotPD');
            end
        end


        function removeEdgeInternal(obj, lcEdgeId)
            %removeEdgeInternal Remove ONE loop closure edge from pose graph
            %   This internal method is only to be invoked by removeEdges.
            %   lcEdgeId must be a VALID loop closure edge ID.

            idx = [1:lcEdgeId-1, lcEdgeId+1:obj.NumEdges];

            % update edges
            obj.EdgeNodePairs(1:(size(idx,2)+1),:) = [obj.EdgeNodePairs(idx, :); obj.EdgeNodePairs(1,:) * 0];
            [~, loc] = ismember(lcEdgeId, obj.LoopClosureEdgeIDs);
            idxl = [1:loc-1, loc+1:obj.NumLoopClosureEdges];
            if isempty(idxl) % when no loop closure edge is left in the graph
                obj.LoopClosureEdgeNodePairs = obj.LoopClosureEdgeNodePairs * 0;
            else
                obj.LoopClosureEdgeNodePairs(1:size(idxl,2)+1,:) = [obj.LoopClosureEdgeNodePairs(idxl, :); obj.EdgeNodePairs(1,:) * 0];
            end

            % update loop closure edge Ids
            [~, loc] = ismember(lcEdgeId, obj.LoopClosureEdgeIDsInternal);
            if loc < obj.NumLoopClosureEdges
                obj.LoopClosureEdgeIDsInternal(loc:obj.NumLoopClosureEdges) = [obj.LoopClosureEdgeIDsInternal(loc+1:obj.NumLoopClosureEdges)-1, 0];
            else
                obj.LoopClosureEdgeIDsInternal(obj.NumLoopClosureEdges) = 0;
            end

            % deal with the edge constraints
            obj.EdgeMeasurements.replaceBlock(lcEdgeId, 1, nan(obj.TformSize));
            obj.EdgeInfoMatrices.replaceBlock(lcEdgeId, 1, inf(obj.PoseInfoMatSize));

            P = obj.EdgeMeasurements.Matrix;
            Im = obj.EdgeInfoMatrices.Matrix;

            % For code generation, the size of the variable cannot change
            % once defined. While assignment we should specify the range of
            % elements to replace.
            EdgePosesMatrix = [reshape(P(~isnan(P)), [], obj.TformSize(2)); zeros(obj.TformSize)];
            obj.EdgeMeasurements.Matrix(1:size(EdgePosesMatrix,1),:) = EdgePosesMatrix;
            EdgeInfoMatricesMatrix = [reshape(Im(~isinf(Im)), [], obj.PoseInfoMatSize(2)); zeros(obj.PoseInfoMatSize)];
            obj.EdgeInfoMatrices.Matrix(1:size(EdgeInfoMatricesMatrix,1),:) = EdgeInfoMatricesMatrix;

            % update count
            obj.NumLoopClosureEdges = obj.NumLoopClosureEdges - 1;
            obj.NumEdges = obj.NumEdges - 1;

            if coder.target('MATLAB')
                obj.MaxNumNodes = obj.NumNodes;
                obj.MaxNumEdges = obj.NumEdges;
            end
        end

        function resetScene(~, ax)
            %resetScene
            daspect(ax, [1 1 1]);
            grid(ax, 'on');
            box(ax, 'on');
            xlabel(ax, 'X');
            ylabel(ax, 'Y');
            zlabel(ax, 'Z');
        end

        function plotGraph(obj, ax, idDisplayMode)
            %plotGraph
            hold(ax, 'on');
            nodeEstimates = obj.nodeEstimates;

            if obj.NumEdges  == 0
                return;
            end

            lcIds = obj.LoopClosureEdgeIDs;
            allEdgeIds = 1:obj.NumEdges;
            allEdgeNodePairs = obj.edgeNodePairs;
            
            trajIds = find(~ismember(allEdgeIds, lcIds) & ~obj.IsLandmarkNode(allEdgeNodePairs(:,2)) );
            trajEdgeNodePairs = obj.EdgeNodePairs(trajIds,:);
            
            landmarkEdgeNodePairs = allEdgeNodePairs(obj.IsLandmarkNode(allEdgeNodePairs(:,2)),:);

            hLandmarkEdges = [];
            hLandmarkNodes = [];
            for m = 1:size(landmarkEdgeNodePairs,1)
                n1 = nodeEstimates(landmarkEdgeNodePairs(m,1), :);
                n2 = nodeEstimates(landmarkEdgeNodePairs(m,2), :);
                
                hLandmarkEdges = obj.drawEdge(ax, hLandmarkEdges, n1, n2, '-', 'color', [0.9, 0.9, 0.9]);
                hLandmarkNodes = obj.drawNode(ax, hLandmarkNodes, n2, 'd', 'MarkerSize', 3, 'color', [0, 0.5, 0]);
            end            
            
            
            hTrajEdges = [];
            hTrajNodes = [];
            for k = 1:numel(trajIds)
                n1 = nodeEstimates(trajEdgeNodePairs(k,1), :);
                n2 = nodeEstimates(trajEdgeNodePairs(k,2), :);

                hTrajEdges = obj.drawEdge(ax, hTrajEdges, n1, n2, 'b-');
                hTrajNodes = obj.drawEdge(ax, hTrajNodes, n1, n2, 'b.');
            end
            


            if strcmp(idDisplayMode, 'all') || strcmp(idDisplayMode, 'nodes')
                nIds = (1:obj.NumNodes)';
                obj.drawID(ax, nodeEstimates, num2str(nIds), 'Color', 'k', 'FontWeight', 'bold');
            end

            if strcmp(idDisplayMode, 'all')

                for i = 1:numel(trajIds)
                    n1 = nodeEstimates(trajEdgeNodePairs(i,1), :);
                    n2 = nodeEstimates(trajEdgeNodePairs(i,2), :);
                    obj.drawID(ax, 0.5*(n1+n2), num2str(trajIds(i)), 'Color', 'b');
                end
            end

            hLoops = [];
            if ~isempty(lcIds)
                lcEdges = obj.edgeNodePairs(lcIds);

                for i = 1:numel(lcIds)
                    n1 = nodeEstimates(lcEdges(i,1), :);
                    n2 = nodeEstimates(lcEdges(i,2), :);
                    hLoops = obj.drawEdge(ax, hLoops, n1, n2, 'r');

                    if ~strcmp(idDisplayMode, 'off')
                        obj.drawID(ax, 0.5*(n1+n2), num2str(lcIds(i)), 'Color', 'r');
                    end
                    if strcmp(idDisplayMode, 'loopclosures')
                        obj.drawID(ax, n1, num2str(lcEdges(i,1)), 'Color', 'k', 'FontWeight', 'bold');
                        obj.drawID(ax, n2, num2str(lcEdges(i,2)), 'Color', 'k', 'FontWeight', 'bold');
                    end
                end
            end
            hold(ax, 'off');
        end

    end

    methods
        function lcIds = get.LoopClosureEdgeIDs(obj)
            %get.LoopClosureEdgeIDs
            lcIds  = obj.LoopClosureEdgeIDsInternal(1:obj.NumLoopClosureEdges);
        end
        
        function lmIds = get.LandmarkNodeIDs(obj)
            %get.LandmarkNodeIDs
            nodeIds = 1:obj.NumNodes; 
            lmIds = nodeIds(obj.IsLandmarkNode);
        end

    end

    methods (Access = ?nav.algs.internal.InternalAccess)
        
        function scaleInfoMat(obj, edgeId, scale)
            %scaleInfoMat
            
            T = scale * obj.EdgeInfoMatrices.extractBlock(edgeId, 1);
            obj.EdgeInfoMatrices.replaceBlock(edgeId, 1, T);
        end
        

        function [P, Omega, E] = dump(obj)
            %dump
            P = obj.NodeEstimates.Matrix(1:obj.NumNodes*obj.TformSize(1), :);
            Omega = obj.EdgeInfoMatrices.Matrix(1:obj.NumEdges*obj.PoseInfoMatSize(1), :);
            E = obj.EdgeMeasurements.Matrix(1:obj.NumEdges*obj.TformSize(1), :);
        end

        function quickUpdateNodeEstimates(obj, P)
            %quickUpdateNodeEstimates Update the estimated states of the 
            %   existing nodes in the pose graph. No change is made to
            %   the graph topology.
            obj.NodeEstimates.Matrix(1:size(P,1),:) = P;
        end

        function updateNodeEstimates(obj, estimates)
            %updateNodeEstimates Overwrite the node estimates in pose graph
            %    with a set of given estimates. No change is made to the 
            %    graph topology.
            assert(size(estimates,1) == obj.NumNodes);
            for i = 1:size(estimates,1)
                estimate = estimates(i,:);
                T = obj.pToTform(estimate);
                obj.NodeEstimates.replaceBlock(i, 1, T);
            end
        end

        function updateEdgeConstraint(obj, edgeId, measurement, measurementInfoMat)
            %updateEdgeConstraint Update a particular edge constraint
            %   Expecting valid edgeId, measurement vector and
            %   information matrix.

            if isempty(measurement)
                Trel = obj.EdgeMeasurements.extractBlock(edgeId, 1);
            elseif isvector(measurement)
                Trel = obj.pToTform(measurement);
            else
                Trel = measurement;
            end

            if nargin < 4 || isempty(measurementInfoMat)
                omega = obj.EdgeInfoMatrices.extractBlock(edgeId, 1);
            elseif isvector(measurementInfoMat)
                omega = obj.deserializeInformationMatrix(measurementInfoMat);
            else
                omega = measurementInfoMat;
            end

            obj.EdgeMeasurements.replaceBlock(edgeId, 1, Trel);
            obj.EdgeInfoMatrices.replaceBlock(edgeId, 1, omega);
        end


        function neighboringEdgeNodePairs = findEdgeNodePairs(obj, nodeId)
            %findEdgeNodePairs Find all edges associated with the given node in the
            %   current pose graph and return the node pairs
            %   EDGENODEPAIRS = findEdges(POSEGRAPH, NODEID) returns the
            %   node pairs corresponding to all the edges that node with
            %   NODEID is involved.

            idx = find(obj.EdgeNodePairs == nodeId);
            edgeIds = mod(idx, obj.NumEdges);
            edgeIds(edgeIds == 0) = obj.NumEdges;
            neighboringEdgeNodePairs = obj.EdgeNodePairs(edgeIds,:);
        end

        function clear(obj)
            %clear Return the pose graph to pristine state
            obj.NumNodes = 1;
            obj.NumEdges = 0;
            obj.NumLoopClosureEdges = 0;
            
            if coder.target('MATLAB')
                obj.MaxNumEdges = 1;
                obj.MaxNumNodes = 1;
            end
                        
            obj.NodeEstimates = robotics.core.internal.BlockMatrix(obj.MaxNumNodes, 1, obj.TformSize);
            obj.NodeEstimates.replaceBlock(1,1, eye(obj.TformSize));
            
            obj.NodeMap = zeros(obj.MaxNumNodes,1);
            obj.NodeDims = zeros(obj.MaxNumNodes,1);
            obj.IsLandmarkNode = false(obj.MaxNumNodes,1);
            obj.NodeMap(1) = 1;
            obj.NodeDims(1) = obj.PoseDeltaLength;
            
            obj.EdgeNodePairs = zeros(obj.MaxNumEdges, 2);
            obj.LoopClosureEdgeNodePairs = zeros(obj.MaxNumEdges, 2);
            obj.LoopClosureEdgeIDsInternal = zeros(1, obj.MaxNumEdges);
            obj.EdgeMeasurements = robotics.core.internal.BlockMatrix(obj.MaxNumEdges, 1, obj.TformSize);
            obj.EdgeInfoMatrices = robotics.core.internal.BlockMatrix(obj.MaxNumEdges, 1, obj.PoseInfoMatSize);

        end

        function clearAfter(obj, nodeId)
            %clearAfter Clear the pose graph after NODEID (including NODEID)
            %   This method is intended to be used with the lidar SLAM app
            %   only.

            assert(nodeId > 1 && nodeId <= obj.NumNodes);
            
            % first need to figure out which edges to keep
            edgeMask = all(ismember(obj.EdgeNodePairs, 1:nodeId-1), 2);
            
            % extract all the nodes and edge information in current pose
            % graph
            [measurements, measurementInfoMats] = obj.edgeConstraints;
            nodePairs = obj.edgeNodePairs;
            isLandmarkNode = obj.IsLandmarkNode;
            
            measurementsRaw = copy(obj.EdgeMeasurements);
            infoMatsRaw = copy(obj.EdgeInfoMatrices);
            nodeEstimatesRaw = copy(obj.NodeEstimates);
            
            % clear pose graph
            obj.clear();
            
            % rebuild graph topology and meta data through the official
            % API, so we do not need manipulate the internal properties one
            % by one.
            edgeMap = zeros(size(nodePairs, 1), 1);
            for k = 1:size(nodePairs, 1)
                if edgeMask(k)
                    nodePair = nodePairs(k,:);
                    if ~isLandmarkNode(nodePair(2))
                        obj.addRelativePose(measurements(k,:), measurementInfoMats(k,:), nodePair(1), nodePair(2));
                    else
                        obj.addPointLandmark(measurements(k, 1:obj.PointLength),...
                            measurementInfoMats(k, 1:obj.CompactPointInfoMatLength), nodePair(1), nodePair(2));
                    end
                    edgeMap(obj.NumEdges) = k;
                end
            end
            
            % override node states and edge constraints to avoid numeric
            % issues.
            for i = 1:obj.NumEdges
                obj.EdgeMeasurements.replaceBlock(i, 1, measurementsRaw.extractBlock(edgeMap(i), 1) );
                obj.EdgeInfoMatrices.replaceBlock(i, 1, infoMatsRaw.extractBlock(edgeMap(i), 1) );
            end
            
            for i = 1:obj.NumNodes
                obj.NodeEstimates.replaceBlock(i, 1, nodeEstimatesRaw.extractBlock(i, 1) );
            end

        end
        
        function populateFromStruct(obj, s)
            %populateFromStruct To be used by loadobj ONLY for pose graph
            %   objects saved in .mat file before 21a
            
            % note that all base class props are lost in the auto-converted
            % struct.

            % obj is assumed to be in initial clean state.
            
            edgeNodePairs = s.Edges;
            edgeMeasurements = s.EdgePoses;
            edgeInfoMatrices = s.EdgeInfoMatrices;
            nodeEstimates = s.Nodes;
            
            % establish graph connection with dummy edge constraints
            edgeIds = zeros(1,s.NumEdges);
            for i = 1:s.NumEdges
                infoMat = obj.serializeInformationMatrix(eye(obj.CompactPointInfoMatLength));
                [~, id] = obj.addRelativePose(obj.zeroPose, infoMat, edgeNodePairs(i,1), edgeNodePairs(i,2));
                edgeIds(i) = id;
            end
            
            % update edge constraints directly using the data from the
            % struct to avoid any precision issues
            for i = 1:obj.NumEdges
                obj.updateEdgeConstraint(edgeIds(i), edgeMeasurements.extractBlock(i,1), edgeInfoMatrices.extractBlock(i,1));
            end
            
            % reset the node estimates
            obj.quickUpdateNodeEstimates(nodeEstimates.Matrix);
            
            % update the max numbers
            obj.MaxNumEdges = s.MaxNumEdges;
            obj.MaxNumNodes = s.MaxNumNodes;
        end
    end

    methods(Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % Let the coder know about non-tunable parameters
            props = {'MaxNumEdges', 'MaxNumNodes'};
        end
    end
end
