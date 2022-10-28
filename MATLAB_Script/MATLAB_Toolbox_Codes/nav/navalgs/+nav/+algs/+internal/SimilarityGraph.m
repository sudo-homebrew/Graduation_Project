classdef SimilarityGraph < nav.algs.internal.PoseGraphBase
%SimilarityGraph Create a 3D similarity graph representation
%   POSEGRAPH = SimilarityGraph() creates a Sim3 Graph object
%
%   POSEGRAPH = SimilarityGraph('MaxNumEdges', _, 'MaxNumNodes', _)
%   specifies upper bound on the Number of edges and nodes allowed in
%   POSEGRAPH when generating code. This limit is only required when
%   generating code.
%
%   SIMILARITYGRAPH properties:
%      NumNodes              - Number of nodes in similarity graph
%      NumEdges              - Number of edges in similarity graph
%      NumLoopClosureEdges   - Number of loop closure edges in similarity graph
%      LoopClosureEdgeIDs    - Loop closure edge IDs in similarity graph
%
%   SIMILARITYGRAPH methods:
%      addRelativePose                 - Add a new relative pose to similarity graph
%      setEdgeScale                    - Sets the scale of added loop constraint
%      updateEdgeInformationMatrix     - Updates the edge information matrix
%      edges                           - Get edges from similarity graph
%      nodes                           - Get nodes from similarity graph
%      updateGraphNodes                - Modifies the node poses
%      edgeConstraints                 - Get edge constraints from similarity graph
%      findEdgeID                      - Find edge ID given an edge
%      removeEdges                     - Remove edges from similarity graph
%      copy                            - Create a copy of the object
%      show                            - Plot similarity graph in MATLAB figure
%
%See also poseGraph, poseGraph3D, optimizePoseGraph

%   Copyright 2019-2020 The MathWorks, Inc.

%#codegen
    properties (Constant, Access = ?nav.algs.internal.InternalAccess)
        PoseLength = 7
        
        PointLength = 3 % [x,y,z] for now.

        PoseDeltaLength = 7

        TformSize = [4 4]

        PoseInfoMatSize = [7 7]
        
        PointInfoMatSize = [3 3]

        CompactPoseInfoMatLength = 28
        
        CompactPointInfoMatLength = 6
        
        DefaultCompactPoseInfoMat = [1 0 0 0 0 0 0 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1]
        
        DefaultCompactPointInfoMat = [1 0 0 1 0 1]
        
        DefaultPoseInfoMat = eye(7)
        
    end

    methods
        function obj = SimilarityGraph(varargin)
            %POSEGRAPH3D Constructor
            obj@nav.algs.internal.PoseGraphBase(varargin{:});
        end
        

        
        function setEdgeScale(obj, edgeId, scale)
            %setEdgeScale sets the scale of an edge
            %   setEdgeScale(POSEGRAPH, EDGEID, SCALE) Sets the scale of
            %   the loop edge specified by EDGEID
            %   SCALE – scalar scale associated with the edge
            %   EDGEID – ID of the edge (as a positive integer)
            
            narginchk(3,3);
            validateattributes(scale, {'numeric'},{'scalar','nonnan', 'finite', 'nonempty', 'positive'}, 'setEdgeScale', 'scale');
            validateattributes(edgeId, {'numeric'},{'scalar','nonnan', 'finite', 'integer', 'nonempty', 'positive', '<=', obj.NumEdges}, 'setEdgeScale', 'edgeId');
            lcIds = obj.LoopClosureEdgeIDs;
            li = ismember(edgeId, lcIds);
            if li
                sform = obj.EdgeMeasurements.extractBlock(edgeId,1);
                sform(4,4) = scale;
                obj.EdgeMeasurements.replaceBlock(edgeId,1,sform);
            else
                coder.internal.error('nav:navalgs:similaritygraph:CanOnlySetScaleForLoopEdges');
            end
        end
        
        function updateEdgeInformationMatrix(obj,edgeId, infoMat)
            %updateEdgeInformationMatrix updates the edge information matrices
            %   updateEdgeInformationMatrix(POSEGRAPH, EDGEID, INFOMAT)
            %   Sets the information matrix associated with the specified edge 
            %   EDGEID – ID of the edge (as a positive integer)
            %   INFOMAT – sim3 compact information matrix 1-by-28
            
            narginchk(3,3);
            validateattributes(edgeId, {'numeric'},{'scalar','nonnan', 'finite', 'integer', 'nonempty', 'positive', '<=', obj.NumEdges}, 'updateEdgeInformationMatrix', 'edgeId');
            infoMatOut = validateEdgeCovariance(obj,infoMat);
            obj.EdgeInfoMatrices.replaceBlock(edgeId,1,infoMatOut);
        end
        
        function updateGraphNodes(obj,nodePoses, nodeIds)
            %updateGraphNodes updates the existing graph nodes 
            %    updateGraphNodes(obj,nodePoses) updates all the nodes in
            %    the graph and resets scales of all nodes to 1
            %    nodePoses - N-by-7 matrix where N is the number of nodes
            %    in the graph and resets the sepcified node scales to 1
            %    updateGraphNodes(obj,nodePoses, nodeIds) updates the
            %    specified nodes
            %    nodeIds - vector positive integer node ids
            
            if nargin > 2
                validateattributes(nodeIds, {'numeric'},{'nonnan', 'finite', 'integer', 'positive', 'nonempty', 'vector', '<=', obj.NumNodes}, 'updateGraphNodes', 'nodeIds');
                L = length(nodeIds);
            else
                L = obj.NumNodes;
                nodeIds = 1:L;
            end
            validateattributes(nodePoses, {'numeric'},{'nonnan', 'finite', 'nonempty','ncols',obj.PoseLength,'nrows',L}, 'updateGraphNodes', 'nodePoses');
            for i = 1:L
                obj.NodeEstimates.replaceBlock(nodeIds(i), 1,obj.poseToTform(nodePoses(i,:)));
            end
        end
        
        function [nodePoses,nodeScales] = nodeEstimates(obj, nodeIds)
            %nodeEstimates Returns node estimates in similarity graph
            %   NODEESTS = nodeEstimates(POSEGRAPH) returns all the current 
            %   node estimates in POSEGRAPH. While POSEGRAPH is a similarity
            %   graph, NODEESTS only contains the SE(3) part of the estimate.
            %
            %   NODEESTS = nodeEstimates(POSEGRAPH, NODEIDS) returns the
            %   current node estimates corresponding to the specified NODEIDS. 
            %   NODEIDS is a vector of positive integers.
            %   
            %   [NODEESTS, SCALES] = nodeEstimates(___) also returns the 
            %   estimated SCALES associated with the NODEESTS.

            if nargin > 1
                validateattributes(nodeIds, {'numeric'},{'nonnan', 'finite', 'integer', 'positive', 'nonempty', 'vector', '<=', obj.NumNodes}, 'nodes', 'nodeIds');
                L = length(nodeIds);
            else
                L = obj.NumNodes;
                nodeIds = 1:L;
            end
            nodePosesVec = zeros(L, obj.PoseLength+1);

            for i = 1:L
                nodePosesVec(i,:) = obj.tformToPose(obj.NodeEstimates.extractBlock(nodeIds(i), 1));
            end
            nodePoses = nodePosesVec(:,1:obj.PoseLength);
            nodeScales = nodePosesVec(:,obj.PoseLength+1);
        end
        
        function [relPoses, infoMats, scales] = edgeConstraints(obj, edgeIds)
            %edgeConstraints Returns edge constraints in similarity graph
            %   [RELPOSES, INFOMATS, SCALES] = edgeConstraints(POSEGRAPH) returns
            %   all the edge constraints. the edge constraints are returned
            %   in two parts:
            %   RELPOSES - the measured relative SE(3) poses
            %   INFOMATS - the Sim(3) information matrices in compact form (N-by-28)
            %              associated with RELPOSES and SCALES
            %   SCALES   - the measured scales associated with the edges
            %
            %   [RELPOSES, INFOMATS, SCALES] = edgeConstraints(POSEGRAPH, EDGEIDS)
            %   returns the edge constraints corresponding to specified
            %   edge IDs, as listed in EDGEIDS. EDGEIDS is a vector of
            %   positive integers.
            if nargin > 1
                validateattributes(edgeIds, {'numeric'},{'nonnan', 'finite', 'integer', 'positive', 'nonempty', 'vector', '<=', obj.NumEdges}, 'edgeConstraints', 'edgeIds');
                L = length(edgeIds);
            else
                L = obj.NumEdges;
                edgeIds = 1:L;
            end

            posesVec = zeros(L, obj.PoseLength+1);

            for i = 1:L
                posesVec(i,:) = obj.tformToPose(obj.EdgeMeasurements.extractBlock(edgeIds(i), 1));
            end
            if nargout > 1
                infoMats = zeros(L, obj.CompactPoseInfoMatLength);
                for i = 1:L
                    infoMats(i,:) = obj.serializeInformationMatrix(obj.EdgeInfoMatrices.extractBlock(edgeIds(i), 1));
                end
            end
            relPoses = posesVec(:,1:obj.PoseLength);
            scales = posesVec(:,obj.PoseLength+1);
        end
    end

    methods (Access = protected)
        function defaultObj = getDefaultObject(~,maxNumEdges,maxNumNodes)
            %getDefaultObject
            if coder.target('MATLAB')
                defaultObj = nav.algs.internal.SimilarityGraph;
            else
                % Codegen requires maxNumEdges and nodes to be set which
                % initializing an empty graph
                if nargin < 3
                    defaultObj = nav.algs.internal.SimilarityGraph('MaxNumEdges',8000,'MaxNumNodes',8001);
                else
                    defaultObj = nav.algs.internal.SimilarityGraph('MaxNumEdges',maxNumEdges,'MaxNumNodes',maxNumNodes);
                end
            end
        end

        function hLine = drawEdge(~, ax, hLine, n1, n2, varargin)
            %drawEdge
            if isempty(hLine)
                hLine = plot3(ax, [n1(1,1), n2(1,1)], [n1(1,2), n2(1,2)], [n1(1,3), n2(1,3)], varargin{:});
            else
                hLine.XData = [hLine.XData, nan, n1(1), n2(1)];
                hLine.YData = [hLine.YData, nan, n1(2), n2(2)];
                hLine.ZData = [hLine.ZData, nan, n1(3), n2(3)];
            end
        end
        
        function hPoint = drawNode(~, ax, hPoint, n, varargin) %#ok<INUSD,INUSL>
            %drawNode
        end

        function drawID(~, ax, loc, str, varargin)
            %drawID
            text(ax, loc(:,1), loc(:,2), loc(:,3), str, varargin{:});
        end

    end

    methods (Access = ?nav.algs.internal.InternalAccess)

        function [tformNew, infoMatNew] = invertConstraint(~, tform, infoMat)
            %invertConstraint
            [tformNew, infoMatNew] = nav.algs.internal.PoseGraphHelpers.invertConstraintSim3(tform, infoMat);
        end

        function pose = tformToPose(~, T)
            %tformToPose
            pose = robotics.core.internal.Sim3Helpers.sformToPoseVecSim3(T);
        end

        function T = poseToTform(~, pose)
            %poseToTform
            T = robotics.core.internal.Sim3Helpers.poseVecToSformSim3(pose);
        end
        
        function Trel = pToTform(obj, p)
            %pToTform
            Trel = obj.poseToTform(p);
        end

        function T = tforminv(~, pose)
            %tforminv
            T = robotics.core.internal.Sim3Helpers.sforminvSim3(pose);
        end

        function infoMat = deserializeInformationMatrix(~, im)
            %deserializeInformationMatrix
            infoMat = robotics.core.internal.Sim3Helpers.deserializeInformationMatrixSim3(im);
        end

        function im = serializeInformationMatrix(~, infoMat)
            %serializeInformationMatrix
            im = robotics.core.internal.Sim3Helpers.serializeInformationMatrixSim3(infoMat);
        end
        
        function tform = multiplyTform(~,T1,T2)
            tform = robotics.core.internal.Sim3Helpers.sformMultiplySim3(T1,T2);
        end
        
        function poseOut = validateEdgePose(obj,pose)
            %validateEdgePose
            
            validateattributes(pose, {'numeric'},{'nonnan', 'finite', 'real', 'nonempty', 'vector', 'numel', obj.PoseLength}, 'addRelativePose', 'pose');
            poseOut = double(pose);
        end
        
        function infoMatOut = validateEdgeCovariance(obj,infoMat)
            %validateEdgeCovariance
            
            validateattributes(infoMat, {'numeric'},{'nonnan', 'finite', 'real', 'nonempty', 'vector', 'numel', obj.CompactPoseInfoMatLength}, 'updateEdgeInformationMatrix', 'infoMat');
            infoMatOut = obj.deserializeInformationMatrix(infoMat);
            
            if ~robotics.core.internal.isPositiveDefinite(infoMatOut)
                nav.algs.internal.error('nav:navalgs', 'posegraph:InformationMatrixNotPD');
            end
        end
        
        function [relPose, infoMat, fromNodeId, toNodeId, needsNewNode, constraintNeedsInversion] = validateAddRelativePoseInputs(obj, varargin)
            %validateAddRelativePoseInputs
            narginchk(2,4);

            constraintNeedsInversion = false;

            if nargin <= 2 % inexplicit robot trajectory increment
                fromNodeId = obj.NumNodes;
                toNodeId = obj.NumNodes+1;
                needsNewNode = true;
            elseif nargin == 3 % branching
                validateattributes(varargin{2}, {'numeric'},{'scalar','nonnan', 'finite', 'integer', 'nonempty', 'positive', '<=', obj.NumNodes}, 'addRelativePose', 'fromNodeId');
                fromNodeId = varargin{2};
                toNodeId = obj.NumNodes+1;
                needsNewNode = true;
            else
                validateattributes(varargin{2}, {'numeric'},{'nonnan', 'finite', 'integer', 'nonempty', 'positive','scalar'}, 'addRelativePose', 'fromNodeId');
                validateattributes(varargin{3}, {'numeric'},{'nonnan', 'finite', 'integer', 'nonempty', 'positive','scalar'}, 'addRelativePose', 'toNodeId');
                nodeId1 = varargin{2};
                nodeId2 = varargin{3};
                if nodeId1 == nodeId2
                    nav.algs.internal.error('nav:navalgs', 'posegraph:TwoNodeIdsCannotBeEqual');
                elseif(nodeId1 <= obj.NumNodes) && (nodeId2 <= obj.NumNodes) && ( abs(nodeId1 - nodeId2)>1 || isempty(obj.findEdgeID([nodeId1, nodeId2]))) % loop closure
                    fromNodeId = nodeId1;
                    toNodeId = nodeId2;
                    needsNewNode = false;
                elseif (min(nodeId1, nodeId2) <= obj.NumNodes) && (max(nodeId1, nodeId2)-obj.NumNodes == 1) % explicit robot trajectory increment
                    fromNodeId = min(nodeId1, nodeId2);
                    toNodeId = obj.NumNodes+1;
                    needsNewNode = true;
                    if fromNodeId ~= nodeId1
                        constraintNeedsInversion = true;
                    end
                else
                    nav.algs.internal.error('nav:navalgs', 'posegraph:UnsupportedNodeIdPairs');
                end
            end

            rp = varargin{1};
            relPose = obj.validateEdgePose(rp);
            infoMat = obj.DefaultPoseInfoMat;
        end

    end

end
