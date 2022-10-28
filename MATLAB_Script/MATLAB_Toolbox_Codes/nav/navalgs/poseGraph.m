classdef poseGraph < nav.algs.internal.PoseGraphBase
%poseGraph Create a 2D pose graph representation
%   POSEGRAPH = poseGraph() creates a 2D pose graph object
%
%   POSEGRAPH = poseGraph('MaxNumEdges', _, 'MaxNumNodes', _)
%   specifies upper bound on the Number of edges and nodes allowed in
%   POSEGRAPH when generating code. This limit is only required when
%   generating code.
%
%   POSEGRAPH properties:
%      NumNodes            - Number of nodes in pose graph
%      NumEdges            - Number of edges in pose graph
%      NumLoopClosureEdges - Number of loop closure edges in pose graph
%      LoopClosureEdgeIDs  - Loop closure edge IDs in pose graph
%      LandmarkNodeIDs     - Landmark node IDs in the pose graph
%
%   POSEGRAPH methods:
%      addRelativePose     - Add a new relative pose to pose graph
%      addPointLandmark    - Add point landmark to pose graph
%      edgeNodePairs       - Get edge node pairs from pose graph
%      nodeEstimates       - Get node estimates from pose graph
%      edgeConstraints     - Get edge constraints from pose graph
%      edgeResidualErrors  - Compute residual error for each edge
%      findEdgeID          - Find edge ID given an edge
%      removeEdges         - Remove edges from pose graph
%      copy                - Create a copy of the object
%      show                - Plot pose graph in MATLAB figure
%
%See also poseGraph3D, optimizePoseGraph

%   Copyright 2017-2020 The MathWorks, Inc.

%#codegen
    properties (Constant, Access = ?nav.algs.internal.InternalAccess)
        
        % See description of these constants in nav.algs.internal.PoseGraphBase
        
        PoseLength = 3

        PoseDeltaLength = 3
        
        PointLength = 2

        TformSize = [3 3]

        PoseInfoMatSize = [3 3]
        
        PointInfoMatSize = [2 2]

        CompactPoseInfoMatLength = 6
        
        CompactPointInfoMatLength = 3
        
        DefaultCompactPoseInfoMat = [1 0 0 1 0 1]
        
        DefaultCompactPointInfoMat = [1 0 1]
    end

    methods
        function obj = poseGraph(varargin)
            %POSEGRAPH Constructor
            obj@nav.algs.internal.PoseGraphBase(varargin{:});
        end
    end

    methods (Access = protected)
        function defaultObj = getDefaultObject(~, maxNumEdges, maxNumNodes)
            %getDefaultObject
            if coder.target('MATLAB')
                defaultObj = poseGraph;
            else
                % Codegen requires maxNumEdges and nodes to be set which
                % initializing an empty graph
                if nargin < 3
                    defaultObj = poseGraph('MaxNumEdges',8000,'MaxNumNodes',8001);
                else
                    defaultObj = poseGraph('MaxNumEdges',maxNumEdges,'MaxNumNodes',maxNumNodes);
                end
            end
        end

        function hLine = drawEdge(~, ax, hLine, n1, n2, varargin)
            %drawEdge
            if isempty(hLine)
                hLine = plot(ax, [n1(1), n2(1)], [n1(2), n2(2)], varargin{:});
            else
                hLine.XData = [hLine.XData, nan, n1(1), n2(1)];
                hLine.YData = [hLine.YData, nan, n1(2), n2(2)];
            end
        end
        
        function hPoint = drawNode(~, ax, hPoint, n, varargin)
            %drawNode
            if isempty(hPoint)
                hPoint = plot(ax, n(1), n(2), varargin{:});
            else
                hPoint.XData = [hPoint.XData, nan, n(1)];
                hPoint.YData = [hPoint.YData, nan, n(2)];
            end
        end

        function drawID(~, ax, loc, str, varargin)
            %drawID
            text(ax, loc(:,1), loc(:,2), str, varargin{:});
        end
        
    end


    methods (Access = ?nav.algs.internal.InternalAccess)

        function [tformNew, infoMatNew] = invertConstraint(~, tform, infoMat)
            %invertConstraint
            [tformNew, infoMatNew] = nav.algs.internal.PoseGraphHelpers.invertConstraintSE2(tform, infoMat);
        end
        
        function pose = tformToPose(~, T)
            %tformToPose
            pose = robotics.core.internal.SEHelpers.tformToPoseSE2(T);
        end
        
        function pointVec = tformToPoint(~, T)
            %tformToPoint
            pointVec = [T(1,3), T(2,3), nan];
        end

        function Trel = pToTform(obj, p)
            %pToTform The input argument, p, could either be a point vector
            %    or a pose vector. For facilitate codegen, p is always an
            %    1-by-PoseLength vector. It p represents a point, it is
            %    padded with nan's at the end.
            if any(isnan(p), 'all')
                Trel = obj.pointToTform(p);
            else
                Trel = obj.poseToTform(p);
            end
        end
        
        function T = poseToTform(~, pose)
            %poseToTform
            T = robotics.core.internal.SEHelpers.poseToTformSE2(pose);
        end
        
        function T = pointToTform(~, point)
            %pointToTform
            T = [1 0 point(1);
                 0 1 point(2);
                 0 0 1];
        end
        
        function poseVec = zeroPose(~)
            %zeroPose
            poseVec = [0 0 0];
        end

        function T = tforminv(~, pose)
            %tforminv Invert the transformation matrix
            T = robotics.core.internal.SEHelpers.tforminvSE2(pose);
        end

        function infoMat = deserializeInformationMatrix(obj, im)
            %deserializeInformationMatrix Compact vector to full matrix
            if isnan(im(obj.CompactPoseInfoMatLength))
                infoMat = nan(obj.PoseInfoMatSize);
                infoMat(1:obj.PointInfoMatSize(1), 1:obj.PointInfoMatSize(2)) = robotics.core.internal.SEHelpers.deserializeInformationMatrixPoint2(im);
            else
                infoMat = robotics.core.internal.SEHelpers.deserializeInformationMatrixSE2(im);
            end
        end

        function im = serializeInformationMatrix(obj, infoMat)
            %serializeInformationMatrix Full matrix to compact vector
            if any(isnan(infoMat), 'all')
                im = nan(1, obj.CompactPoseInfoMatLength);
                im(1:obj.CompactPointInfoMatLength) = ... 
                    robotics.core.internal.SEHelpers.serializeInformationMatrixPoint2(infoMat(1:obj.PointInfoMatSize(1), 1:obj.PointInfoMatSize(2)));
            else
                im = robotics.core.internal.SEHelpers.serializeInformationMatrixSE2(infoMat);
            end
        end


    end

    methods(Static, Hidden)

        function obj = loadobj(s)
            %loadobj Customized load behavior for backward compatibility
            if isstruct(s)
                obj = poseGraph();
                obj.populateFromStruct(s);
            else
                obj = s;
            end
            
        end
    end



end
