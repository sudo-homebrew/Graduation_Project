classdef BlockInserter2 < handle
%This class is for internal use only. It may be removed in the future.

%BLOCKINSERTER2 Utility class that forms the sparse Hessian matrix and
%   gradient vector by block for a particular pose graph data structure. 
%   The block size can be different and the block does not need to be square.
%   This is an evolved version of nav.algs.internal.BlockInserter.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

    properties
        
        Gradient
        NodeDims
        NodeMap
        
        NumNodes
        NumEdges
        
        HessianCSC % sparse Hessian matrix in compressed sparse column (CSC) format
        HessianCSCCount
    end

    methods
        function obj = BlockInserter2(numNodes, nodeMap, numEdges, nodeDims, poseDeltaLength)
            %BLOCKINSERTER2 Constructor
            %   numNodes   - Number of nodes, i.e. the number of sections in 
            %                gradient vector or along any dimension of 
            %                the (symmetric) Hessian matrix
            %   nodeMap    - A numNodes-by-1 vector that maps section index 
            %                (i.e. node ID) to matrix entry index
            %   numEdges   - Number of edges, or the number of times needed 
            %                for Hessian block insertion
            %   nodeDims   - A numNodes-by-1 matrix, the ith entry of which 
            %                corresponds to the dimension of ith node
            %   poseDeltaLengh - The length of perturbation vector for pose
            %                    node. Needed for codegen purpose.
 
            if coder.target('matlab')
                N = nodeMap(end) + nodeDims(end) - 1;
            else
                N = numNodes * poseDeltaLength; % for codegen, we have to do an overestimate on memory allocation
            end
            obj.Gradient = zeros(N, 1);
            obj.NodeDims = nodeDims;
            obj.NodeMap = nodeMap;
            
            obj.NumNodes = numNodes;
            obj.NumEdges = numEdges;
            
            maxNodeDim = max(nodeDims);
            obj.HessianCSC = zeros((4*numEdges + 1)*maxNodeDim*maxNodeDim, 3);
            obj.HessianCSCCount = 1;
        end

        function insertHessianBlock(obj, i, j, blockij)
            %insertHessianBlock Insert block (i,j) in the Hessian matrix

            rowStart = obj.NodeMap(i);
            colStart = obj.NodeMap(j);
            
            sz = size(blockij);
            %szExpected = [obj.NodeDims(i), obj.NodeDims(j)];
            %assert(all(sz == szExpected));
            
            num = numel(blockij);
            [m, n] = ind2sub(sz, 1:numel(blockij));

            obj.HessianCSC(obj.HessianCSCCount: obj.HessianCSCCount + num - 1, :) = [rowStart+m(:)-1, colStart+n(:)-1, blockij(:)];
            obj.HessianCSCCount = obj.HessianCSCCount + num;
        end
        
        function insertHessianFourBlocks(obj, i, j, hessii, hessij, hessji, hessjj) 
            %insertHessianFourBlocks Insert the 4 Hessian blocks that
            %   correspond to one edge (between node i and node j)
            %   in one function call.

            rowStart0 = obj.NodeMap(i) - 1;
            colStart1 = obj.NodeMap(j) - 1;
            
            rowStart1 = colStart1;
            colStart0 = rowStart0;
            
            % block ii
            dim_i = obj.NodeDims(i); 
            numEntries1 = dim_i * dim_i;
            [m1, n1] = ind2sub([dim_i, dim_i], 1:numEntries1);
            
            % block ij
            dim_j = obj.NodeDims(j);
            numEntries2 = dim_i * dim_j;
            [m2, n2] = ind2sub([dim_i, dim_j], 1:numEntries2);
            
            % block ji
            [m3, n3] = ind2sub([dim_j, dim_i], 1:numEntries2); % block ji has the same number of entries as block ij
            
            % block jj
            numEntries4 = dim_j * dim_j;
            [m4, n4] = ind2sub([dim_j, dim_j], 1:numEntries4);
            

            H = [hessii(:); hessij(:); hessji(:); hessjj(:)];
            
            totalNumEntries = numEntries1 + 2 * numEntries2 + numEntries4;
            obj.HessianCSC(obj.HessianCSCCount: obj.HessianCSCCount + totalNumEntries - 1, :) = [ [rowStart0 + m1(:), colStart0 + n1(:);
                                                                                                   rowStart0 + m2(:), colStart1 + n2(:);
                                                                                                   rowStart1 + m3(:), colStart0 + n3(:);
                                                                                                   rowStart1 + m4(:), colStart1 + n4(:)], H(:)];
            obj.HessianCSCCount = obj.HessianCSCCount + totalNumEntries;
        end
        
        function [I, J, V] = getHessianCSC(obj)
            %getHessianCSC
            csc = obj.HessianCSC(1:obj.HessianCSCCount-1, :);
            I = csc(:,1);
            J = csc(:,2);
            V = csc(:,3);
        end


        function insertGradientBlock(obj, i, blocki)
        %insertGradientBlock

            rowStart = obj.NodeMap(i);

            obj.Gradient(rowStart : rowStart+obj.NodeDims(i)-1, 1) = ...
                obj.Gradient(rowStart : rowStart+obj.NodeDims(i)-1, 1) + blocki; % +=

        end
    end
end
