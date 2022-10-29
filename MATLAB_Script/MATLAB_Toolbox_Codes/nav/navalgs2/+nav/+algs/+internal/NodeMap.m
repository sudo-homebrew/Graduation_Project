classdef NodeMap < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%NODEMAP nav.algs.internal.NodeMap provides a data structure help the
%   user keep track of relationship between different nodes. Specifically,
%   it maintains two maps internally:
%
%   1) nodeId --> parentNodeId
%   2) nodeId --> nodeData
 
% Copyright 2021 The MathWorks, Inc.

%#codegen
    properties (Access=protected)
        %DataDim
        DataDim
    end

    properties (Access=private)
        %NodeMapInternal Opaque object that represents the internal NodeMap
        NodeMapInternal
    end
    
    methods
        function obj = NodeMap(dim)
            %NODEMAP Construct an instance of this class
            
            validateattributes(dim, {'double'},{'scalar', 'positive', 'finite', 'integer'}, 'NodeMap', 'Data Dimension');
            obj.NodeMapInternal = nav.algs.internal.builtin.NodeMap(dim);
        end
        
        function nodeId = insertNode(obj, nodeData, parentId)
            %insertNode Insert a new node into node map. The newly added node
            %    becomes a child of the node with parentId.
            
             nodeId = obj.NodeMapInternal.insertNode(nodeData, parentId);
        end

        function [nodeData, parentId] = getNodeData(obj, nodeId)
            %getNodeData Extract data vector associated with the given nodeId
            %   It also returns ID of the parent node as the second output argument 
            
            result = obj.NodeMapInternal.getNodeData(nodeId);
            nodeData = result(1:end-1);
            parentId = result(end);
        end
        
        function pthData = traceBack(obj, idx)
            %traceBack Retrieve the node states along the path from node
            %   idx back to the root node
            
            pthData = obj.NodeMapInternal.traceBack(idx)';
        end
    end
    
        
    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
            name = 'nav.algs.internal.codegen.NodeMap';
        end
    end
end

