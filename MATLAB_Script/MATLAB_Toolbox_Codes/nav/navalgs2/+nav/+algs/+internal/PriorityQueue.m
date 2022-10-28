classdef PriorityQueue < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%PriorityQueue nav.algs.internal.PriorityQueue provides a thin wrapper
%   of std::priority_queue in MATLAB. The data is assumed to be in
%   the form of a vector, and one of the elements can be selected as 
%   the score. The wrapper assumes "min heap", as the smallest element
%   appears as the top().
%
%   PriorityQueue(DATADIMENSION, PRIMEIDX) creates a priority queue object
%   whose data is vector of size DATADIMENSION and the queue is sorted
%   based on the PRIMEIDX-th element in the data.
%
%   PriorityQueue methods:
%     push             - Pushes node in priority queue
%     top              - Returns the data and nodeId of the node with lowest score
%     pop              - Removes node with lowest sore
%     isEmpty          - Checks if the queue is empty
%     size             - Returns the size of the priority queue
%
%   Example:
%   
%   % Creates a priority queue. Each node is a 5-vector, and the last
%   % element is used as the primary key
%   obj = nav.algs.internal.PriorityQueue(5, 5)
%
%   % Push in the first element
%   obj.push([1 2 3 4 5]);
%
%   % Push in the second element
%   obj.push([1 2 1 6 2]);
%
%   % Pop the min node. Here, data should be [1 2 1 6 2]
%   [data, idx] = obj.top();

% Copyright 2019-2021 The MathWorks, Inc.

%#codegen
    properties (Access=protected)
        %DataDim
        DataDim
    end

    properties (Access=private)
        %PQInternal Builtin priority queue data structure
        PQInternal
    end

    methods
        function obj = PriorityQueue(dim, primeIdx)
            %PriorityQueue Constructor

            validateattributes(dim, {'double'},{'scalar', 'positive', 'finite', 'integer'}, 'PriorityQueue', 'Data Dimension');
            validateattributes(primeIdx, {'double'},{'scalar', 'positive', 'finite', 'integer', '<=', dim}, 'PriorityQueue','Prime Index');

            obj.DataDim = dim;
            obj.PQInternal = nav.algs.internal.builtin.PriorityQueue(dim, primeIdx-1); % 1-index in matlab
        end

        function flag = isEmpty(obj)
            %isEmpty Returns true if queue is empty
            flag = obj.PQInternal.isEmpty();
        end

        function idx = push(obj, nodeData)
            %push Insert data to priority queue. 
            %   The assigned Id for the inserted node is returned.
            
            % Validation in each push call, even though it can be costly, but help in memory allocation. 
            validateattributes(nodeData, {'double'},{'vector', 'finite', 'ncols', obj.DataDim}, 'PriorityQueue', 'Push');
            idx = obj.PQInternal.push(nodeData) + 1;
        end

        function [topData, topId] = top(obj)
            %top Returns the data vector and ID of the node with min score in priority queue. 
            %   Returns {zeros(obj.DataDim), -1} if queue is empty.

            result = obj.PQInternal.top();
            topData = result.Data;
            topId = result.ID + 1;
        end

        function pop(obj)
            %pop Removes node with min score.
            %   If the queue is empty, this is a no-op.
            obj.PQInternal.pop();
        end

        function sz = size(obj)
            %size Returns number of nodes in the queue.
            sz = obj.PQInternal.size();
        end


    end
    
    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
            name = 'nav.algs.internal.codegen.PriorityQueue';
        end
    end
end
