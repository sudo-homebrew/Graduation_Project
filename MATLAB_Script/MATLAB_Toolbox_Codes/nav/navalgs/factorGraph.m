classdef factorGraph < nav.algs.internal.InternalAccess
%FACTORGRAPH Create a factor graph object
%   A factor graph is a bipartite graph consisting of factors connected to
%   to variable nodes. The nodes represent the unknown random variables in
%   the estimation problem, whereas the factors represent probabilistic
%   constraints on those variables, derived from measurements or prior
%   knowledge.
%
%   G = FACTORGRAPH() creates an empty factor graph object 
%
%   FACTORGRAPH properties:
%      NumNodes          - Number of nodes in factor graph
%      NumFactors        - Number of factors in factor graph
%       
%   FACTORGRAPH methods:
%      nodeIDs        - Retrieve all the node IDs in the graph
%      addFactor      - Add a factor to the graph
%      nodeState      - Get or set the state of a node
%      fixNode        - Fix or free a node during optimization
%      isNodeFixed    - Query if a node is fixed during optimization
%      hasNode        - Whether the graph has a node with specified ID
%      nodeType       - Query the type of a node
%      isConnected    - If the graph is connected
%      optimize       - Optimize factor graph
%
%   Example:
%      % Create a factor graph and add a factor
%      G = factorGraph;
%      f = factorTwoPoseSE2([2, 3]);
%      G.addFactor(f);
%
%   References:
%
%   [1] F. Dellaert, "Factor graphs and GTSAM: A hands-on introduction,"
%       Georgia Institute of Technology, Atlanta, GA, USA, Tech. Rep. 
%       GT-RIM_CP&R-2012-002, Sep. 2012. 
%
%   See also factorGraphSolverOptions, importFactorGraph, factorTwoPoseSE2,
%   factorTwoPoseSE3, factorPoseSE3Prior, factorGPS, factorIMU,
%   factorVelocity3Prior, factorIMUBiasPrior

%   Copyright 2021 The MathWorks, Inc.

    properties (Access=protected)
        GraphInternal
    end

    properties (Dependent, SetAccess=protected)
        %NumNodes Number of nodes in the factor graph
        NumNodes

        %NumFactors Number of factors in the factor graph
        NumFactors
    end

    methods % getters
        function nn = get.NumNodes(obj)
            %get.NumNodes
            nn = obj.GraphInternal.getNumNodes();
        end

        function nf = get.NumFactors(obj)
            %get.NumFactors
            nf = obj.GraphInternal.getNumFactors();
        end
    end

    
    methods
        function obj = factorGraph()
            %FACTORGRAPH Constructor
            obj.GraphInternal = nav.algs.internal.builtin.FactorGraph;
        end

        function ids = nodeIDs(obj)
            %nodeIDs Retrieve all the nodeIDs currently in the factor graph
            ids = obj.GraphInternal.getAllNodeIDs;
        end
        
        function fID = addFactor(obj, factor)
            %addFactor Add a factor to the factor graph
            %   FID = ADDFACTOR(G,F) adds a factor, F, to the factor
            %   graph, G, and returns the factor ID, FID.
            %
            %   Example:
            %      G = factorGraph;
            %      f = factorTwoPoseSE2([2,3]);
            %      G.addFactor(f);

            fID = -1;

            if isa(factor, "factorIMU")
                fID = obj.GraphInternal.addFactor(factor.createBuiltinObject);
                return;
            else
                if isa(factor, "factorTwoPoseSE2") || isa(factor, "factorVelocity3Prior") || ...
                    isa(factor, "factorIMUBiasPrior")
                    measurement = factor.Measurement;
                    Im = (factor.Information)';
                
                elseif isa(factor, "nav.algs.internal.factorPoseSE2Prior") || isa(factor, "nav.algs.internal.factorPoseSE2AndPoint2") || ...
                    isa(factor, "nav.algs.internal.factorPoseSE3AndPoint3")
                    measurement = factor.Measurement;
                    Im = (factor.Information)';

                elseif isa(factor, "factorTwoPoseSE3") || isa(factor, "factorPoseSE3Prior")
                    measurement = factor.Measurement;
                    % internal API expects SE(3) pose in [x, y, z, qx, qy, qz, qw]
                    measurement = [measurement(1:3), measurement(5:7), measurement(4)]; 
                    Im = (factor.Information)';
                    
                elseif isa(factor, "factorGPS")
                    [measurement, information]= factor.factorGraphMeasurements();
                    Im = (information)';
                else
                    return;
                end
                fID = obj.GraphInternal.addFactorGaussianNoiseModel(factor.FactorType, factor.NodeID, ...
                        measurement, Im(:));
            end
            coder.internal.errorIf(fID==-1, 'nav:navalgs:factorgraph:MismatchedNodeType');
        end

        function output = nodeState(obj, id, state)
            %nodeState Get or update the state of a node
            %   OUTPUT = NODESTATE(OBJ,ID) returns the current state of the
            %   node with the specified ID in OUTPUT.
            %
            %   OUTPUT = NODESTATE(OBJ,ID,STATE) sets the state of the
            %   node with the specified ID with STATE. If the operation is
            %   successful, STATE is also returned in OUTPUT.
            
            narginchk(2,3);
            nav.algs.internal.validation.preValidateNodeID(id, 'factorGraph/nodeState', 'id');
            id = double(id);
            if nargin == 2
                % get
                output = obj.GraphInternal.getNodeState(id);
                coder.internal.errorIf(any(isnan(output)), 'nav:navalgs:factorgraph:NodeIDNotFoundInFactorGraph')
                % The C++ layer returns the quaternion angle as the last
                % element. Put the quaternion angle first to match the
                % MATLAB format. 
                if strcmp(obj.GraphInternal.getNodeType(id), 'POSE_SE3') ...
                        && (numel(output) == 7)
                    output = [output(1:3), output(7), output(4:6)];
                end
            else
                % set
                validateattributes(state, 'numeric', ...
                    {'vector', 'real', 'nonnan', 'finite','nonempty'}, 'factorGraph/nodeState', 'state');
                status = obj.GraphInternal.setNodeState(id, state);
                coder.internal.errorIf(status == -1, 'nav:navalgs:factorgraph:NodeIDNotFoundInFactorGraph');
                coder.internal.errorIf(status == -2, 'nav:navalgs:factorgraph:MismatchedNodeStateDimension');
                % The C++ layer stores the quaternion angle as the last
                % element. Put the quaternion angle last to match the C++
                % format.
                if strcmp(obj.GraphInternal.getNodeType(id), 'POSE_SE3')
                    obj.GraphInternal.setNodeState(id, [state(1:3), state(5:7), state(4)]);
                end
                output = state;
            end
        end

        function fixNode(obj, id, flag)
            %fixNode Fix or free a node in optimization
            %   FIXNODE(OBJ,ID) fixes the node with the specified ID
            %   during optimization. 
            %
            %   FIXNODE(OBJ,ID,FLAG) fixes the node with specified ID
            %   during the optimization if FLAG is true, and free it if
            %   FLAG is false.

            narginchk(2,3);
            nav.algs.internal.validation.preValidateNodeID(id, 'factorGraph/fixNode', 'id');
            id = double(id);
            
            if nargin == 2
                status = obj.GraphInternal.fixNode(id);
            else
                validateattributes(flag, 'logical', {'scalar'}, 'factorGraph/fixNode', 'flag');
                if flag
                    status = obj.GraphInternal.fixNode(id);
                else
                    status = obj.GraphInternal.freeNode(id);
                end
            end
            coder.internal.errorIf(~status, 'nav:navalgs:factorgraph:NodeIDNotFoundInFactorGraph');
        end

        function isFixed = isNodeFixed(obj, id)
            %isNodeFixed Query if a node is fixed during optimization
            %
            %   ISFIXED = ISNODEFIXED(G,ID) returns the flag, ISFIXED,
            %   that indicates whether or not the node, ID, is fixed.
            nav.algs.internal.validation.preValidateNodeID(id, 'factorGraph/isNodeFixed', 'id');
            id = double(id);
            result = obj.GraphInternal.isNodeFixed(id);
            nodeNotFound = ~hasNode(obj, id);
            coder.internal.errorIf(nodeNotFound || (result == -1), ...
                'nav:navalgs:factorgraph:NodeIDNotFoundInFactorGraph');
            isFixed = logical(result);
        end

        function flag = hasNode(obj, id)
            %hasNode Check whether the factor graph has certain node ID
            %
            %   FLAG = HASNODE(G,ID) returns the boolean FLAG that
            %   indicates whether or not the node, ID, exists in the factor
            %   graph G.
            nav.algs.internal.validation.preValidateNodeID(id, 'factorGraph/hasNode', 'id');
            id = double(id);
            flag = obj.GraphInternal.hasNode(id);
        end

        function type = nodeType(obj, id)
            %nodeType Get the type of the node with the given ID
            %
            %   TYPE = NODETYPE(G,ID) returns the type of the node, ID, in
            %   the factor graph, G. Supported node types are:
            %       "POSE_SE3"
            %       "POSE_SE2"
            %       "VEL3"
            %       "POINT_XY"
            %       "POINT_XYZ"
            %       "IMU_BIAS"

            nav.algs.internal.validation.preValidateNodeID(id, 'factorGraph/nodeType', 'id');
            id = double(id);
            t = obj.GraphInternal.getNodeType(id);
            coder.internal.errorIf(isempty(t), 'nav:navalgs:factorgraph:NodeIDNotFoundInFactorGraph');
            type = string(t);
        end

        function flag = isConnected(obj)
            %isConnected Whether the graph is connected (only has a single
            %   connected component)
            flag = obj.GraphInternal.isConnected();
        end

        function solnInfo = optimize(obj, opts)
            %optimize Optimize the graph
            %   SOLNINFO = OPTIMIZE(OBJ, OPTS) runs graph optimization. The
            %   solver is configured with OPTS, a factorGraphSolverOptions
            %   object.
            %
            narginchk(2,2);
            validateattributes(opts, {'factorGraphSolverOptions'}, {'scalar', 'nonempty'}, 'factorGraph/optimize', 'opts');
            solnInfo = obj.GraphInternal.optimize(opts.toStruct);
        end

    end
end

