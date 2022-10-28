classdef factorPoseSE3AndPoint3 < nav.algs.internal.FactorGaussianNoiseModel
%FACTORPOSESE3ANDPOINT3 Creates a factor object that relates an SE(3) pose
%  and a 3D point.
%
%   F = FACTORPOSESE3ANDPOINT3(ID) returns a factorPoseSE3AndPoint3 object,
%   F, with the node identification number set to ID. The measurement
%   represents a relative position [dx,dy,dz]. By default the measurement
%   is set to [0,0,0] and the corresponding information matrix is set to
%   eye(3).
%
%   F = FACTORPOSESE3ANDPOINT3(...,Name=Value,...) returns a
%   factorPoseSE3AndPoint3 object, F, with each specified property name set
%   to the specified value. You can specify additional name-value pair
%   arguments in any order as (Name1=Value1,...,NameN=ValueN).
%
%   FACTORPOSESE3ANDPOINT3 methods:
%       nodeType          - Retrieve the node type for a specified node ID
%
%   FACTORPOSESE3ANDPOINT3 properties:
%       NodeID            - Node ID number in factor graph
%       Measurement       - Measured relative position [dx,dy,dz]
%       Information       - Information matrix associated with measurement
%
%   See also factorGraph.
    
%   Copyright 2021 The MathWorks, Inc.

    properties (Hidden, Constant)
        FactorType = "SE3_Point3_F";
    end
    
    methods
        function obj = factorPoseSE3AndPoint3(ids, varargin)
            %FACTORPOSESE3ANDPOINT3 Constructor
            narginchk(1, Inf);
            obj@nav.algs.internal.FactorGaussianNoiseModel(ids, 2, 'factorPoseSE3AndPoint3', zeros(1,3), eye(3), varargin{:});
        end
    end

    methods (Access=protected)
        function type = nodeTypeImpl(obj, id)
            if find(obj.NodeID == id) == 1
                type = nav.internal.factorgraph.NodeTypes.SE3;
            else
                type = nav.internal.factorgraph.NodeTypes.Point3;
            end
        end
    end
end

