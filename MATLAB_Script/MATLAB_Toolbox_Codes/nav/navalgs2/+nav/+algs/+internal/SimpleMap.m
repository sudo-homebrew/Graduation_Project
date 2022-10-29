classdef SimpleMap < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%SIMPLEMAP nav.algs.internal.SimpleMap provides a thin wrapper for std::unordered_map<int, std::vector<double>>
%   i.e. a map between integer IDs and data vectors 
 
% Copyright 2021 The MathWorks, Inc.

%#codegen

    properties (Access=protected)
        %DataDim
        DataDim
    end

    properties (Access=private)
        %MapInternal Opaque pointer to the internal SimpleMap object
        MapInternal
    end
    
    methods
        function obj = SimpleMap(dim)
            %SIMPLEMAP Constructor
            
            validateattributes(dim, {'double'},{'scalar', 'positive', 'finite', 'integer'}, 'SimpleMap', 'Data Dimension');
            obj.MapInternal = nav.algs.internal.builtin.SimpleMap(dim);
        end
        
        function insertData(obj, id, data)
            %insertData Associate a data vector with the given id.
            %   id is expected to be of integer values
            
             obj.MapInternal.insertData(id, data);
        end

        function data = getData(obj, id)
            %getNodeData Extract data vector associated with the given id
            %   id is expected to be of integer values
            
            data = obj.MapInternal.getData(id);
        end

    end
    
        
    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
            name = 'nav.algs.internal.codegen.SimpleMap';
        end
    end
end

