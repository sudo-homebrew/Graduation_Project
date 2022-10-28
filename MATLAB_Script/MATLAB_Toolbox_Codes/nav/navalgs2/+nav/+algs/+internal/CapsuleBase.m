classdef CapsuleBase < nav.algs.internal.InternalAccessValue
% This class is for internal use only. It may be removed in the future.

%CapsuleBase Base-class for the Capsule/Capsule3D primitives used in collision checking

%   Copyright 2020 The MathWorks, Inc.

    %#codegen

    properties
        %ID The tag used to refer to a specific obstacle
        ID
        
        %Geometry A Struct containing the geometric properties of a capsule
        Geometry
    end
        
    properties (Dependent)
        %States N-by-M list of poses assumed by the capsule
        %
        %   N - Number of states
        %   M - State dimension
        States
    end
    
    properties (Constant, Access = {?nav.algs.internal.InternalAccess, ?nav.algs.internal.CapsuleBase})
        %NumCirclePts Number of points to use in visualization
        NumCirclePts = 31;
    end
    
    properties (Constant, Abstract, Access = {?nav.algs.internal.InternalAccess, ?nav.algs.internal.CapsuleBase})
        %DefaultGeometry Default geometry if none is provided
        DefaultGeometry
        
        %Dimension Dimension of the Capsule, either 2D or 3D
        Dimension
    end
    
    properties (Access = {?nav.algs.internal.InternalAccess, ?nav.algs.internal.CapsuleBase})
        %NumStates Number of states currently taken by capsule
        NumStates = 1;
        
        %Position 
        Position
        
        %OrientationVector
        OrientationVector
    end
    
    properties (Abstract, Access = protected)
        %StatesInternal N-by-M list of poses assumed by the capsule
        %
        %   N - Number of states
        %   M - State dimension
        StatesInternal
        
        %GeometryInternal A struct containing the internal geometry properties
        GeometryInternal;
    end
    
    methods
        function obj = CapsuleBase(varargin)
            dim = obj.Dimension;
            [ID, L, R, pose] = obj.parseConstructorInput(dim, varargin{:});
            obj.ID = ID;
            obj.GeometryInternal.Length = L;
            obj.GeometryInternal.Radius = R;
            obj.GeometryInternal.FixedTransform = pose;
            obj.Position = pose(1:dim,end)';
            obj.OrientationVector = pose(1:dim,1)';
            
            if dim == 2
                states = zeros(1,3);
                coder.varsize('states', [inf 3], [1 0]);
            else
                states = [0 0 0 1 0 0 0];
                coder.varsize('states', [inf 7], [1 0]);
            end
            
            obj.StatesInternal = states;
        end
        
        function sObj = struct(obj)
            sObj = struct('ID',obj.ID,'Geometry',obj.Geometry,'StatesInternal',obj.StatesInternal,'Position',obj.Position,'OrientationVector',obj.OrientationVector);
        end
    end
    
    methods %set methods
        function geom = get.Geometry(obj)
        %get.Geometry
            geom = obj.GeometryInternal;
        end
        
        function obj = set.Geometry(obj, geom)
        %set.Geometry
            obj = obj.setGeom(geom);
        end
        
        function obj = set.States(obj, states)
        %set.States
            obj.StatesInternal = obj.convertToInternal(states);
            obj.NumStates = size(states,1);
        end
        
        function states = get.States(obj)
        %get.States
            states = obj.StatesInternal;
        end
    end
    
    methods (Access = {?nav.algs.internal.InternalAccess, ?nav.algs.internal.CapsuleBase})
        function obj = setGeom(obj, geom)
        %setGeom
            obj.GeometryInternal.Length = geom.Length;
            obj.GeometryInternal.Radius = geom.Radius;
            obj.GeometryInternal.FixedTransform = obj.validateTransform(geom.FixedTransform);
            obj.Position = obj.GeometryInternal.FixedTransform(1:obj.Dimension,end)';
            obj.OrientationVector = obj.GeometryInternal.FixedTransform(1:obj.Dimension,1)';
            obj.OrientationVector = obj.OrientationVector/norm(obj.OrientationVector);
        end
        
        function tform = validateTransform(obj, tform)
        %validateTransform Ensure H transform is 3x3
            validateattributes(tform,{'numeric'},{'nonnan','finite','real','size',obj.TSize},'setTransform','FixedTransform');
        end
    end
    
    methods (Static, Hidden)
        function nItems = getFirstNItems(array, N)
        %getFirstNItems Retrieves the first N elements of the incoming array.
        %   If N is larger than the number of stored states, the remaining states
        %   take the value of the last stored state.
            numItems = size(array,1);
            if numItems == N
                nItems = array;
            elseif numItems < N
                % Last state is held in place if number requested is
                % greater than number stored
                nItems = [array; repelem(array(end,:),N-numItems,1)];
            else
                nItems = array(1:N,:);
            end
        end
        
        function [ID, L, R, fixedTransform] = parseConstructorInput(dim, varargin)
        %parseConstructorInput Parse inputs to capsule object
            narginchk(1,5)
            if nargin == 2 && isa(varargin{:}, 'nav.algs.internal.CapsuleBase')
            %parseConstructorInput(DIM, CAPOBJ)
                if coder.target('MATLAB')
                    other = varargin{:};
                else
                    other = struct(varargin{:});
                end
                
                ID = other.ID;
                L = other.Geometry.Length;
                R = other.Geometry.Radius;
                fixedTransform = other.Geometry.FixedTransform;
            else
                ID = nan;
                L = 2;
                R = 1;
                fixedTransform = eye(dim+1);
                if nargin > 1
                %parseConstructorInput(DIM, ID)
                    ID = varargin{1};
                    validateattributes(ID,{'numeric'},{'scalar'}, 'Capsule', 'ID');
                    if ~isnan(ID)
                        validateattributes(ID,{'numeric'},{'integer'}, 'Capsule', 'ID');
                    end
                end
                if nargin > 2
                %parseConstructorInput(DIM, ID, L)
                    L = varargin{2};
                    validateattributes(L,{'numeric'},{'real','scalar','positive'}, 'Capsule', 'Length');
                end
                if nargin > 3
                %parseConstructorInput(DIM, ID, L, R)
                    R = varargin{3};
                    validateattributes(R,{'numeric'},{'real','scalar','positive'}, 'Capsule', 'Radius');
                end
                if nargin > 4
                %parseConstructorInput(DIM, ID, L, R, pose)
                    fixedTransform = varargin{4};
                    validateattributes(fixedTransform,{'numeric'},{'real','size',[dim+1 dim+1]}, 'CapsuleBase', 'FixedTransform');
                end
            end
        end
    end
end