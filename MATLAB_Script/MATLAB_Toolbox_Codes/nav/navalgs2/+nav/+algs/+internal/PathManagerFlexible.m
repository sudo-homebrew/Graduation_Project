classdef PathManagerFlexible < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%PathManagerFlexible Manages fixed or varsize properties for reference path objects
%
%   This class manages the reference path's resizeable properties under the
%   following conditions:
%       1) When the runtime environment is MATLAB
%       2) When DynamicMemoryAllocation has not been turned off
%       3) When the Waypoints provided at construction are not varsize
%
%   This class resizes the internal properties depending on the number of
%   Waypoints provided to the path. If the user constructs the class using
%   a finite MaxNumWaypoints, the number of waypoints is limited to the
%   number specified during runtime.
%
%   See also nav.algs.internal.PathManagerFixed, 
%            nav.algs.internal.FrenetReferencePath, 
%            referencePathFrenet
%
%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties (SetAccess = immutable)
        %MaxNumWaypoints Maximum number of waypoints allowed in reference path
        %
        %   Accepts a scalar which determines the maximum number of 
        %   waypoints accepted by this object. 
        %
        %       Inf           - The path is explicitly resizeable.
        %       (Default)
        %
        %                       NOTE: Requires DynamicMemoryAllocation='On'
        %                             for codegen
        %
        %       VALUE         - The maximum number of waypoints in the
        %                       object is limited to VALUE. Use this to 
        %                       create a resizeable path when 
        %                       DynamicMemoryAllocation is not allowed 
        %                       during codegen.
        MaxNumWaypoints
    end
    properties
        %Waypoints Presampled points along the path
        Waypoints
    end
    properties (Access = ?nav.algs.internal.InternalAccess)
        %NumWaypoints Current number of waypoints in path
        NumWaypoints
        
        %Arclengths Arclength of each clothoid segment in path
        Arclengths
        
        %BoundingBoxes XY Limits for each bounding box
        %
        %   BoundingBoxes is a 4xM matrix, of the form 
        %   [Left; Right; Bottom; Top], where M is the number of segments 
        %   in the piecewise continuous clothoid spline.
        BoundingBoxes
        
        %SegStarts Clothoid parameters defined at start of each clothoid segment
        %
        %   M-by-[x y theta kappa dKappa arclength] 
        SegStarts
    end
    methods
        function obj = PathManagerFlexible(waypoints, maxSize)
            narginchk(2,2);
            
            % Set waypoint limit
            obj.MaxNumWaypoints = maxSize;
            
            % Set varsize info on internal properties
            fixedSize = coder.internal.isConst(size(waypoints));
            wp = waypoints;
            if ~fixedSize
                % If the waypoints provided are not fixed-size, then we
                % mark all resizeable properties as varsize unbounded.
                s = [];
                bbox = [];
                sstart = [];
                if coder.internal.isConst(size(waypoints,2))
                    coder.varsize('wp',[inf size(waypoints,2)],[1 0]);
                else
                    coder.varsize('wp',[inf 3],[1 1]);
                end
                coder.varsize('s',[inf 1],[1 0]);
                coder.varsize('bbox',[4 inf],[0 1]);
                coder.varsize('sstart',[inf 6],[1 0]);
            else
                % If the incoming waypoints are fixed-size, then the path
                % becomes fixed-size after construction.
                n = size(waypoints,1);
                s = zeros(n,1);
                sstart = zeros(n,6);
                bbox = zeros(4,n-1);
            end
            obj.Waypoints = wp;
            obj.Arclengths = s;
            obj.BoundingBoxes = bbox;
            obj.SegStarts = sstart;
        end
        function set.Waypoints(obj, waypoints)
            % Check against bounds
            obj.validateWaypoints(waypoints, obj.MaxNumWaypoints); %#ok<MCSUP>
            obj.Waypoints = waypoints;
        end
    end
    
    methods(Access={?nav.algs.internal.FrenetReferencePath},Static)
        function validateWaypoints(waypoints, maxNumAllowed)
        %validateWaypoints Verifies type and size of waypoints
            validateattributes(waypoints, {'numeric'},{'nonempty','finite','real'},'FrenetReferencePath','waypoints');
            validateattributes(size(waypoints,1), {'double'},{'>',1,'<=',maxNumAllowed},'FrenetReferencePath','waypoints rows');
            
            % Verify that the waypoints are not coincident
            xy = waypoints(:,1:2);
            diffDist = xy(:,1:2) - circshift(xy, -1, 1);
            coder.varsize('totalDist');
            totalDist = cumsum(sqrt(sum(diffDist .* diffDist,2)));
            
            % Allow the reference path to have same start and goal point
            if size(waypoints,1) > 2 && isequal(waypoints(1,:),waypoints(end,:))
                totalDist = totalDist(1:end-1);
            end
            
            validateattributes(totalDist, {'numeric'},{'increasing'},'FrenetReferencePath','Waypoints distance');
        end
    end
    
    methods (Static, Hidden)
        function result = matlabCodegenNontunableProperties(~)
        %matlabCodegenNontunableProperties Mark properties as nontunable during codegen
        %
        % Marking properties as 'Nontunable' indicates to Coder that
        % the property should be made compile-time Constant.
            result = {'MaxNumWaypoints'};
        end
    end
end