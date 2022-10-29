classdef PathManagerFixed < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%PathManagerFixed Manages upper-bounded properties for reference path objects
%
%   This class is used when DynamicMemoryAllocation has been turned off
%   during codegen workflows. The manager class preallocates memory for all
%   properties which may change during runtime, and returns only the
%   portions of the variables that are actively being used.
%
%   The user must set MaxNumWaypoints to a finite scalar during
%   construction of the reference path, otherwise MATLAB Coder will throw
%   an error.
%
%   See also nav.algs.internal.PathManagerFlexible, 
%            nav.algs.internal.FrenetReferencePath, 
%            referencePathFrenet
%
%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties
        %Waypoints Presampled points along the path
        Waypoints
    end
    properties (SetAccess = immutable)
        %MaxNumWaypoints Maximum number of waypoints allowed in reference path
        %
        %   Accepts a scalar which determines the maximum number of 
        %   waypoints accepted by this object. 
        %
        %       VALUE         - The maximum number of waypoints in the
        %                       object is limited to VALUE. Use this to 
        %                       create a resizeable path when 
        %                       DynamicMemoryAllocation is not allowed 
        %                       during codegen.
        MaxNumWaypoints
    end
    properties(Access = ?nav.algs.internal.InternalAccess)
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
        function obj = PathManagerFixed(waypoints, maxSize)
            narginchk(2,2);
            validateattributes(maxSize,{'numeric'},{'>',1,'integer','finite'},'FrenetReferencePath','MaxNumWaypoints');
            
            % Reserve max memory for each resizeable property. Each
            % property has an associated get/set method which provide the
            % client (path object) only with the currently used portion of 
            % preallocated memory. The size of this portion is dictated by
            % the number of waypoints.
            obj.MaxNumWaypoints = maxSize;
            obj.Arclengths      = zeros(maxSize,1);
            obj.SegStarts       = zeros(maxSize,6);
            obj.BoundingBoxes   = zeros(4,maxSize-1);
            obj.Waypoints       = waypoints;
        end
        
        function wp = get.Waypoints(obj)
            wp = obj.Waypoints(1:obj.NumWaypoints,:);
        end
        function set.Waypoints(obj, wp)
            maxPts = obj.MaxNumWaypoints; %#ok<MCSUP>
            obj.validateWaypoints(wp, maxPts); 
            obj.Waypoints = obj.embedMatrix(maxPts,wp,2);
            obj.NumWaypoints = size(wp,1); %#ok<MCSUP>
        end
        
        function s = get.Arclengths(obj)
            s = obj.Arclengths(1:obj.NumWaypoints);
        end
        function set.Arclengths(obj,s)
            obj.Arclengths = obj.embedMatrix(obj.MaxNumWaypoints,s,2);
        end
        
        function bbox = get.BoundingBoxes(obj)
            bbox = obj.BoundingBoxes(:,1:(obj.NumWaypoints-1));
        end
        function set.BoundingBoxes(obj,bbox)
            obj.BoundingBoxes = obj.embedMatrix(obj.MaxNumWaypoints,bbox,1);
        end
        
        function ss = get.SegStarts(obj)
            ss = obj.SegStarts(1:obj.NumWaypoints,:);
        end
        function set.SegStarts(obj,ss)
            obj.SegStarts = obj.embedMatrix(obj.MaxNumWaypoints,ss,2);
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
            totalDist = cumsum(sqrt(sum(diffDist .* diffDist,2)));
            
            % Allow the reference path to have same start and goal point
            if size(waypoints,1) > 2 && isequal(waypoints(1,:),waypoints(end,:))
                validateattributes(totalDist(1:end-1), {'numeric'},{'increasing'},'FrenetReferencePath','Waypoints distance');
            else
                validateattributes(totalDist, {'numeric'},{'increasing'},'FrenetReferencePath','Waypoints distance');
            end
            
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
        
        function out = embedMatrix(maxSize, incomingVal, fixedDim)
        %embedMatrix Allocate a new matrix and populate entries
            inSz = size(incomingVal);
            switch fixedDim
                case 2
                    out = zeros(maxSize, inSz(2));
                    out(1:inSz(1),:) = incomingVal;
                otherwise
                    out = zeros(inSz(1),maxSize);
                    out(:,1:inSz(2)) = incomingVal;
            end
        end
    end
end