classdef VisualizationInfo
    %This class is for internal use only. It may be removed in the future.
    
    %VisualizationInfo Visualization info for rigidBodyTree
    %   The information contained here only affects the SHOW method of
    %   RigidBodyTree and can therefore be ignored when comparing
    %   RigidBodyTree objects.

    %   Copyright 2016-2019 The MathWorks, Inc.
    
    %#codegen
    
    properties (Constant)
        
        %ShowTagPrefix
        ShowTagPrefix = 'DO_NOT_EDIT_'
        
    end
    
    
    
    properties
        
        %ShowTag Tag to identify graphic objects
        ShowTag
        
        %EstimatedMaxReach Estimated max reach of the manipulator
        %   The max reach of a manipulator is the maximum of the three 
        %   coordinates (absolute value) of the farthest possible 
        %   point on the robot away from the base origin. 
        %
        %   Default: 1 (meter)
        EstimatedMaxReach = 1
        
        %IsMaxReachUpToDate A boolean indicator that shows whether the
        %   current EstimatedMaxReach is up-to-date 
        %
        %   Default: false
        IsMaxReachUpToDate = false
        
    end
    
    methods
        
        function obj = VisualizationInfo()
            %VisualizationInfo Create VisualizationInfo object
            obj.ShowTag = [robotics.manip.internal.VisualizationInfo.ShowTagPrefix, randomString(5)];
        end
        
        function out = isequal(varargin)
            %isequal Always true for objects of this class
            %   Objects of this class are ignored for comparison purposes.
            out = true;
        end
        
        function out = isequaln(varargin)
            %isequaln  Always true for objects of this class
            %   Objects of this class are ignored for comparison purposes.
            out = true;
        end
    end
end


function s = randomString(n)
%randomString Generate a random string with length N
    charset = char(['a':'z' '0':'9' 'A':'Z' ]);
    nset = length(charset);
    idx = randi(nset,1,n); 
    s = charset(idx);
end
