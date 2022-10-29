classdef Capsule < nav.algs.internal.CapsuleBase
% This class is for internal use only. It may be removed in the future.

%Capsule A 2D collision-checking primitive and helper class for the dynamicCapsuleList

%   Copyright 2020 The MathWorks, Inc.

    %#codegen

    properties (Constant, Access = {?nav.algs.internal.InternalAccess, ?nav.algs.internal.CapsuleBase})
        %DefaultGeometry Default geometry if none is provided
        DefaultGeometry = struct('Length',2,'Radius',1,'FixedTransform',eye(3));
        
        %Dimension Dimension of the Capsule, either 2D or 3D
        Dimension = 2
        
        %TSize Size of FixedTransform
        TSize = [3 3];
    end

    properties (Access = protected)
        %StatesInternal N-by-[x y Oz] list of poses assumed by the capsule
        StatesInternal = [0 0 0];
        
        %GeometryInternal A struct containing the internal geometry properties
        GeometryInternal = nav.algs.internal.Capsule.DefaultGeometry;
    end

    methods (Static)
        function [axHandle, capGroupHandle] = showCapsule(axHandle, xyList, vList, L, R, objIdx, timeSteps, showAllStates, capGroupHandle)
        %showCapsule Displays the capsule in 2D at specifies time steps
            % Hold behavior, need to display all ego states in
            % case we highlight collisions
            if showAllStates && timeSteps(end) >= size(xyList,1)/2
                xy = reshape(xyList(:,objIdx),2,[]);
                v = reshape(vList(:,objIdx),2,[]);
            else
                stepIdx = min(reshape(timeSteps,1,[]),size(xyList,1)/2);
                indices = (stepIdx-1)*2+[1;2];
                xy = reshape(xyList(indices,objIdx),2,[]);
                v = reshape(vList(indices,objIdx),2,[]);
            end

            if nargin < 9
                capGroupHandle = nav.algs.internal.showCapsule2D(axHandle, nav.algs.internal.Capsule.NumCirclePts, xy, v, L, R);
            else
                nav.algs.internal.showCapsule2D(axHandle, nav.algs.internal.Capsule.NumCirclePts, xy, v, L, R, capGroupHandle);
            end
        end
        
        function [position, orientation] = applyStatesToGeometry(capObj, numStates)
        %applyStatesToGeometry Converts SE2 states to 2D transformations and applies to position/orientation
            narginchk(1,2)
            if nargin == 1
                states = capObj.StatesInternal;
                numStates = size(states,1);
            else
                states = nav.algs.internal.CapsuleBase.getFirstNItems(capObj.StatesInternal, numStates);
            end
            
            % Convert states and fixed transform to angles
            aS = reshape(states(:,end),1,[]);
            aF = atan2(capObj.Geometry.FixedTransform(2),capObj.Geometry.FixedTransform(1));
            
            % Cache orientation of the Capsule
            orientation = reshape([cos(aS+aF); sin(aS+aF)],[],1);
            
            % Convert state angles to rotation matrix
            cT = cos(aS);
            sT = sin(aS);
            R = reshape([cT;sT;-sT;cT],2,2,[]);
            
            % Extract translation from states and apply rotation to fixed position
            xy = reshape(states(:,1:2)',[],1);
            p = repmat(capObj.Position,2,1,numStates);
            position = reshape(dot(R,p,2),[],1) + xy;
        end
        
        function states = convertToInternal(states)
        %convert2SE3 Validates the SE2 state matrix
            validateattributes(states,{'numeric'}, ...
                {'nonnan', 'finite', 'size', [nan, 3]}, ...
                'set.States', 'states');
        end
    end
end