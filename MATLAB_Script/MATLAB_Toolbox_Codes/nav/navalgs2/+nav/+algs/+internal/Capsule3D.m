classdef Capsule3D < nav.algs.internal.CapsuleBase
% This class is for internal use only. It may be removed in the future.

%Capsule3D A 3D collision-checking primitive and helper class for the dynamicCapsuleList

%   Copyright 2020 The MathWorks, Inc.

    %#codegen

    properties (Constant, Access = {?nav.algs.internal.InternalAccess, ?nav.algs.internal.CapsuleBase})
        %DefaultGeometry Default geometry if none is provided
        DefaultGeometry = struct('Length',2,'Radius',1,'FixedTransform',eye(4));
        
        %Dimension Dimension of the Capsule, either 2D or 3D
        Dimension = 3;
        
        %TSize Size of FixedTransform
        TSize = [4 4];
    end

    properties (Access = protected)
        %StatesInternal N-by-[x y z qW qX qY qZ] list of poses assumed by the capsule
        StatesInternal = [0 0 0 1 0 0 0];
        
        %GeometryInternal A struct containing the internal geometry properties
        GeometryInternal = nav.algs.internal.Capsule3D.DefaultGeometry;
    end
    
    methods (Static)
        function [axHandle, capGroupHandle] = showCapsule(axHandle, capObj, timeSteps, showAllStates, capGroupHandle)
        %showCapsule Displays the capsule in 3D at specifies time steps
            geometry = capObj.Geometry;
            geometry.Position = capObj.Position;
            geometry.OrientationVector = capObj.OrientationVector;
            
            % Hold behavior, need to display all ego states in
            % case we highlight collisions
            stepIdx = min(timeSteps,size(capObj.StatesInternal,1));
            if showAllStates && stepIdx(end) >= size(capObj.StatesInternal,1)
                states = capObj.StatesInternal;
            else
                states = capObj.StatesInternal(stepIdx,:);
            end
            if nargin <= 4
                capGroupHandle = nav.algs.internal.showCapsule3D(axHandle, nav.algs.internal.CapsuleBase.NumCirclePts, geometry, states);
            else
                nav.algs.internal.showCapsule3D(axHandle, nav.algs.internal.CapsuleBase.NumCirclePts, geometry, states, capGroupHandle);
            end
        end
        
        function [position, orientation] = applyStatesToGeometry(capObj, numStates)
        %applyStatesToGeometry Converts SE3 states to 3D transformations and applies to position/orientation
            narginchk(1,2)
            if nargin == 1
                states = capObj.StatesInternal;
                numStates = size(states,1);
            else
                states = nav.algs.internal.CapsuleBase.getFirstNItems(capObj.StatesInternal, numStates);
            end
            
            % Convert quaternion to rotation matrix
            R = quat2rotm(states(:,4:end));
            xyz = reshape(states(:,1:3)',[],1);
            
            % Cache the position and orientation of the Capsule
            v = repmat(capObj.OrientationVector,3,1,numStates);
            orientation = reshape(dot(R,v,2),[],1);
            p = repmat(capObj.Position,3,1,numStates);
            position = reshape(dot(R,p,2),[],1) + xyz;
        end
        
        function se3states = convertToInternal(states)
        %convertToInternal Ensures incoming states are true SE3-quaternion
            validateattributes(states,{'numeric'}, ...
                {'nonnan', 'finite', 'size', [nan, 7]}, ...
                'set.States', 'states');
            se3states = states;
            % Check for invalid quaternions
            mag = vecnorm(states(:,4:7),2,2);
            
            coder.internal.errorIf(any(mag == 0), 'nav:navalgs:dynamiccapsulelist:InvalidQuaternion');
            se3states(:,4:7) = se3states(:,4:7)./repmat(mag,1,4);
        end
    end
end
