classdef JtControlMarker < robotics.manip.internal.InteractiveMarker
    %This class is for internal use only. It may be removed in the future.
    
    %JTCONTROLMARKER Marker class for interactiveRigidBodyTree
    
    %   Copyright 2019 The MathWorks, Inc.
    
    properties (SetAccess = private, GetAccess = ?robotics.manip.internal.InternalAccess)
        JointConfigIndex
        
        JointType
        
        JointAxis
        
        JointPositionAtButtonClick
    end
    
    
    properties (SetAccess = private, GetAccess = ?robotics.manip.internal.InternalAccess)
        % Visual elements of this specific marker class
        
        RevJointCircle
        
        RevJointArrow
        
        PrisJointLine
        
        PrisJointArrow
    end
    
    properties (Access = private, Constant)
        MarkerColor = [1 0.85 0]
        
        MarkerColorSelected = [0.9 0.8 0]
        
        ArrowRadius = 0.02
        
        ArrowLength = 0.05
    end
    
    
    methods
        function obj = JtControlMarker(ax, markerPose, jointBody, backendObj, hgTransformsHandles, treeLineData, baselineConfig, scaleFactor)
            %JtControlMarker Constructor
            
            obj.InteractiveRBTBackend = backendObj;
            
            obj.RobotHGTransforms = hgTransformsHandles;
            obj.RobotLineData = treeLineData;
            obj.BaselineConfig = baselineConfig;
            
            % Assign the axes
            obj.AxesHandle = ax;
            
            % Set joint properties
            obj.JointConfigIndex = obj.InteractiveRBTBackend.RigidBodyTree.TreeInternal.PositionDoFMap(jointBody.BodyInternal.Index,1);
            obj.JointAxis = jointBody.Joint.JointAxis;
            obj.JointType = jointBody.Joint.Type;
            
            % Assign scale
            obj.ScaleFactor = scaleFactor;
            
            % Draw circles
            if strcmp(obj.JointType, 'revolute')
                [circlePoints, R] = obj.getRevJointMarkerPts(markerPose);
                [arrowPtsX, arrowPtsY, arrowPtsZ] = obj.getRevJointMarkerArrowPts(R, circlePoints);
                obj.RevJointCircle = line(ax, circlePoints(1,:), circlePoints(2,:), circlePoints(3,:), 'color', obj.MarkerColor, 'ButtonDownFcn', @(cb, evt)buttonDownCBRevJoint(obj, cb, evt), 'LineWidth', obj.LineWidth);
                obj.RevJointArrow = surface(ax, arrowPtsX, arrowPtsY, arrowPtsZ, 'linestyle', 'none', 'facecolor', obj.MarkerColor, 'facelighting', 'gouraud');
                
            else
                [axlePoints, R] = obj.getPrisJointMarkerLinePts(markerPose);
                [arrowPtsX, arrowPtsY, arrowPtsZ] = obj.getPrisJointMarkerArrowPts(R, axlePoints);
                obj.PrisJointLine = line(ax, axlePoints(1,:), axlePoints(2,:), axlePoints(3,:), 'color', obj.MarkerColor, 'ButtonDownFcn', @(cb, evt)buttonDownCBPrisJoint(obj, cb, evt), 'LineWidth', obj.LineWidth);
                obj.PrisJointArrow = surface(ax, arrowPtsX, arrowPtsY, arrowPtsZ, 'linestyle', 'none', 'facecolor', obj.MarkerColor, 'facelighting', 'gouraud', 'ButtonDownFcn', @(cb, evt)buttonDownCBPrisJoint(obj, cb, evt));
            end
            
            % Update the marker's position
            obj.MarkerPose = markerPose;
        end
        
        function delete(obj)
            %delete Destructor
            
            if strcmp(obj.JointType, 'revolute')
                delete(obj.RevJointArrow);
                delete(obj.RevJointCircle);
            else
                delete(obj.PrisJointArrow);
                delete(obj.PrisJointLine);
            end
        end
        
        function updateMarker(obj, markerPose, scaleFactor, lineWidth)
            %updateMarker Update the marker pose and tunable properties
            %   This marker updates the marker pose. Optionally, the scale
            %   factor and line width may also be changed. When these
            %   arguments are not specified, default values are used.
            
            % Set optional arguments
            if nargin > 3
                obj.LineWidth = lineWidth;
            elseif nargin > 2
                obj.ScaleFactor = scaleFactor;
            end
            
            if strcmp(obj.JointType, 'revolute')
                [circlePoints, R] = obj.getRevJointMarkerPts(markerPose);
                [arrowPtsX, arrowPtsY, arrowPtsZ] = obj.getRevJointMarkerArrowPts(R, circlePoints);
                obj.RevJointCircle.XData = circlePoints(1,:);
                obj.RevJointCircle.YData = circlePoints(2,:);
                obj.RevJointCircle.ZData = circlePoints(3,:);
                obj.RevJointArrow.XData = arrowPtsX;
                obj.RevJointArrow.YData = arrowPtsY;
                obj.RevJointArrow.ZData = arrowPtsZ;
            else
                [axlePoints, R] = obj.getPrisJointMarkerLinePts(markerPose);
                [arrowPtsX, arrowPtsY, arrowPtsZ] = obj.getPrisJointMarkerArrowPts(R, axlePoints);
                obj.PrisJointLine.XData = axlePoints(1,:);
                obj.PrisJointLine.YData = axlePoints(2,:);
                obj.PrisJointLine.ZData = axlePoints(3,:);
                obj.PrisJointArrow.XData = arrowPtsX;
                obj.PrisJointArrow.YData = arrowPtsY;
                obj.PrisJointArrow.ZData = arrowPtsZ;
            end
            
            % Update the marker's position
            obj.MarkerPose = markerPose;
        end
    end
    
    methods (Access = protected)
        
        function [circlePoints, worldCircleRotm, p] = getRevJointMarkerPts(obj, T)
            %getRevJointMarkerPts Get points for the revolute joint marker
            %   The revolute joint marker is a circle centered about the
            %   joint. This method returns the points that define that
            %   circle given a transform indicating the marker body pose.
            
            % Get data from transform
            R = T(1:3, 1:3);
            p = T(1:3,4);
            
            % Apply scale factor
            circleRadius = obj.ScaleFactor*obj.CircleRadius;
            
            % Define basic circle
            theta = 0:pi/32:2*pi;
            circleX = circleRadius*cos(theta);
            circleY = circleRadius*sin(theta);
            circleZ = zeros(1,length(circleX));
            
            % Find the rotation of the joint axis relative to the X-axis
            % by taking the cross product with the x-axis vector.
            circleRotm = obj.computeJointAxisOrientationRelativeToX;
            worldCircleRotm = R*circleRotm;
            
            % Circles and axles vary by orientation
            circlePoints = worldCircleRotm*[circleZ; circleX; circleY] + p;
        end
        
        function [linePts, worldLineRotm, p] = getPrisJointMarkerLinePts(obj, T)
            %getPrisJointMarkerLinePts Get points for the prismatic joint marker
            %   The prismatic joint marker is a line drawn through the
            %   joint. This method returns the points that define that
            %   line given a transform indicating the marker body pose.
            
            % Get data from transform
            R = T(1:3, 1:3);
            p = T(1:3,4);
            
            % Apply scale factor
            markerHalfLineLength = obj.ScaleFactor*obj.CircleRadius;
            
            % By default, this marker is oriented about the X-axis.
            % Determine the rotation of the marker relative to the X-axis
            % to ensure that it is correctly drawn.
            lineRotm = obj.computeJointAxisOrientationRelativeToX;
            worldLineRotm = R*lineRotm;
            
            
            linePts = worldLineRotm*[-markerHalfLineLength, 0, 0; markerHalfLineLength, 0, 0]' + p;
        end
        
        function [xPts, yPts, zPts] = getPrisJointMarkerArrowPts(obj, R, linePts)
            
            % Since the marker arrow is shared between different joint
            % marker types (prismatic and revolute), it is plotted in a
            % particular configuration and must be rotated to ensure that
            % it is consistent with the prismatic marker positive direction
            Rlocal = [-1 0 0; 0 1 0; 0 0 -1]; %axang2rotm([0 1 0 pi])
            
            % Get base location from the linePts data
            p = linePts(:,2);
            
            % Create the arrow at the point given by p and with orientation
            % T = R*Rlocal, i.e. the orientation of the marker on the axis,
            % but re-oriented in the world frame based on the marker's
            % orientation in the world (given by R)
            [xPts, yPts, zPts] = obj.createMarkerArrow(R*Rlocal, p);
        end
        
        function [xPts, yPts, zPts] = getRevJointMarkerArrowPts(obj, R, linePts)
            
            % Since the marker arrow is shared between different joint
            % marker types (prismatic and revolute), it is plotted in a
            % particular configuration and must be rotated to ensure that
            % it is consistent with the revolute marker positive direction
            Rlocal = [0 0 1; 0 1 0; -1 0 0]; %axang2rotm([0 1 0 pi/2])
            
            % Get attachment location from the linePts data
            p = linePts(:,1);
            
            % Create the arrow at the point given by p and with orientation
            % T = R*Rlocal, i.e. the orientation of the marker on the axis,
            % but re-oriented in the world frame based on the marker's
            % orientation in the world (given by R)
            [xPts, yPts, zPts] = obj.createMarkerArrow(R*Rlocal, p);
        end
        
        function updateMarkerColor(obj, isMarkerSelected)
            %updateMarkerColor Update the marker color
            %   This method updates the marker color based on the input
            %   flag, isMarkerSelected. If the flag is true, then the
            %   marker is set to its color when selected; otherwise the
            %   default color is used. The target of the color change
            %   depends on the joint type.
            
            if isMarkerSelected
                markerColor = obj.MarkerColorSelected;
            else
                markerColor = obj.MarkerColor;
            end
            
            if strcmp(obj.JointType, 'revolute')
                obj.RevJointCircle.Color = markerColor;
            else
                obj.PrisJointLine.Color = markerColor;
                obj.PrisJointArrow.FaceColor = markerColor;
            end            
        end
        
        function [xPts, yPts, zPts] = createMarkerArrow(obj, R, p)
            %createMarkerArrow Create a conic arrow surface
            %   Given a 3x3 rotation matrix and a 1x3 translation vector,
            %   this function creates a conic surface at p with the
            %   orientation specified by R.
            
            % The arrow is built using a cone that is defined by a circle
            % on one end (when X is 0) and a point on the other (where X is
            % positive). 
            theta = 0:pi/16:2*pi;
            numCirclePts = length(theta);
            coneBaseRadius = 3*obj.ArrowRadius/obj.LineWidth;
            outerCirclePts = [coneBaseRadius*cos(theta); coneBaseRadius*sin(theta); zeros(1,numCirclePts)];
            
            % Define the X, Y, and Z data for a cone perched on the origin. Each surface has two rows that define
            % the bounds of that data set. For the X and Y data, the two
            % bounds are the origin and the perimeter of the circle. The
            % Z-data is all zeros at the base, while the other row of Z's
            % is set to the desired arrow length. Offset the length of the
            % marker slightly ensure that the tip is clearly visible (and
            % not covered by the marker line instead).
            localXSurfPts = [zeros(1,numCirclePts); obj.ArrowLength*ones(1,numCirclePts)] - obj.ArrowLength/3;
            localYSurfPts = [zeros(1,numCirclePts); outerCirclePts(1,:)];
            localZSurfPts = [zeros(1,numCirclePts); outerCirclePts(2,:)];
            
            % Rotate and position the surface
            xyz1 = R*[localXSurfPts(1,:); localYSurfPts(1,:); localZSurfPts(1,:)] + p;
            xyz2 = R*[localXSurfPts(2,:); localYSurfPts(2,:); localZSurfPts(2,:)] + p;
            
            % Rearrange into XData, YData, and ZData
            xPts = [xyz1(1,:); xyz2(1,:)];
            yPts = [xyz1(2,:); xyz2(2,:)];
            zPts = [xyz1(3,:); xyz2(3,:)];
        end
    end
    
    %% Revolute Joint Control Callbacks
    
    methods (Access = ?robotics.manip.internal.InternalAccess)
        
        function buttonDownCBRevJoint(obj, hcbo, evt)
            %buttonDownCBRevJoint Button-down callback for revolute joint marker
            %   This is the button-down callback for the revolute joint
            %   marker. Given a handle callback object, hcbo, that has been
            %   clicked on, this callback sets key selection details about
            %   the location of the click in the figure and on the object
            %   and stores the marker pose at the time of the click. It
            %   also updates the move callbacks so they are specific to the
            %   revolute joint marker.
            
            % Register starting points for the rotation that will be used
            % in the rotation move callback
            obj.registerStartingPointsForRotation(hcbo, evt);
            
            % Store the marker pose at the instant the marker is first
            % selected
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            obj.JointPositionAtButtonClick = obj.InteractiveRBTBackend.Configuration(obj.JointConfigIndex);
            
            % Update the button callbacks
            fig = obj.AxesHandle.Parent;
            fig.WindowButtonMotionFcn = @(cb, evt)buttonMoveCBRevJoint(obj, cb, evt);
            fig.WindowButtonUpFcn = @(cb, evt)buttonUpCBRevJoint(obj, cb, evt);
            
            % Update color of selected objects
            obj.updateMarkerColor(true);
        end
        
        function buttonMoveCBRevJoint(obj, hcbo, evt) %#ok<INUSD>
            %buttonMoveCBRevJoint Button-move callback for revolute joint marker
            %   This is the button-move callback for the revolute joint
            %   marker. Given a handle callback object, hcbo, that has been
            %   clicked on, this callback updates the line marker position
            %   based on the current position of the mouse. It then updates
            %   the position of the joint in question and updates the joint
            %   configuration in the backend and on the axes.
            
            % Compute the marker's rotation relative to the initial value
            % when it was first clicked
            theta = obj.updateCircleMarkerRotation;
            
            % Apply the rotation to the joint in question by adding the
            % relative rotation, theta, to the original value
            obj.InteractiveRBTBackend.Configuration(obj.JointConfigIndex) = obj.JointPositionAtButtonClick + theta;
            obj.updateAxesConfig;
            
            % Update color of selected objects
            obj.updateMarkerColor(true);
        end
        
        function  buttonUpCBRevJoint(obj, hcbo, evt ) %#ok<INUSD>
            %buttonUpCBRevJoint Button-up callback for the revolute joint marker
            %   This method is the button-up callback for the revolute
            %   joint marker. It resets values and appearances to their
            %   initial pre-selected state.
            
            obj.resetHCBOCallbacks(hcbo);
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            
            % Update color of selected objects
            obj.updateMarkerColor(false);
        end
        
        %% Prismatic Joint Translation Callbacks
        
        function buttonDownCBPrisJoint(obj, hcbo, evt )
            %buttonDownCBPrisJoint Button-down callback for prismatic joint marker
            %   This is the button-down callback for the prismatic joint
            %   marker. Given a handle callback object, hcbo, that has been
            %   clicked on, this callback sets key selection details about
            %   the location of the click in the figure and on the object
            %   and stores the marker pose at the time of the click. It
            %   also updates the move callbacks so they are specific to the
            %   prismatic joint marker.
            
            % Update the line marker
            obj.registerStartingPointsForLinearMotion(hcbo, evt, obj.JointAxis);
            
            % Store the marker pose at the instant the marker is first
            % selected
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            obj.JointPositionAtButtonClick = obj.InteractiveRBTBackend.Configuration(obj.JointConfigIndex);
                        
            % Update axes callbacks
            obj.AxesHandle = hcbo.Parent;
            fig = obj.AxesHandle.Parent;
            fig.WindowButtonMotionFcn = @(cb, evt)buttonMoveCBPrisJoint(obj, cb, evt);
            fig.WindowButtonUpFcn = @(cb, evt)buttonUpCBPrisJoint(obj, cb, evt);
            
            % Update color of selected objects
            obj.updateMarkerColor(true);
        end
        
        function buttonMoveCBPrisJoint(obj, hcbo, evt ) %#ok<INUSD>
            %buttonMoveCBPrisJoint Button-move callback for prismatic joint marker
            %   This is the button-move callback for the prismatic joint
            %   marker. Given a handle callback object, hcbo, that has been
            %   clicked on, this callback updates the line marker position
            %   based on the current position of the mouse. It then updates
            %   the position of the joint in question and updates the joint
            %   configuration in the backend and on the axes.
            
            % Update the marker's position
            markerDist = obj.updateLinearMarkerPosition;
            
            % Update configuration and visualization
            obj.InteractiveRBTBackend.Configuration(obj.JointConfigIndex) = markerDist + obj.JointPositionAtButtonClick;
            obj.updateAxesConfig;
            
            % Update color of selected objects
            obj.updateMarkerColor(true);
        end
        
        function  buttonUpCBPrisJoint(obj, hcbo, evt ) %#ok<INUSD>
            %buttonUpCBPrisJoint Button-up callback for the prismatic joint marker
            %   This method is the button-up callback for the prismatic
            %   joint marker. It resets values and appearances to their
            %   initial pre-selected state.
            
            obj.resetHCBOCallbacks(hcbo);
            
            % Update color of selected objects
            obj.updateMarkerColor(false);
        end
        
        %% Helper methods
        
        function axisRotation = computeJointAxisOrientationRelativeToX(obj)
            %computeJointAxisOrientationRelativeToX Compute the orientation of the joint-axis relative to the X-axis
            %   By default, the points for the prismatic marker are drawn
            %   along the X-axis, and the points for the circle in the
            %   revolute marker are drawn about the X=axis. Therefore, if
            %   the actual joint axis lies on a different vector in the
            %   local frame, a rotation matrix is needed to transform the
            %   joint axis in X to the actual joint axis. This method
            %   computes that rotation.
            
            % Find the rotation of the joint axis relative to the X-axis
            % by taking the cross product with the x-axis vector.
            rotVec = cross([1 0 0], obj.JointAxis/norm(obj.JointAxis));
            if any(rotVec ~= 0)
                axisRotation = axang2rotm([rotVec asin(norm(rotVec))]);
            elseif (dot(obj.JointAxis/norm(obj.JointAxis), [1 0 0]) < 0)
                % If the cross product is zero but the dot product is
                % negative, the circle should be rotated pi radians about
                % z, as the axis is flipped.
                axisRotation = axang2rotm([0 0 1 pi]);
            else
                % If the cross product is zero and the dot product is
                % positive, the joint-axis is already in line and no
                % rotation is required.
                axisRotation = eye(3);
            end
        end
        
    end
    
    
end