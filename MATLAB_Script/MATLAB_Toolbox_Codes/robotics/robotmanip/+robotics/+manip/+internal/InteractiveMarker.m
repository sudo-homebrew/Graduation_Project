classdef (Abstract) InteractiveMarker < robotics.manip.internal.InternalAccess
    %This class is for internal use only. It may be removed in the future.
    
    %INTERACTIVEMARKER Base class for interactive marker with callbacks
    
    %   Copyright 2019 The MathWorks, Inc.
    
    properties
        %TODO: Make this dynamic
        ScaleFactor = 1
        
        LineWidth = 3
        
        MarkerPose
        
        AxesHandle
    end
    
    properties (Constant)
        
        CircleRadius = 0.2
    end
    
    properties (SetAccess = ?robotics.manip.internal.InternalAccess, GetAccess = protected)
        InteractiveRBTBackend
        
        RobotHGTransforms
        
        RobotLineData
        
        BaselineConfig
    end
    
    % Properties used for callbacks
    properties (SetAccess = protected, GetAccess = ?robotics.manip.internal.InternalAccess)
        
        PointOnCircle
        
        InitialClickPtOnMarker
        
        SurfaceNormal
        
        CircleCenter
        
        LineMarkerVector
        
        MarkerPoseAtButtonClick
    end
    
    %% Abstract methods
    methods (Abstract)
        updateMarker(markerPose, varargin)
    end
    
    %% Callback helpers
    
    methods (Access = protected)
        function updateAxesConfig(obj)
            %updateAxesConfig Update the axes visuals
            
            % Compute the transform tree, then update the HGTransforms
            Ttree = obj.InteractiveRBTBackend.RigidBodyTree.TreeInternal.forwardKinematics(obj.InteractiveRBTBackend.Configuration);
            robotics.manip.internal.RigidBodyTreeVisualizationHelper.fastVisualizationUpdate(obj.RobotHGTransforms, obj.RobotLineData, Ttree, obj.BaselineConfig);
        end
        
        function registerStartingPointsForRotation(obj, hcbo, evt)
            %registerStartingPointsForRotation Initialize parameters for circle marker callbacks
            %   This method is called when a circular marker used to
            %   control rotation is clicked on. The method defines the
            %   point on the circle that was first clicked on, and
            %   initializes some other properties that are used in
            %   subsequent callbacks. The method accepts the handle
            %   callback object, hcbo. In this case, the handle callback
            %   object is the handle to the circular marker in the figure.
            
            % Save the X, Y, and Z data points of the circle that was
            % clicked on
            P = [hcbo.XData; hcbo.YData; hcbo.ZData];
            
            % Compute the circle center from the marker pose
            obj.CircleCenter = obj.MarkerPose(1:3,4);
            
            % Compute the surface normal using the cross product of two
            % vectors on the surface spaced about pi/4 radians apart
            nQuarterRot = floor(size(P,2)/4);
            v1 = P(:,1) - obj.CircleCenter;
            v2 = P(:,nQuarterRot) - obj.CircleCenter;
            
            % The surface normal will be v1 x v2 since the circle is drawn
            % in the counter-clockwise direction relative to its axis
            obj.SurfaceNormal = robotics.core.internal.SEHelpers.skew(v1)*(v2);
            obj.SurfaceNormal = obj.SurfaceNormal/norm(obj.SurfaceNormal);
            
            % Get the currentPoint line for the figure. This is a 2x3
            % matrix depicting the line in 3D space that the user could be
            % clicking on. Since the figure is a 3D world projected onto a
            % 2D monitor, whenever the user clicks on a point, there is
            % actually a line from the front to back of the figure that they
            % could be clicking on, which is what the current point
            % captures.
            obj.AxesHandle = hcbo.Parent;
            
            % Initialize the point on the marker that the user has clicked
            % on using the figure event callback data. The event data is
            % only provided at button-click (not during subsequent dragging
            % motions).
            obj.InitialClickPtOnMarker = evt.IntersectionPoint';
        end
        
        function theta = updateCircleMarkerRotation(obj)
            %updateCircleMarkerRotation Update properties for circle marker callbacks
            %   This method is called when a circular marker used to
            %   control rotation is dragged from an initial position. Given
            %   the current point of the mouse in the figure and the
            %   initial point that was clicked on (defined in the
            %   button-down callback), this method computes the perceived
            %   rotation of the marker. The method then updates the marker
            %   pose and proceeds to execute the updateMarker method, which
            %   updates the figure.
            
            % Get the currentPoint line for the figure. This is a 2x3
            % matrix depicting the line in 3D space that the user could be
            % clicking on. Since the figure is a 3D world projected onto a
            % 2D monitor, whenever the user clicks on a point, there is
            % actually a line from the front to back of the figure that they
            % could be clicking on, which is what the current point
            % captures.
            cp = obj.AxesHandle.CurrentPoint;
            
            % Compute the intersection of the current-point line with the
            % circular rotation marker
            mousePtInCirclePlane = obj.computeMarkerCurrentPointLineIntersection(cp(1,:)', cp(2,:)', obj.CircleCenter, obj.SurfaceNormal);
                        
            % Define two vectors, one from the circle center to the initial
            % click point, and one from the circle center to the current
            % point and normalize them
            circleCtrToCurrPt = (mousePtInCirclePlane - obj.CircleCenter);
            circleCtrToCurrPt = circleCtrToCurrPt/norm(circleCtrToCurrPt);
            circleCtrToInitPt = (obj.InitialClickPtOnMarker - obj.CircleCenter);
            circleCtrToInitPt =  circleCtrToInitPt/norm(circleCtrToInitPt);
            costheta = circleCtrToCurrPt'*circleCtrToInitPt;
            
            % Take the cross product of the two vectors to get their normal
            % vector, which has magnitude sin(theta), since they are both
            % normalized
            crossProd = -robotics.core.internal.SEHelpers.skew(circleCtrToCurrPt)*circleCtrToInitPt;
            sintheta = sign(crossProd'*obj.SurfaceNormal)*norm(crossProd)';
            theta = atan2(sintheta, costheta);
            
            % Compute marker orientation with respect to reference
            % coordinates
            R = expm(theta*robotics.core.internal.SEHelpers.skew(obj.SurfaceNormal));
            R0 = obj.MarkerPoseAtButtonClick(1:3, 1:3);
            obj.MarkerPose(1:3, 1:3) = R*R0;
            
            % Update the marker pose
            obj.updateMarker(obj.MarkerPose);
        end
        
        function registerStartingPointsForLinearMotion(obj, hcbo, evt, axisDirection)
            %registerStartingPointsForLinearMotion Initialize parameters for line marker callbacks
            %   This method is called when a line marker used to control
            %   translation is clicked on. The method defines
            %   initialization properties, notably LineMarkerVector, which
            %   is a normalized vector in the direction of the marker. The method accepts the
            %   handle callback object, hcbo, and a known axis direction
            %   for the marker. In this case, the handle callback object is
            %   the handle to the linear marker in the figure, or to its
            %   gripper.
            
            % Initialize the point on the marker that the user has clicked
            % on using the figure event callback data. The event data is
            % only provided at button-click (not during subsequent dragging
            % motions).
            obj.InitialClickPtOnMarker = evt.IntersectionPoint';
            
            % Update the axis handle
            obj.AxesHandle = hcbo.Parent;
            
            % Compute a vector in the direction of the line that is clicked
            % on and normalize it
            obj.LineMarkerVector = obj.MarkerPose(1:3,1:3)*axisDirection(:);  % x axis direction
            obj.LineMarkerVector = obj.LineMarkerVector/norm(obj.LineMarkerVector);
        end
        
        function markerDist = updateLinearMarkerPosition(obj)
            %updateLinearMarkerPosition Update properties for line marker callbacks
            %   This method is called when a line marker used to control
            %   translation is dragged from an initial position. Given the
            %   current point of the mouse in the figure and the initial
            %   point that was clicked on (defined in the button-down
            %   callback), this method computes the perceived translation
            %   of the marker. The method then updates the marker pose and
            %   proceeds to execute the updateMarker method, which updates
            %   the figure.
            
            % Get the currentPoint line for the figure. This is a 2x3
            % matrix depicting the line in 3D space that the user could be
            % clicking on. Since the figure is a 3D world projected onto a
            % 2D monitor, whenever the user clicks on a point, there is
            % actually a line from the front to back of the figure that they
            % could be clicking on, which is what the current point
            % captures. The store the two ends of the current point line
            cp = obj.AxesHandle.CurrentPoint;
            cp1 = cp(1,:)';
            cp2 = cp(2,:)';
            
            % Project the mouse motion onto the direction of the axis
            % associated with the line marker
            currentPointVector = cp1 - cp2;
            n0 = robotics.core.internal.SEHelpers.skew(currentPointVector)*obj.LineMarkerVector;
            n1 = robotics.core.internal.SEHelpers.skew(currentPointVector)*n0;
            
            % Compute the motion of the current selection relative to the
            % original marker position
            initMarkerPos = obj.MarkerPoseAtButtonClick(1:3,4);
            c2 = initMarkerPos + ( ((cp1 - initMarkerPos)'*n1 )/(obj.LineMarkerVector'*n1)) * obj.LineMarkerVector;
            
            % Compute the offset of the initial click point relative to the
            % top of the marker to ensure that position takes into account
            % where the marker was clicked on
            clickOffset = (dot((obj.InitialClickPtOnMarker - initMarkerPos), obj.LineMarkerVector)/norm(obj.LineMarkerVector)^2)*obj.LineMarkerVector;
            
            % Update the marker position
            obj.MarkerPose(1:3,4) = c2 - clickOffset;
            
            % Compute the distance the marker has traveled along its axis
            markerDistVector = obj.MarkerPose(1:3,4) - initMarkerPos;
            markerDist = sign(markerDistVector'*obj.LineMarkerVector)*norm(markerDistVector);
            
            obj.updateMarker(obj.MarkerPose);
        end
    end
    
    methods (Static, Access = protected)
        function intersectionPoint = computeMarkerCurrentPointLineIntersection(currentPt1, currentPt2, pointOnCircle, surfaceNormal)
            %computeMarkerCurrentPointLineIntersection Compute the intersection of the current-point line and the circular marker
            %   When a user clicks on the marker, the axes return a
            %   "CurrentPoint" object, which is really 2 3D points that
            %   span a line indicating where the user could have clicked.
            %   This method computes the approximate intersection of that
            %   line with the circular marker.
            
            nume = -surfaceNormal'*(currentPt1 - pointOnCircle);
            deno = surfaceNormal'*(currentPt2 - currentPt1);
            s = nume/deno;
            intersectionPoint = currentPt1 + s*(currentPt2- currentPt1);
        end
    end
    
    methods (Static, Access = protected)
        function resetHCBOCallbacks(hcbo)
            %resetHCBOCallbacks Reset object callbacks to their general state
            
            set(hcbo,'Pointer','arrow')
            set(hcbo,'WindowButtonMotionFcn','')
            set(hcbo,'WindowButtonUpFcn','')
        end
    end
    
end