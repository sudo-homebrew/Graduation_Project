classdef IKControlMarker < robotics.manip.internal.InteractiveMarker
    %This class is for internal use only. It may be removed in the future.
    
    %IKCONTROLMARKER Marker class for interactiveRigidBodyTree
    %   The IK marker consists of three circles, each centered about 3
    %   axles that correspond to the X, Y, and Z directions of the marker,
    %   and a spherical center marker element used for unconstrained
    %   translation. When the marker is first initialized and assigned to a
    %   body, the X, Y, and Z axles align with the X, Y, and Z axes of the
    %   body that the marker is affiliated with. If the user clicks on an
    %   axle and drags along it, the marker will move in a straight line
    %   constrained the vector direction defined by the axles. If the
    %   user clicks on a circle, they can rotate it, which will rotate the
    %   marker along the circle, i.e. about the corresponding axle. If the
    %   user clicks on the global translation marker, they can move the
    %   marker along the world X, Y, or Z coordinates. These marker
    %   elements are defined as properties and are referenced in the
    %   routines below.

    %   Copyright 2019-2021 The MathWorks, Inc.
    
    properties (SetAccess = private, GetAccess = ?robotics.manip.internal.InternalAccess)
        % Visual elements of this specific marker class
        
        %XCircle Circle that defines rotation about the marker's X-axis
        %   This circle rotates the marker about the X-axle
        XCircle
        
        %YCircle Circle that defines rotation about the marker's Y-axis
        %   This circle rotates the marker about the Y-axle
        YCircle
        
        %ZCircle Circle that defines rotation about the marker's Z-axis
        %   This circle rotates the marker about the Z-axle
        ZCircle
        
        %XAxle Line that defines translation along the marker's X-axis
        XAxle
        
        %YAxle Line that defines translation along the marker's Y-axis
        YAxle
        
        %ZAxle Line that defines translation along the marker's Z-axis
        ZAxle
        
        %XAxleGrip Sphere placed at the end of the XAxle line
        %   This sphere can be used as a grip. It also indicates the
        %   positive direction of the X-axis on the marker.
        XAxleGrip
        
        %YAxleGrip Sphere placed at the end of the YAxle line
        %   This sphere can be used as a grip. It also indicates the
        %   positive direction of the Y-axis on the marker.
        YAxleGrip
        
        %ZAxleGrip Sphere placed at the end of the ZAxle line
        %   This sphere can be used as a grip. It also indicates the
        %   positive direction of the Z-axis on the marker.
        ZAxleGrip
        
        %GlobalTransGrip Marker element used to translate the marker freely in Cartesian space
        GlobalTransGrip
    end
    
    properties (Access = private, Constant)
        XColor = [1 0 0]
        
        XColorSelected = [0.7 0.1 0.1]
        
        YColor = [0 1 0]
        
        YColorSelected = [0.1 0.7 0.1]
        
        ZColor = [0 0 1]
        
        ZColorSelected = [0.4 0.4 0.8]
        
        GlobalColor = [0.7 0.7 0.7]
        
        GlobalColorSelected = [0.5 0.5 0.5]
        
        AxleGripOffsetCoefficent = 1.4
        
        AxleGripSphereRadius = .02
        
        GlobalTransHandleRadius = 0.15
    end

    properties (Dependent)
        %Visible Property to indicate whether or not the marker is visible
        Visible
    end
    
    properties (Access = private)
        %ExternalButtonDownCB Handle to function callback to be triggered by marker selection
        ExternalButtonDownCB

        %ExternalButtonMoveCB Handle to function callback to be triggered by clicking on and moving the marker
        ExternalButtonMoveCB
        
        %ExternalButtonUpCB Handle to function callback to be triggered by releasing the marker
        ExternalButtonUpCB
    end
    
    methods
        function obj = IKControlMarker(ax, markerPose, scaleFactor, varargin)
            %IKControlMarker Constructor
            %   The IK Control Marker creates a 6-DoF Marker that can be
            %   moved around a figure window. When it is initialized, the
            %   marker is placed on the axis given by AX, at the pose set
            %   by MARKERPOSE, and using the specified scalefactor (a
            %   number that is defined relative to 1). This marker object
            %   can be selected and dragged using the controls on the
            %   object. Additionally, the three callback functions
            %   ExternalButtonDownCB, ExternalButtonMoveCB, and
            %   ExternalButtonUpCB, can be set using optional Name/Value
            %   pairs and govern what will happen when an object in the
            %   marker is clicked on, selected and moved, or released,
            %   respectively. These inputs can be left empty if nothing
            %   happens during the corresponding action. The IK Control
            %   marker actually does not do any inverse kinematics; it is
            %   named for legacy reasons, but any associated inverse
            %   kinematics is handled by the external callbacks specified
            %   in the input.
            
            % Assign the axes
            obj.AxesHandle = ax;
            
            % Assign scale
            obj.ScaleFactor = scaleFactor;

            % Assign external button-move and button-up callbacks to be
            % triggered by the local callbacks, which only govern marker
            % behavior
            names = {'ExternalButtonDownCB', 'ExternalButtonMoveCB', 'ExternalButtonUpCB'};
            defaults = {@(pose)(pose), @(pose)(pose), @(pose)(pose)}; % The default value is a no-action function that still accepts the same input
            parser = robotics.core.internal.NameValueParser(names, defaults);
            parse(parser, varargin{:});
            obj.ExternalButtonDownCB = parameterValue(parser, 'ExternalButtonDownCB');
            obj.ExternalButtonMoveCB = parameterValue(parser, 'ExternalButtonMoveCB');
            obj.ExternalButtonUpCB = parameterValue(parser, 'ExternalButtonUpCB');
            
            % Get scaled circle, axle, and sphere data
            [xcPts, ycPts, zcPts, xaPts, yaPts, zaPts] = obj.getCirclePts(markerPose);
            [smallSphereX, smallSphereY, smallSphereZ] = obj.getSpherePts;
            [globalTransX, globalTransY, globalTransZ] = obj.getGlobalTransSurfPts(markerPose);
            
            % Draw circles and assign callbacks. The callbacks are kept
            % distinct to ensure that if one marker is clicked on, but the
            % marker then drags to another object, only the first item is
            % triggered
            obj.XCircle = line(ax, xcPts(1,:), xcPts(2,:), xcPts(3,:), 'color', obj.XColor, 'ButtonDownFcn', @(cb, evt)buttonDownCBRotX(obj, cb, evt), 'LineWidth', obj.LineWidth);
            obj.YCircle = line(ax, ycPts(1,:), ycPts(2,:), ycPts(3,:), 'color', obj.YColor, 'ButtonDownFcn', @(cb, evt)buttonDownCBRotY(obj, cb, evt), 'LineWidth', obj.LineWidth);
            obj.ZCircle = line(ax, zcPts(1,:), zcPts(2,:), zcPts(3,:), 'color', obj.ZColor, 'ButtonDownFcn', @(cb, evt)buttonDownCBRotZ(obj, cb, evt), 'LineWidth', obj.LineWidth);
            
            % Draw lines
            obj.XAxle = line(ax, xaPts(1,:), xaPts(2,:), xaPts(3,:), 'color', obj.XColor, 'LineWidth', obj.LineWidth, 'ButtonDownFcn', @(cb, evt)buttonDownCBTransX(obj, cb, evt));
            obj.YAxle = line(ax, yaPts(1,:), yaPts(2,:), yaPts(3,:), 'color', obj.YColor, 'LineWidth', obj.LineWidth, 'ButtonDownFcn', @(cb, evt)buttonDownCBTransY(obj, cb, evt));
            obj.ZAxle = line(ax, zaPts(1,:), zaPts(2,:), zaPts(3,:), 'color', obj.ZColor, 'LineWidth', obj.LineWidth, 'ButtonDownFcn', @(cb, evt)buttonDownCBTransZ(obj, cb, evt));
            
            % Draw axle grips
            obj.XAxleGrip = surface(ax, smallSphereX + xaPts(1,2), smallSphereY + xaPts(2,2), smallSphereZ + xaPts(3,2), 'linestyle', 'none', 'facecolor', obj.XColor, 'facelighting', 'gouraud', 'ButtonDownFcn', @(cb, evt)buttonDownCBTransX(obj, cb, evt));
            obj.YAxleGrip = surface(ax, smallSphereX + yaPts(1,2), smallSphereY + yaPts(2,2), smallSphereZ + yaPts(3,2), 'linestyle', 'none', 'facecolor', obj.YColor, 'facelighting', 'gouraud', 'ButtonDownFcn', @(cb, evt)buttonDownCBTransY(obj, cb, evt));
            obj.ZAxleGrip = surface(ax, smallSphereX + zaPts(1,2), smallSphereY + zaPts(2,2), smallSphereZ + zaPts(3,2), 'linestyle', 'none', 'facecolor', obj.ZColor, 'facelighting', 'gouraud', 'ButtonDownFcn', @(cb, evt)buttonDownCBTransZ(obj, cb, evt));
            
            % Draw global translation object
            obj.GlobalTransGrip = surface(ax, globalTransX, globalTransY, globalTransZ, 'linestyle', 'none', 'facecolor', obj.GlobalColor, 'facelighting', 'gouraud', 'ButtonDownFcn', @(cb, evt)buttonDownCBMoveCartesian(obj, cb, evt));
            
            % Update the marker's position
            obj.MarkerPose = markerPose;
        end
        
        function delete(obj)
            %delete Destructor
            
            delete(obj.GlobalTransGrip);
            
            delete(obj.XCircle);
            delete(obj.XAxle);
            delete(obj.XAxleGrip);
            
            delete(obj.YCircle);
            delete(obj.YAxle);
            delete(obj.YAxleGrip);
            
            delete(obj.ZCircle);
            delete(obj.ZAxle);
            delete(obj.ZAxleGrip);
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
            
            % Get scaled circle, axle, and sphere data
            [xcPts, ycPts, zcPts, xaPts, yaPts, zaPts] = obj.getCirclePts(markerPose);
            [smallSphereX, smallSphereY, smallSphereZ] = obj.getSpherePts;
            [globalTransX, globalTransY, globalTransZ] = obj.getGlobalTransSurfPts(markerPose);
            
            % Update HG X, Y, and Z data for the X Assembly
            obj.update2DHGData(obj.XCircle, xcPts);
            obj.update2DHGData(obj.XAxle, xaPts);
            obj.update3DHGData(obj.XAxleGrip, smallSphereX + xaPts(1,2), smallSphereY + xaPts(2,2), smallSphereZ + xaPts(3,2));
            
            % Update HG X, Y, and Z data for the Y Assembly
            obj.update2DHGData(obj.YCircle, ycPts);
            obj.update2DHGData(obj.YAxle, yaPts);
            obj.update3DHGData(obj.YAxleGrip, smallSphereX + yaPts(1,2), smallSphereY + yaPts(2,2), smallSphereZ + yaPts(3,2));
            
            % Update HG X, Y, and Z data for the Z Assembly
            obj.update2DHGData(obj.ZCircle, zcPts);
            obj.update2DHGData(obj.ZAxle, zaPts);
            obj.update3DHGData(obj.ZAxleGrip, smallSphereX + zaPts(1,2), smallSphereY + zaPts(2,2), smallSphereZ + zaPts(3,2));
            
            % Update HG X, Y, and Z data for the global Assembly
            obj.update3DHGData(obj.GlobalTransGrip, globalTransX, globalTransY, globalTransZ);
            
            % Update the marker's position
            obj.MarkerPose = markerPose;
        end

        function set.Visible(obj, isVisible)
            %set.Visible Set the marker as visible or not
            %   The Visible property may be logical or onoffstate, and
            %   indicates whether or not the marker body should be
            %   displayed in the figure axes.

            obj.XAxle.Visible = isVisible;
            obj.YAxle.Visible = isVisible;
            obj.ZAxle.Visible = isVisible;
            obj.XAxleGrip.Visible = isVisible;
            obj.YAxleGrip.Visible = isVisible;
            obj.ZAxleGrip.Visible = isVisible;
            obj.XCircle.Visible = isVisible;
            obj.YCircle.Visible = isVisible;
            obj.ZCircle.Visible = isVisible;
            obj.GlobalTransGrip.Visible = isVisible;
        end

        function isVisible = get.Visible(obj)
            %get.Visible Indicates whether or not the gripper is visible
            %   This method queries the axle grip visibility to indicate
            %   whether the whole marker is visible. This should be
            %   sufficient since there is no user-facing way to otherwise
            %   affect the visibility of only a subset of marker elements.

            isVisible = obj.XAxleGrip.Visible && obj.YAxleGrip.Visible && obj.ZAxleGrip.Visible;
        end
    end
    
    methods (Access = protected)
        
        function [xcPts, ycPts, zcPts, xaPts, yaPts, zaPts] = getCirclePts(obj, T)
            %getCirclePts Get points for the circles and axles on the IK marker
            %   The IK marker includes three circles and axles that are
            %   used for translation in and rotation about X, Y, and Z.
            %   This method computes the points needed to define those
            %   circles and lines given a transform representing the
            %   initial pose of the marker (relative to the figure origin).
            
            % Get data from transform
            R = T(1:3, 1:3);
            p = T(1:3,4);
            
            % Apply scale factor
            circleRadius = obj.ScaleFactor*obj.CircleRadius;
            
            % The offset is applied as a coefficient to the circle radius
            % so that it maintains proportions with scale
            extendedAxle = obj.AxleGripOffsetCoefficent*circleRadius;
            
            % Define basic circle
            theta = 0:pi/32:2*pi;
            circleX = circleRadius*cos(theta);
            circleY = circleRadius*sin(theta);
            circleZ = zeros(1,length(circleX));
            
            % Circles and axles vary by orientation. These are all the same
            % circle, drawn in the XY plane, and rotated into the YZ plane
            % to create the X direction circle, and into the XZ plane to
            % create the Y direction circle. Each circle is used as a
            % marker for rotation about it's designated axis; e.g. if the
            % user clicks on and rotates the Z-Circle, the marker will
            % rotate about the Z axis. This design is key as it ensures
            % that a surface normal can be designed in the marker callbacks
            % that is consistently in the correct direction.
            xcPts = R*eul2rotm([0,pi/2,0])*[circleX; circleY; circleZ] + p;
            ycPts = R*eul2rotm([0,0,-pi/2])*[circleX; circleY; circleZ] + p;
            zcPts = R*[circleX; circleY; circleZ] + p;
            
            xaPts = R*[-circleRadius, 0, 0; extendedAxle, 0, 0]' + p;
            yaPts = R*[0, -circleRadius, 0; 0, extendedAxle, 0]' + p;
            zaPts = R*[0, 0, -circleRadius; 0, 0, extendedAxle]' + p;
        end
        
        function [axleGripSphereX, axleGripSphereY, axleGripSphereZ] = getSpherePts(obj)
            %getSpherePts Get points for the spheres on the IK marker
            %   The IK marker includes a small sphere that is used to grip
            %   and move each axle. This method defines the X, Y, and Z
            %   points needed to recreate that sphere using a "surface"
            %   method.
            
            % Unit sphere
            [sphereX, sphereY, sphereZ] = sphere(40);

            % The grips scale at the same rate as the other markers
            sphereScale = obj.AxleGripSphereRadius;
            
            %Scale the axle grip relative to the line width of the
            %associated marker axles and rotation circles. This is
            %important because the spherical grip lies on the end of the
            %axles, so it's size is really relative to the line thickness
            %(i.e. the LineWidth of the line), rather than the overall
            %length of the axles. Divide by the default line width to
            %ensure a consistent scale.
            lineGripScaleFactor = obj.LineWidth/3;
            axleGripSphereX = lineGripScaleFactor*sphereScale*sphereX;
            axleGripSphereY = lineGripScaleFactor*sphereScale*sphereY;
            axleGripSphereZ = lineGripScaleFactor*sphereScale*sphereZ;
        end
        
        function [transSurfXPts, transSurfYPts, transSurfZPts] = getGlobalTransSurfPts(obj, T)
            
            % Get data from transform
            R = T(1:3, 1:3);
            p = T(1:3,4);
            
            % Circle and axle sizes
            theta = 0:pi/16:2*pi;
            numCirclePts = length(theta);
            circleRadius = obj.ScaleFactor*obj.GlobalTransHandleRadius;
            outerCirclePts = [circleRadius*cos(theta); circleRadius*sin(theta); zeros(1,numCirclePts)];
            
            % Define the X, Y, and Z data for a flat circular surface
            % located at the origin. Each surface has two rows that define
            % the bounds of that data set. For the X and Y data, the two
            % bounds are the origin and the perimeter of the circle. The
            % Z-data is all zeros, which ensures that the surface is flat.
            localXSurfPts = [outerCirclePts(1,:); zeros(1,numCirclePts)];
            localYSurfPts = [outerCirclePts(2,:); zeros(1,numCirclePts)];
            localZSurfPts = zeros(2,numCirclePts);
            
            % Rotate and position the surface
            xyz1 = R*[localXSurfPts(1,:); localYSurfPts(1,:); localZSurfPts(1,:)] + p;
            xyz2 = R*[localXSurfPts(2,:); localYSurfPts(2,:); localZSurfPts(2,:)] + p;
            
            % Rearrange into XData, YData, and ZData
            transSurfXPts = [xyz1(1,:); xyz2(1,:)];
            transSurfYPts = [xyz1(2,:); xyz2(2,:)];
            transSurfZPts = [xyz1(3,:); xyz2(3,:)];
        end
        
    end
    
    methods (Static, Access = private)
        function update2DHGData(hgObject, dataPts)
            
            hgObject.XData = dataPts(1,:);
            hgObject.YData = dataPts(2,:);
            hgObject.ZData = dataPts(3,:);
            
        end
        
        function update3DHGData(hgObject, xdata, ydata, zdata)
            
            hgObject.XData = xdata;
            hgObject.YData = ydata;
            hgObject.ZData = zdata;
            
        end
    end
    
    %% Button callbacks
    
    methods (Access = ?robotics.manip.internal.InternalAccess)
    %% Global Translation Button Callbacks
    
        function buttonDownCBMoveCartesian(obj, hcbo, evt) %#ok<INUSD>
            %buttonDownCBMoveCartesian Button-down callback for cartesian motion
            %   This method is the button-down callback for the global
            %   cartesian motion control. The method gets the axes handle
            %   from the hcbo object (the object that was clicked on),
            %   stores the marker pose at the time it was selected, and
            %   sets the button-move and button-up callbacks.
            
            % Store the marker pose at the instant the marker is first
            % selected
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            
            obj.AxesHandle = hcbo.Parent;
            fig = obj.AxesHandle.Parent;
            fig.WindowButtonMotionFcn = @(cb, evt)buttonMoveCBMoveCartesian(obj, cb, evt);
            fig.WindowButtonUpFcn = @(cb, evt)buttonUpCBMoveCartesian(obj, cb, evt);
            
            % Update the surface color
            obj.GlobalTransGrip.FaceColor = obj.GlobalColorSelected;

            % Trigger any external callbacks
            obj.triggerButtonDownCallbacks();
        end
        
        function buttonMoveCBMoveCartesian(obj, hcbo, evt ) %#ok<INUSD>
            %buttonMoveCBMoveCartesian Button-move callback for cartesian motion
            %   This method is the button-move callback for the global
            %   cartesian motion control. The method relates the current
            %   point being clicked on to the original pose and updates the
            %   IK and configuration accordingly.
            
            % The current point actually returns the two ends of a line
            % along which the user could have clicked (because it's a 2D
            % interface to a 3D world)
            cp = obj.AxesHandle.CurrentPoint;
            
            % Need to determine where the object was clicked on
            intersectLine = cp(2,:) - cp(1,:);
            objCentroidVector = obj.MarkerPoseAtButtonClick(1:3,4)' - cp(1,:);
            projectedCentroid = dot(intersectLine, objCentroidVector)/dot(intersectLine, intersectLine)*intersectLine + cp(1,:);
            
            % Project the centroid onto the intersection
            newMarkerPose = [obj.MarkerPoseAtButtonClick(1:3,1:3) projectedCentroid'; 0 0 0 1];
            
            % Update the marker pose
            obj.updateMarker(newMarkerPose);
            
            % Update the surface color
            obj.GlobalTransGrip.FaceColor = obj.GlobalColorSelected;

            % Trigger any external callback actions
            obj.triggerButtonMoveCallbacks();
        end
        
        function  buttonUpCBMoveCartesian(obj, hcbo, evt ) %#ok<INUSD>
            %buttonMoveCBMoveCartesian Button-up callback for cartesian motion
            %   This method is the button-up callback for the global
            %   cartesian motion control. The method resets callbacks and
            %   appearance changes.
            
            obj.resetHCBOCallbacks(hcbo);
            
            % Update the surface color
            obj.GlobalTransGrip.FaceColor = obj.GlobalColor;
            
            % Trigger any external callbacks
            obj.triggerButtonUpCallbacks();
        end
            
    %% X Translation Button Callbacks
        
        function buttonDownCBTransX(obj, hcbo, evt )
            %buttonDownCBTransX Button-down callback for translation in X
            %   This is the button-down callback for the X-translation
            %   line. Given a handle callback object, hcbo, that has been
            %   clicked on, this callback sets key selection details about
            %   the location of the click in the figure and on the object
            %   and stores the marker pose at the time of the click. It
            %   also updates the move callbacks so they are specific to the
            %   X-translation marker.
            
            obj.registerStartingPointsForLinearMotion(hcbo, evt, [1 0 0]);
            
            % Store the marker pose at the instant the marker is first
            % selected
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            
            fig = obj.AxesHandle.Parent;
            fig.WindowButtonMotionFcn = @(cb, evt)buttonMoveCBTransX(obj, cb, evt);
            fig.WindowButtonUpFcn = @(cb, evt)buttonUpCBTransX(obj, cb, evt);
            
            % Update color of selected objects
            obj.XAxle.Color = obj.XColorSelected;
            obj.XAxleGrip.FaceColor = obj.XColorSelected;

            % Trigger any external callbacks
            obj.triggerButtonDownCallbacks();
        end
        
        function buttonMoveCBTransX(obj, hcbo, evt ) %#ok<INUSD>
            %buttonMoveCBTransX Button-move callback for translation in X
            %   This is the button-move callback for the X-translation
            %   circle. Given a handle callback object, hcbo, that has been
            %   clicked on, this callback updates the line marker position
            %   based on the current position of the mouse. It also
            %   computes the inverse kinematics and updates the
            %   configuration in the backend and on the axes.
            
            % Update the marker's position
            obj.updateLinearMarkerPosition;
            
            % Update color of selected objects
            obj.XAxle.Color = obj.XColorSelected;
            obj.XAxleGrip.FaceColor = obj.XColorSelected;

            % Trigger any external callback actions
            obj.triggerButtonMoveCallbacks();
        end
        
        % x - window button up callback
        function  buttonUpCBTransX(obj, hcbo, evt ) %#ok<INUSD>
            %buttonUpCBTransX Button-up callback for translation in X
            %   This method is the button-up callback for translation in X.
            %   It resets values and appearances to their initial
            %   pre-selected state.
            
            obj.resetHCBOCallbacks(hcbo);
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            
            % Update color of selected objects
            obj.XAxle.Color = obj.XColor;
            obj.XAxleGrip.FaceColor = obj.XColor;
            
            % Trigger any external callbacks
            obj.triggerButtonUpCallbacks();
        end
        
        %% Y Translation Button Callbacks
        
        function buttonDownCBTransY(obj, hcbo, evt )
            %buttonDownCBTransY Button-down callback for translation in Y
            %   This is the button-down callback for the Y-translation
            %   line. Given a handle callback object, hcbo, that has been
            %   clicked on, this callback sets key selection details about
            %   the location of the click in the figure and on the object
            %   and stores the marker pose at the time of the click. It
            %   also updates the move callbacks so they are specific to the
            %   Y-translation marker.
            
            obj.registerStartingPointsForLinearMotion(hcbo, evt, [0 1 0]);
            
            % Store the marker pose at the instant the marker is first
            % selected
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            
            fig = obj.AxesHandle.Parent;
            fig.WindowButtonMotionFcn = @(cb, evt)buttonMoveCBTransY(obj, cb, evt);
            fig.WindowButtonUpFcn = @(cb, evt)buttonUpCBTransY(obj, cb, evt);
            
            % Update color of selected objects
            obj.YAxle.Color = obj.YColorSelected;
            obj.YAxleGrip.FaceColor = obj.YColorSelected;

            % Trigger any external callbacks
            obj.triggerButtonDownCallbacks();
        end
        
        function buttonMoveCBTransY(obj, hcbo, evt ) %#ok<INUSD>
            %buttonMoveCBTransY Button-move callback for translation in Y
            %   This is the button-move callback for the Y-translation
            %   circle. Given a handle callback object, hcbo, that has been
            %   clicked on, this callback updates the line marker position
            %   based on the current position of the mouse. It also
            %   computes the inverse kinematics and updates the
            %   configuration in the backend and on the axes.
            
            % Update the marker's position
            obj.updateLinearMarkerPosition;
                        
            % Update color of selected objects
            obj.YAxle.Color = obj.YColorSelected;
            obj.YAxleGrip.FaceColor = obj.YColorSelected;

            % Trigger any external callback actions
            obj.triggerButtonMoveCallbacks();
        end
        
        % y - window button up callback
        function  buttonUpCBTransY(obj, hcbo, evt ) %#ok<INUSD>
            %buttonUpCBTransY Button-up callback for translation in Y
            %   This method is the button-up callback for translation in Y.
            %   It resets values and appearances to their initial
            %   pre-selected state.
            
            obj.resetHCBOCallbacks(hcbo);
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            
            % Update color of selected objects
            obj.YAxle.Color = obj.YColor;
            obj.YAxleGrip.FaceColor = obj.YColor;
            
            % Trigger any external callbacks
            obj.triggerButtonUpCallbacks();
        end
        
        %% Z Translation Button Callbacks
        
        function buttonDownCBTransZ(obj, hcbo, evt )
            %buttonDownCBTransZ Button-down callback for translation in Z
            %   This is the button-down callback for the Z-translation
            %   line. Given a handle callback object, hcbo, that has been
            %   clicked on, this callback sets key selection details about
            %   the location of the click in the figure and on the object
            %   and stores the marker pose at the time of the click. It
            %   also updates the move callbacks so they are specific to the
            %   Z-translation marker.
            
            obj.registerStartingPointsForLinearMotion(hcbo, evt, [0 0 1]);
            
            % Store the marker pose at the instant the marker is first
            % selected
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            
            fig = obj.AxesHandle.Parent;
            fig.WindowButtonMotionFcn = @(cb, evt)buttonMoveCBTransZ(obj, cb, evt);
            fig.WindowButtonUpFcn = @(cb, evt)buttonUpCBTransZ(obj, cb, evt);
            
            % Update color of selected objects
            obj.ZAxle.Color = obj.ZColorSelected;
            obj.ZAxleGrip.FaceColor = obj.ZColorSelected;

            % Trigger any external callbacks
            obj.triggerButtonDownCallbacks();
        end
        
        function buttonMoveCBTransZ(obj, hcbo, evt ) %#ok<INUSD>
            %buttonMoveCBTransZ Button-move callback for translation in Z
            %   This is the button-move callback for the Z-translation
            %   circle. Given a handle callback object, hcbo, that has been
            %   clicked on, this callback updates the line marker position
            %   based on the current position of the mouse. It also
            %   computes the inverse kinematics and updates the
            %   configuration in the backend and on the axes.
            
            % Update the marker's position
            obj.updateLinearMarkerPosition;
            
            % Update color of selected objects
            obj.ZAxle.Color = obj.ZColorSelected;
            obj.ZAxleGrip.FaceColor = obj.ZColorSelected;

            % Trigger any external callback actions
            obj.triggerButtonMoveCallbacks();
        end
        
        % z - window button up callback
        function  buttonUpCBTransZ(obj, hcbo, evt ) %#ok<INUSD>
            %buttonUpCBTransZ Button-up callback for translation in Z
            %   This method is the button-up callback for translation in Z.
            %   It resets values and appearances to their initial
            %   pre-selected state.
            
            obj.resetHCBOCallbacks(hcbo);
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            
            % Update color of selected objects
            obj.ZAxle.Color = obj.ZColor;
            obj.ZAxleGrip.FaceColor = obj.ZColor;
            
            % Trigger any external callbacks
            obj.triggerButtonUpCallbacks();
        end
    end
    
    %% X Rotation callbacks
    
    methods (Access = ?robotics.manip.internal.InternalAccess)
        % x-axis rotation callback
        function buttonDownCBRotX(obj, hcbo, evt)
            %buttonDownCBRotX Button-down callback for rotation in X
            %   This is the button-down callback for the X-rotation circle.
            %   Given a handle callback object, hcbo, that has been clicked
            %   on, this callback sets key selection details about the
            %   location of the click in the figure and on the object and
            %   stores the marker pose at the time of the click. It also
            %   updates the move callbacks so they are specific to the
            %   X-rotation marker.
            
            % Register starting points for the rotation that will be used
            % in the rotation move callback
            obj.registerStartingPointsForRotation(hcbo, evt);
            
            % Store the marker pose at the instant the marker is first
            % selected
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;

            % Update the button callbacks
            fig = obj.AxesHandle.Parent;
            fig.WindowButtonMotionFcn = @(cb, evt)buttonMoveCBRotX(obj, cb, evt);
            fig.WindowButtonUpFcn = @(cb, evt)buttonUpCBRotX(obj, cb, evt);
            
            % Update color of selected objects
            obj.XCircle.Color = obj.XColorSelected;

            % Trigger any external callbacks
            obj.triggerButtonDownCallbacks();
        end
        
        % x-axis rotation button move callback
        function buttonMoveCBRotX(obj, hcbo, evt) %#ok<INUSD>
            %buttonMoveCBRotX Button-move callback for rotation in X
            %   This is the button-move callback for the X-rotation circle.
            %   Given a handle callback object, hcbo, that has been clicked
            %   on, this callback updates the line marker position based on
            %   the current position of the mouse. It also computes the
            %   inverse kinematics and updates the configuration in the
            %   backend and on the axes.
            
            % Update the marker's rotation
            obj.updateCircleMarkerRotation;
            
            % Update color of selected objects
            obj.XCircle.Color = obj.XColorSelected;

            % Trigger any external callback actions
            obj.triggerButtonMoveCallbacks();
        end
        
        function  buttonUpCBRotX(obj, hcbo, evt ) %#ok<INUSD>
            %buttonUpCBRotX Button-up callback for rotation in X
            %   This method is the button-up callback for rotation in X. It
            %   resets values and appearances to their initial
            %   pre-selected state.

            obj.resetHCBOCallbacks(hcbo);
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            
            % Update color of selected objects
            obj.XCircle.Color = obj.XColor;
            
            % Trigger any external callbacks
            obj.triggerButtonUpCallbacks();
        end
        
        %% Y Rotation callbacks
        function buttonDownCBRotY(obj, hcbo, evt)
            %buttonDownCBRotY Button down Y-rotation callback
            %   This is the button-down callback for the Y-rotation circle.
            %   Given a handle callback object, hcbo, that has been clicked
            %   on, this callback sets key selection details about the
            %   location of the click in the figure and on the object and
            %   stores the marker pose at the time of the click. It also
            %   updates the move callbacks so they are specific to the
            %   Y-rotation marker.
            
            % Register starting points for the rotation that will be used
            % in the rotation move callback
            obj.registerStartingPointsForRotation(hcbo, evt);
            
            % Store the marker pose at the instant the marker is first
            % selected
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;

            % Update the button callbacks
            fig = obj.AxesHandle.Parent;
            fig.WindowButtonMotionFcn = @(cb, evt)buttonMoveCBRotY(obj, cb, evt);
            fig.WindowButtonUpFcn = @(cb, evt)buttonUpCBRotY(obj, cb, evt);
            
            % Update color of selected objects
            obj.YCircle.Color = obj.YColorSelected;

            % Trigger any external callbacks
            obj.triggerButtonDownCallbacks();
        end
        
        function buttonMoveCBRotY(obj, hcbo, evt) %#ok<INUSD>
            %buttonMoveCBRotY Button-move callback for rotation in Y
            %   This is the button-move callback for the Y-rotation circle.
            %   Given a handle callback object, hcbo, that has been clicked
            %   on, this callback updates the line marker position based on
            %   the current position of the mouse. It also computes the
            %   inverse kinematics and updates the configuration in the
            %   backend and on the axes.
            
            % Update the marker's rotation
            obj.updateCircleMarkerRotation;
            
            % Update color of selected objects
            obj.YCircle.Color = obj.YColorSelected;

            % Trigger any external callback actions
            obj.triggerButtonMoveCallbacks();
        end
        
        function  buttonUpCBRotY(obj, hcbo, evt ) %#ok<INUSD>
            %buttonUpCBRotY Button-up callback for rotation in Y
            %   This method is the button-up callback for rotation in Y. It
            %   resets values and appearances to their initial pre-selected
            %   state.
            
            obj.resetHCBOCallbacks(hcbo);
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            
            % Update color of selected objects
            obj.YCircle.Color = obj.YColor;
            
            % Trigger any external callbacks
            obj.triggerButtonUpCallbacks();
        end
        
        %% Z Rotation callbacks
        
        function buttonDownCBRotZ(obj, hcbo, evt)
            %buttonDownCBRotZ Button down Z-rotation callback
            %   This is the button-down callback for the Z-rotation circle.
            %   Given a handle callback object, hcbo, that has been clicked
            %   on, this callback sets key selection details about the
            %   location of the click in the figure and on the object and
            %   stores the marker pose at the time of the click. It also
            %   updates the move callbacks so they are specific to the
            %   Z-rotation marker.
            
            % Register starting points for the rotation that will be used
            % in the rotation move callback
            obj.registerStartingPointsForRotation(hcbo, evt);
            
            % Store the marker pose at the instant the marker is first
            % selected
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;

            % Update the button callbacks
            fig = obj.AxesHandle.Parent;
            fig.WindowButtonMotionFcn = @(cb, evt)buttonMoveCBRotZ(obj, cb, evt);
            fig.WindowButtonUpFcn = @(cb, evt)buttonUpCBRotZ(obj, cb, evt);
            
            % Update color of selected objects
            obj.ZCircle.Color = obj.ZColorSelected;

            % Trigger any external callbacks
            obj.triggerButtonDownCallbacks();
        end
        
        function buttonMoveCBRotZ(obj, hcbo, evt) %#ok<INUSD>
            %buttonMoveCBRotZ Button-move callback for rotation in Z
            %   This is the button-move callback for the Z-rotation circle.
            %   Given a handle callback object, hcbo, that has been clicked
            %   on, this callback updates the line marker position based on
            %   the current position of the mouse. It also computes the
            %   inverse kinematics and updates the configuration in the
            %   backend and on the axes.
            
            % Update the marker's rotation
            obj.updateCircleMarkerRotation;
            
            % Update color of selected objects
            obj.ZCircle.Color = obj.ZColorSelected;

            % Trigger any external callback actions
            obj.triggerButtonMoveCallbacks();
        end
        
        function buttonUpCBRotZ(obj, hcbo, evt ) %#ok<INUSD>
            %buttonUpCBRotZ Button-up callback for rotation in Z
            %   This method is the button-up callback for rotation in Z. It
            %   resets values and appearances to their initial
            %   pre-selected state.

            obj.resetHCBOCallbacks(hcbo);
            obj.MarkerPoseAtButtonClick = obj.MarkerPose;
            
            % Update color of selected objects
            obj.ZCircle.Color = obj.ZColor;
            
            % Trigger any external callbacks
            obj.triggerButtonUpCallbacks();
        end
    end

    methods (Access = private)
        function triggerButtonDownCallbacks(obj)
            %triggerButtonDownCallbacks Trigger external callbacks when marker is clicked on

            obj.ExternalButtonDownCB(obj.MarkerPose);
        end

        function triggerButtonMoveCallbacks(obj)
            %triggerButtonMoveCallbacks Trigger external callbacks when marker is selected and dragged

            obj.ExternalButtonMoveCB(obj.MarkerPose);
        end

        function triggerButtonUpCallbacks(obj)
            %triggerButtonUpCallbacks Trigger external callbacks when marker is released

            obj.ExternalButtonUpCB(obj.MarkerPose);
        end
    end
    
end
