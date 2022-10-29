classdef DHUtilities < robotics.manip.internal.InternalAccess
    % This class is for internal use only and may be removed in a future release
    
    %DHUtilities Collection of utility methods for constructing and analyzing robots using Denavit-Hartenberg parameterizations
    
    %   Copyright 2021 The MathWorks, Inc.
    
    methods (Static)        
        function [foundValidDHParams, conversionDetails] = searchForDHParameterization(originalSixDoFChain, tol)
            %searchForDHParameterization Attempt to find a DH parameterization for a rigidBodyTree object
            %   Given a rigid body tree object representing six-DoF chain
            %   (one branch), and a tolerance, this method searches for a
            %   representation using the extended Denavit-Hartenberg
            %   system, which consists of a 6x4 matrix of DH parameters,
            %   as well as a 6x2 matrix of Z-Screws, described in detail
            %   below. The tolerance is used for rounding during numerical
            %   comparisons. The method returns a flag that is true if a DH
            %   parameterization is found for the chain, along with a
            %   structure of conversion details. The structure has the
            %   following fields:
            %
            %      WorldToDHBase            - A 4x4 homogeneous transform
            %                                 matrix that maps the robot
            %                                 origin/base frame to the
            %                                 world origin/world frame
            %
            %      LinkTransforms           - A 4x4x6 matrix of homogeneous
            %                                 transforms that map represent
            %                                 the transform from Jdh to Fdh
            %                                 for the ith frame
            %
            %      DHParams                 - A 6x4 matrix of DH
            %                                 parameters. If a compatible
            %                                 DH parameter could not be
            %                                 found, the value will be NaN
            %
            %      ZScrews                  - A 6x2 matrix of ZScrews,
            %                                 representing the offsets
            %                                 needed to map the source
            %                                 frames to their DH
            %                                 counterparts as translation
            %                                 and rotation about the Z
            %                                 axes. See details below for
            %                                 more information, or refer to
            %                                 the specific methods tasked
            %                                 at extracting these
            %                                 parameters for the precise
            %                                 formulation.
            %
            %      EndEffectorTranslation   - The 4x4 homogeneous transform
            %                                 that maps the end effector
            %                                 position to the point
            %                                 where the last three axes
            %                                 intersect. The mapping is a
            %                                 4x4 matrix, but it is used
            %                                 exclusively to translate the
            %                                 end effector position to the
            %                                 intersection point position;
            %                                 orientation is handled
            %                                 independently.
            %   
            %   The Z-Screws represent the offsets in theta and d of the
            %   original frame relative to its DH frame. Each row of the
            %   Z-Screw matrix corresponds to a DH frame, with the two
            %   elements:
            %
            %      - The first element, dOffset, represents the translation
            %        along the positive Z-axis that the ith DH frame must
            %        be moved to get to the corresponding actual frame
            %
            %      - The second element, thetaOffset, represents the angle
            %        about the positive Z-axis that the ith DH frame must
            %        be rotated to get to the corresponding actual frame
            %
            %   References for Denavit-Hartenberg parameters the Z-Screw format:
            %      [1] K. Lynch and F. Park, Modern Robotics: Mechanics, 
            %          Planning, and Control. Cambridge, UK: Cambridge 
            %          University Press, 2017.
            %      [2] M. Spong, S. Hutchinson, S. Vidyasagar, Robot 
            %          Modeling and Control. John Wiley & Sons, Inc., 2006.
            %      [3] G. Bin Hammam, "Whole-Body Motion Retargeting for 
            %          Humanoids," PhD Dissertation, The Ohio State
            %          University, 2014.
            
            import robotics.manip.internal.AnalyticalIKHelpers
            import robotics.manip.internal.DHUtilities
            
            % The function starts with a tree that has arbitrary joint
            % axes, originalSixDoFChain. This tree is a chain, meaning it
            % has only one branch, and has six degrees of freedom, though
            % it may have more bodies as long as they are fixed. 
            
            % The first step to get to a DH parameterization is to re-model
            % the tree such that the base is co-located with the first
            % joint and all joints rotates around z axes (the other new
            % joint frames are co-located with their old frames, but may
            % have a different orientation). In other words, given a
            % configuration vector q, the position of the end effector
            % relative to the base on the old tree can be found using:
            %      baseToEE = getTransform(oldTree, q, eeBodyName)
            % For a robot built exclusively from the link parameters, the
            % same output would be given by:
            %      baseToEE = worldToNewTreeBase*getTransform(treeFromLinkParameters, q, eeBodyName)
            % Note that this method also returns a second value, newTree,
            % which includes the worldToNewTreeBase as a fixed joint
            % attached to the base, so for that output, the same
            % configuration could be found using:
            %      baseToEE = getTransform(newTree, q, eeBodyName)
            % The method returns the parameterization in a structure
            % containing a set of six transforms and the
            % worldToNewTreeBase transform.
            paramsForTreeWithZAxisJoints = DHUtilities.assignZAxisJoints(originalSixDoFChain, tol);
            
            % Given the matrix representation used to create
            % treeWithZAxisJoints, search for a set of DH parameters and
            % ZScrews that can produce a kinematically equivalent robot
            % when the ZScrews are incorporated into the configuration
            % vector.
            [foundValidDHParams, paramsForTreeModeledWithDH] = DHUtilities.searchForDHParameterizationForZAxisTree(paramsForTreeWithZAxisJoints.LinkTforms, tol);
            
            % Parameterization details
            conversionDetails = struct();
            conversionDetails.WorldToDHBase = paramsForTreeWithZAxisJoints.WorldToBaseTform;
            conversionDetails.LinkTransforms = paramsForTreeModeledWithDH.LinkTransforms;
            conversionDetails.DHParams = paramsForTreeModeledWithDH.DHParams;
            conversionDetails.ZScrews = paramsForTreeModeledWithDH.ZScrews;
            
            % Transforms that relate the end effector to the last joint and
            % to the intersecting axes
            conversionDetails.F6ToEETform = paramsForTreeModeledWithDH.EETform;
            conversionDetails.J6ToEETform = paramsForTreeModeledWithDH.LinkTransforms(:,:,6)*paramsForTreeModeledWithDH.EETform;
            conversionDetails.EndEffectorTranslation = paramsForTreeModeledWithDH.LinkTransforms(:,:,5)*conversionDetails.J6ToEETform;
        end  
    end
    
    %% Helper methods
    
    methods (Static, Access = ?robotics.manip.internal.InternalAccess)
        
        function [detailsStruct, newTree] = assignZAxisJoints(sixDoFChain, tol)
            %assignZAxisJoints Model a rigidBodyTree such that all the joints rotate about the z-axis
            %   Given a six-DoF revolute serial chain with arbitrary joint axes,
            %   this method reparameterizes the robot such that all the
            %   joint frames rotate about a Z axis. Then the pose of the
            %   end effector relative to the original six-DoF chain's base
            %   frame can always be found as:
            %      baseToEE = worldToNewTreeBase*rotz(q1)*T1*rotz(q2)*T2...*rotz(q6)*T6
            %   where q1 to q6 are the joint configuration values, rotz(q)
            %   represents a 4x4 matrix indicating rotation about Z of q,
            %   and T1 to T6 are fixed 4x4 homogeneous transform matrices
            %   (that this method returns). The method outputs the values
            %   for the parameterization in the structure detailsStruct
            %   with the following fields:
            %      WorldToBaseTform   - A frame that relates the original
            %                           base (i.e. the world) to the new
            %                           base colocated with the first joint
            %      LinkTforms         - A 4x4x6 matrix containing T1 to T6
            %
            %   A second argument creates a new tree from these parameters.
            %   Instead of relying on pre-multiplication by the world-to-
            %   new-tree-base frame, this tree just adds an intermediate
            %   fixed "base" link co-located with the first joint, so that
            %   all the parameters are contained in the resultant tree. The
            %   tree output models the tree as detailed in the comments,
            %   with two frames per body: a proximal moving frame, and a
            %   distal fixed frame.
            
            import robotics.manip.internal.AnalyticalIKHelpers
            import robotics.manip.internal.DHUtilities
            
            tfTree = sixDoFChain.TreeInternal.forwardKinematics(zeros(sixDoFChain.TreeInternal.VelocityNumber, 1));
            [worldToNewTreeBase, origBaseFrameToNewBaseFrame] = DHUtilities.getFrameMappingWorldToFirstDHFrame(sixDoFChain, tfTree, tol);
            
            % Initialize key variables used in the conversion
            distalToProximalLinkFrameTforms = zeros(4,4,6);       % There are 6 4x4 transforms for a 6-DoF robot
            
            %Initialize origToNewDistalFrameMapping for i = 0
            origToNewDistalFrameMapping = origBaseFrameToNewBaseFrame;
            for i = 1:5
                % Compute the transform that maps the ith joint frame (the
                % proximal frame on the link) to a joint frame that rotates
                % about the Z axis. This is the inverse of one that does
                % the same for the collocated fixed reference frame (the
                % distal frame of the previous link), because that frame is
                % mapped using the mapping of the subsequent moving frame
                % (this one) when the configuration is zero (See below).
                origToNewProximalFrameMapping = robotics.manip.internal.tforminv(origToNewDistalFrameMapping);
                
                % Compute the link transform, i.e. the transform that
                % relates the frame at the distal end of the joint to that
                % at the proximal end. Since the fixed frame coincides with
                % the upstream joint and the values are equal at theta = 0,
                % a consistent way to compute this value is to compute the
                % position of joint i+1 in the joint i frame.
                linkTransform = AnalyticalIKHelpers.getJointTransform(sixDoFChain, i+1, i, tfTree);
                
                % Compute the transform that maps the distal frame on the ith link (the
                % fixed frame on the link) to one where the axis that
                % coincides with the subsequent joint's axis of rotation is
                % now the Z-axis. When the robot is in the zero
                % configuration, the next joint 's proximal frame will have
                % the same pose as the distal-fixed frame on this link, so
                % this action is really mapping the distal frame on this
                % link to one where the Z axis aligns with the axis of
                % rotation of the next frame's moving joint.
                nextJointBodyIdx = AnalyticalIKHelpers.getBodyIndexFromJointIndex(sixDoFChain, i+1);
                nextJointAxes = sixDoFChain.Bodies{nextJointBodyIdx}.Joint.JointAxis;
                origToNewDistalFrameMapping = DHUtilities.getMappingFromCurrentFrameToZJointAxis(nextJointAxes, tol); % Maps the frame to one where joint axis is about Z
                
                % Compute equivalent frame
                distalToProximalLinkFrameTforms(:,:,i) = origToNewProximalFrameMapping*linkTransform*origToNewDistalFrameMapping;
            end
            
            if sixDoFChain.NumBodies > 6
                % Robots modeled using setFixedTransform(TF) will have at
                % least six, and possibly more bodies, and the end effector
                % body will be the last one in the chain
                sixthJointBodyName = sixDoFChain.BodyNames{AnalyticalIKHelpers.getBodyIndexFromJointIndex(sixDoFChain, 6)};
                lastBodyName = sixDoFChain.BodyNames{end};
                lastLinkTransform = getTransform(sixDoFChain, 0*sixDoFChain.homeConfiguration, lastBodyName, sixthJointBodyName);
            else
                % If this robot was modeled using DH, the last transform
                % captures the offset. If this was not modeled with DH,
                % this value will anyway be eye(4), which matches the
                % assumption that when there are just six bodies, the end
                % effector is collocated with the last joint.
                lastLinkTransform = sixDoFChain.Bodies{6}.Joint.ChildToJointTransform;
            end
            origToNewProximalFrameMapping = robotics.manip.internal.tforminv(origToNewDistalFrameMapping);
            distalToProximalLinkFrameTforms(:,:,6) = origToNewProximalFrameMapping*lastLinkTransform;
            
            % First output
            detailsStruct = struct();
            detailsStruct.WorldToBaseTform = worldToNewTreeBase;
            detailsStruct.LinkTforms = distalToProximalLinkFrameTforms;
            
            if nargout > 1
                % Construct a tree from these frames
                newTree = DHUtilities.buildTreeFromFrames(distalToProximalLinkFrameTforms, worldToNewTreeBase);
            end            
        end
        
        function [foundValidDHParams, detailsStruct] = searchForDHParameterizationForZAxisTree(linkTForms, tol)
            %searchForDHParameterizationForZAxisTree Extract DH from a 6-DoF tree with revolute z-axis joints
            %   This function accepts the link transforms, T1 to T6, that
            %   parameterize a six-DoF rigidBodyTree object. Given an end
            %   effector at the distal frame of the last link, the pose is
            %   found using the following matrix multiplication:
            %      eePose = rotz(q1)*T1*rotz(q2)*T2*...*rotz(q6)*T6
            %   This function then strives to extract a DH representation
            %   for this robot. To ensure that all transforms can be
            %   accounted for using DH, two extra parameters, the ZScrews,
            %   are included. These two parameters for a screw along the Z
            %   axis and provide the missing 2 DoF to allow the frame used
            %   for DH to be moved relative to the actual frame location to
            %   ensure a DH parameterization is found:
            %      - thOffset is added to the configuration input as an
            %        offset to the applicable value of theta
            %      - dOffset shifts the frame along the Z-axis relative to
            %        its position in the original tree
            
            %   This method uses the following approach: In the existing
            %   parameterization, i.e. the predefined frames, each frame is
            %   a guess at the pose of the frame in the DH robot. Then the
            %   goal, for each transform Ti that relates frames i and i+1,
            %   and find the values of thOffset, dOffset, and the
            %   parameters d, a, and alpha that define the transformation
            %   between two subsequent frames. Here thOffset refers to the
            %   revolute part of the ZScrew, i.e. for th ith joint, the sum
            %   of thPredefToDH_(i-1) and thPredefToDH_i. Altogether, this
            %   means the transforms between two predefined frames have the
            %   following form:
            %
            %      i_T_(i+1) = rotz(thPredefToDH_i)*transz(d_i)*transx(a_i)*rotx(alpha_i)*rotz(thDHToPredef_(i+1))*transz(dOffset_(i+1))
            %
            %   For example for two frames that start with the predefined
            %   Frames i and (i+1), both of which rotate about Z, the
            %   following equation shows where the new modified frames i
            %   and i+1 appear. These are the DH frames used by the
            %   modified robot:
            %
            %   rotz(q_i) * i^T_(i+1) * rotz(q_(i+1)) (i+1)^T_(i+2) = ...
            %
            %   rotz(q_i) * -dOffset_i * [ rot(thPredefToDH_i) * 
            %                                                    transl(d_i) * 
            %                                                    transl(a_i) * 
            %                                                    rot(alpha_i) ] * rot(thDHToPredef_i_(i+1)) * dOffset_(i+1) * 
            %                                                                                                                 rotz(q_(i+1)) * -dOffset_(i+1) * [ rot(thPredefToDH_(i+1)) * 
            %                                                                                                                                                                              transl(d_(i+1)) *
            %                                                                                                                                                                              transl(a_(i+1)) *
            %                                                                                                                                                                              rot(alpha_(i+1)) ] * rot(thDHToPredef_(i+1)) * dOffset_(i+1)
            %   
            %            ^                                    ^                                                                             ^                                            ^
            %            |                                    |                                                                             |                                            |
            %        Predefined                          Modified (DH)                                                                  Predefined                                   Modified (DH)
            %        Frame {i}                            Frame {i}                                                                     Frame {i+1}                                   Frame {i+1}
            %
            %   Then the DH parameters are formed from the translation d
            %   and a and rotation alpha, and the ZScrews consist of
            %   offsets dOffset and thetaOffset, where each thetaOffset is
            %   the sum of thDHToPredef_i and thPredefToDH_(i+1).
            %
            %   This means that if we have a robot constructed with the
            %   predefined frames, the position of frame i+2 relative to
            %   frame i would be:
            %      i_T_(i+2) = getTransform(robotWithPredefinedFrames, q, body(i+2).Name)
            %   and for the new robot using the modified (DH) frames, the
            %   same pose could be achieved using:
            %      i_T_(i+2) = getTransform(robotWithModifiedFrames, q+ZScrews(:,2), body(i+2).Name)
            %
            %   Start by assuming that the position of the first frame
            %   is known, i.e. dOffset_1 = 0, but it can still have some
            %   theta Offset that will be accounted for in the
            %   configuration. Then for the ith frame, the goal is to
            %   determine a necessary pose of the (i+1)th frame such that
            %   the robot as modeled up to this point can be represented
            %   using DH:
            %
            %      1) For the ith transform Ti, assume thPredefToDH_i and
            %         dOffset_i are known (from the previous iteration)
            %
            %      2) Refer to the DHUtilities.getDHParamsFromTransform 
            %         method for details on this step. The following is an
            %         abbreviated summary: 
            %
            %         Assume that the pose of the (i+1)th frame is a guess,
            %         and it can be shifted along z or rotated about z to
            %         make the robot up to this frame parameterizable with
            %         DH. To do so, we will find dOffset_(i+1) and
            %         thPredefToDH_i+1 by including those terms in the
            %         transform (they can be freely shifted around the
            %         joint since the joint is along the z-axis anyway.
            %         This works out to solving the equation above for the
            %         ith and (i+1)th frames. Instead of solving directly
            %         from the transform, the Denavit-Hartenberg process
            %         will be followed to determine the parameters based on
            %         their role.
            %
            %       3) Carry the two terms that affect the position of the
            %          ith frame (thPredefToDH_(i+1) and dOffset_(i+1))
            %          into the next calculation. In the calculation below,
            %          thPredefToDH_(i+1) isn't mapped back into the
            %          transform i_T_(i+1). Instead, thPredefToDH_(i+1) is
            %          used as an offset for computing the ZScrews in the
            %          next iteration.
            
            import robotics.manip.internal.AnalyticalIKHelpers
            import robotics.manip.internal.DHUtilities
            
            % Initialize key variables used in the conversion
            newLinkTforms = zeros(4,4,6);       % There are 6 4x4 transforms for a 6-DoF robot
            dhParams = zeros(6,4);
            zscrews = zeros(6,2);
            
            % This utility currently only supports revolute robots, which
            % means that for any supported robot, the theta column of the
            % DH parameters will always be zero (this position is intended
            % for the moving theta variable, but the role of angle that
            % rotates Z to the common normal is instead computed as part of
            % the Z-screw inside DHUtilities.getDHParamsFromTransform.
            dhParams(:,4) = 0; %theta column
            
            distalFrameOriginalToShiftedMapping = eye(4);
            for i = 1:size(linkTForms,3)
                linkTform = linkTForms(:,:,i);
                
                % If the previous frame was found to have a dOffset,
                % meaning the placement of the original frame is shifted
                % relative to the new DH-friendly frame, the relationship
                % between the two frames must be updated to include this
                % offset.
                newLinkTforms(:,:,i) = distalFrameOriginalToShiftedMapping*linkTform;
                
                % Try to extract DH parameters and ZScrews
                [dhParams(i,1:3), thPredefToDH, nextDOffset, thDHToNextPredef] = DHUtilities.getDHParamsFromTransform(newLinkTforms(:,:,i), tol);
                                
                zscrews(i,2) = robotics.internal.wrapToPi(zscrews(i,2) + thPredefToDH);
                if i < size(linkTForms,3)
                    zscrews(i+1,:) = [nextDOffset thDHToNextPredef];
                end                    
                distalFrameOriginalToShiftedMapping = trvec2tform([0 0 nextDOffset]);
            end
            
            detailsStruct = struct;
            detailsStruct.DHParams = dhParams;
            detailsStruct.ZScrews = zscrews;
            detailsStruct.EETform = trvec2tform([0 0 nextDOffset])*axang2tform([0 0 1 thDHToNextPredef]);
            detailsStruct.LinkTransforms = newLinkTforms;
            
            % Check whether tree is compatible
            foundValidDHParams = ~any(isnan(dhParams), 'all');
        end
        
        function tree = buildTreeFromFrames(frames, worldTform)
            %buildTreeFromFrames Construct a rigidBodyTree given the jointToParentFrames
            %   Given a 4x4xN set of homogeneous joint transforms, this
            %   method constructs a rigidBodyTree where the ith body is
            %   attached using the ith transform as the jointToParent
            %   transform. This model co-locates the first joint with the
            %   base, but if a worldTform is provided, an intermediate
            %   fixed joint is used to map the base to first joint, with
            %   jointToParent transform given by worldTform.
            
            if nargin < 2
                worldTform = eye(4);
            end
            
            tree = rigidBodyTree('DataFormat','column');
            tree.BaseName = 'world';
            
            % Create a fixed link that relates the world frame to the robot
            % base frame, which will be colocated with the first joint
            robotBase = rigidBody('robot_base');
            baseJoint = rigidBodyJoint('baseJoint','fixed');
            baseJoint.setFixedTransform(worldTform);
            robotBase.Joint = baseJoint;
            tree.addBody(robotBase, tree.BaseName);
            parentName = robotBase.Name;
            
            for i = 1:size(frames,3)
                % Add a moving frame at the proximal end of the link
                bodyProxFrame = rigidBody(sprintf('body_%i_proximal',i));
                bodyProxFrame.Joint = rigidBodyJoint(sprintf('j%i',i), 'revolute');
                tree.addBody(bodyProxFrame, parentName);
                
                % Add a fixed frame at the distal end of the link
                bodyDistFrame = rigidBody(sprintf('body_%i_distal',i));
                bodyDistJoint = rigidBodyJoint(sprintf('j%i_fixed',i), 'fixed');
                bodyDistJoint.setFixedTransform(frames(:,:,i));
                bodyDistFrame.Joint = bodyDistJoint;
                tree.addBody(bodyDistFrame, bodyProxFrame.Name);
                parentName = bodyDistFrame.Name;
            end
        end
        
        function [dhParam, thPredefToDH, nextDOffset, thDHToNextPredef, validationTransform] = getDHParamsFromTransform(tformJToF, tol)
            %getDHParamsFromTransform Get DH parameters from the transform that relates the frame collocated with the joint to the distal frame
            %   This method converts an arbitrary transform that relates
            %   two frames with z-axis joint vectors using the DH transform
            %   and Z-screws. The method returns a 3-element vector of DH
            %   parameters [a alpha d], as well as the terms thPredefToDH,
            %   nextDOffset, and thDHToNextPredef, which are terms that
            %   relate the pose of the two frames to their position needed
            %   for DH. These variables are used as follows:
            %
            %      1. rotx(thPredefToDH)  - The rotation about Z that 
            %                               rotates a frame's X-axis to an
            %                               x-axis that is a common normal
            %                               with the next frame's Z-axis.
            %                               As a consequence, this is the
            %                               action that rotates the frame
            %                               from its predefined orientation
            %                               to its DH orientation.
            %
            %      2. transz(d)           - Part of the DH transform; the
            %                               translation along the first
            %                               frame's Z-axis 
            %
            %      3. transx(a)           - Part of the DH transform; the
            %                               translation along the first
            %                               frame's X-axis
            %
            %      4. rotx(alpha)         - Part of the DH transform; the
            %                               rotation about the common
            %                               normal
            %
            %      5. transz(nextDOffset) - The translation along the
            %                               second frame's Z axis that
            %                               moves the frame from its
            %                               necessary position for DH to
            %                               its actual position along the
            %                               Z-axis
            %
            %      6. rotz(thDHToNextPredef)   - The rotation about the
            %                                    second frame's Z axis that
            %                                    relates the frame from its
            %                                    necessary position for DH
            %                                    to its next predefined
            %                                    position (i.e. the pose of
            %                                    the next predefined moving
            %                                    frame when q = 0).
            
            import robotics.manip.internal.DHUtilities
            
            % Initialize outputs set at end
            nextDOffset = 0;
            thDHToNextPredef = 0;
            
            % Compute the common normal between the two frames, which is
            % defined by two points: the intersection of the common normal
            % with the frame 1 Z axis, and the intersection of the common
            % normal with the frame 2 Z axis
            nextFrameOriginInCurrentFrame = tformJToF(1:3,4);
            nextFrameZAxisInCurrentFrame = tformJToF(1:3,1:3)*[0 0 1]';
            currentFrameOrigin = [0, 0, 0]';
            currentFrameZAxis = [0, 0, 1]';
            [commonNormalZ1Intersection, commonNormalZ2Intersection] = DHUtilities.findCommonNormalForDH(currentFrameOrigin, currentFrameZAxis, nextFrameOriginInCurrentFrame, nextFrameZAxisInCurrentFrame, tol);
            
            commonNormalUnitVector = DHUtilities.determineDHCommonNormalUnitVector(commonNormalZ1Intersection, commonNormalZ2Intersection, nextFrameZAxisInCurrentFrame, tol);
            
            if isEqualWithinTolerance(commonNormalUnitVector(:), [-1 0 0]', tol)
                % If the common normal is exactly opposite the X-axis, the
                % result is valid, but it will cause an unnecessary
                % rotation of pi about the axis, which can cause downstream
                % issues. The minimize the value of thetaOffset, flip the
                % common normal direction so it matches the current frame's
                % X-axis
                commonNormalUnitVector = [1 0 0]';
            end

            % Compute the rotation about Z that would be needed to rotate
            % the current frame from its current X-axis to the common
            % normal and use that value to get an updated unit vector that
            % handles edge cases
            thPredefToDH = DHUtilities.computeRotationBetweenTwoVectorsAboutAxis([1 0 0], commonNormalUnitVector, [0 0 1], tol);
                    
            if isnan(thPredefToDH)
                dhParam = [NaN NaN NaN];
                return;
            end
            
            % Parameter a -- the translation along x -- is the length of
            % the common normal. Sign is positive if the common normal
            % points at the frame 2 Z axis, and negative otherwise. Use the
            % dot product to check sign.
            a = dot(commonNormalZ2Intersection - commonNormalZ1Intersection, commonNormalUnitVector);
            
            % Parameter d -- the translation along z -- is the distance
            % from the current frame's origin ([0 0 0] wrt to the current
            % frame)  to the start of the common normal along z
            d = dot([0 0 1], commonNormalZ1Intersection);
            
            % Compute parameter alpha, the rotation about X (i.e. the new
            % common normal), needed to rotate the Z axis of frame 1 to the
            % new Z axis (that of frame 2)
            alpha = DHUtilities.computeRotationBetweenTwoVectorsAboutAxis([0 0 1], nextFrameZAxisInCurrentFrame, commonNormalUnitVector, tol);
            
            % Compute dOffset, which constitutes the translation component
            % of the (i+1)th z-screw, and is the difference from the end
            % point of the common normal and the (i+1)th frame's origin.
            % To be consistent with DH, the next frame would have to be
            % positioned at the end of the common normal, so this piece of
            % the z-screw maps the actual frame to this one.
            nextDOffset = dot(nextFrameZAxisInCurrentFrame, nextFrameOriginInCurrentFrame-commonNormalZ2Intersection);
            
            nextFrameXAxisInCurrentFrame = tformJToF(1:3,1:3)*[1 0 0]';
            thDHToNextPredef = DHUtilities.computeRotationBetweenTwoVectorsAboutAxis(commonNormalUnitVector, nextFrameXAxisInCurrentFrame, nextFrameZAxisInCurrentFrame, tol);
            
            dhParam = [a alpha d];
            
            % As an added check, compute the transform from these DH
            % parameters and verify that it matches the input transform
            % that this method was seeking to recreate using DH & Z-screw
            validationTransform = DHUtilities.computeTransformFromDHAndZScrews(thPredefToDH, a, alpha, d, nextDOffset, thDHToNextPredef);
            areDHParamsValid = isEqualWithinTolerance(validationTransform, tformJToF, tol);
            if ~areDHParamsValid
                dhParam = [NaN NaN NaN];
            end
        end
        
        function tform = getMappingFromCurrentFrameToZJointAxis(jointAxis, tol)
            %getMappingFromCurrentFrameToZJointAxis Get transform that maps joint axis to Z axis
            %   Returns a transform that rotates the joint axis to the Z axis
            %   needed for DH:
            %      [jointAxis 1]*Tform = [0 0 1 1];
            %   This also implies:
            %      TForm*[0; 0; 1; 1] = [jointAxis' 1]
            %   Because this method presently uses hard coded mappings, it
            %   currently limits the overall DH extraction to just those
            %   robots where the joint axes fall along the X, Y, or Z axes.
            
            if any(jointAxis < -tol)
                jointAxis = abs(jointAxis);
                flipMat = [1 0 0; 0 -1 0; 0 0 -1]; %axang2tform([1 0 0 pi])
            else
                flipMat = eye(3);
            end
            
            if isEqualWithinTolerance(jointAxis, [1 0 0], tol)
                rotMat = [0 0 1; 1 0 0; 0 1 0];
            elseif isEqualWithinTolerance(jointAxis, [0 1 0], tol)
                rotMat = [0 1 0; 0 0 1; 1 0 0];
            elseif isEqualWithinTolerance(jointAxis, [0 0 1], tol)
                rotMat = eye(3);
            else
                rotMat = NaN;
                tform = NaN;
            end
            
            if ~isnan(rotMat)
                % Flip axes if needed
                rotMat = rotMat*flipMat;
                
                % Assemble transform
                tform = [rotMat zeros(3,1); 0 0 0 1];
            end
        end
        
        function [worldToDHBase, tformF0ToDh0] = getFrameMappingWorldToFirstDHFrame(originalTree, tfTree, tol)
            %getFrameMappingWorldToFirstDHFrame Returns the matrix that maps the origin of the input robot to the origin of a DH-parameterized robot
            %   This function maps the first moving frame of the robot to
            %   one that has a joint axis along Z. The function returns two
            %   matrices: worldToDHBase, which maps the world frame of the
            %   original robot to the new first moving DH frame, and
            %   tformF0toDh0, which maps the first moving frame of the
            %   original robot to the first moving DH frame.
            
            import robotics.manip.internal.AnalyticalIKHelpers
            import robotics.manip.internal.DHUtilities
            
            tformWToF0 = AnalyticalIKHelpers.getJointTransform(originalTree, 1, 0, tfTree);
            firstJointBodyIdx = AnalyticalIKHelpers.getBodyIndexFromJointIndex(originalTree, 1);
            tformF0ToDh0 = DHUtilities.getMappingFromCurrentFrameToZJointAxis(originalTree.Bodies{firstJointBodyIdx}.Joint.JointAxis, tol);
            worldToDHBase = tformWToF0*tformF0ToDh0;
            
        end
        
        function [commonNormalStartPt, commonNormalEndPt] = findCommonNormalForDH(frame1Origin, frame1ZAxis, frame2Origin, frame2ZAxis, tol)
            %findCommonNormalForDH Find the line that is the common normal between the Z-axes of two DH frames
            %   This method returns two points that define the start and
            %   end point of line that forms the common normal between two
            %   Z-axes. The start and end point are located along the
            %   Z-axis lines of the first and second frames, respectively.
            %   The method accepts four values: the origin of the first
            %   frame, the Z-axis of the first frame, the origin of the
            %   second frame, and the Z-axis of the second frame. It is
            %   important that these are all defined in the same reference
            %   frame; in this tool, they are always defined wrt the first
            %   frame. All inputs are 3-element vectors, and the returned
            %   vectors are also 3-element column vectors.
            
            import robotics.manip.internal.DHUtilities
            
            % Make sure all inputs are passed as column vectors
            frame1Origin = frame1Origin(:);
            frame1ZAxis = frame1ZAxis(:);
            frame2Origin = frame2Origin(:);
            frame2ZAxis = frame2ZAxis(:);
            
            originDiff = frame2Origin - frame1Origin;
            
            % Convert input vectors to unit vectors
            frame1ZUnitVector = frame1ZAxis(:)/norm(frame1ZAxis);
            frame2ZUnitVector = frame2ZAxis(:)/norm(frame2ZAxis);
            
            % This method takes the following approach:
            %    There are lines through the two Z-axes. A point on either
            %    line would be defined:
            %       pointOnFrame1ZAxis(C1) = frame1Origin + C1 * frame1ZAxisUnitVector;
            %       pointOnFrame2ZAxis(C2) = frame2Origin + C2 * frame2ZAxisUnitVector;
            %    where C1 and C2 are scalar constants.
            %
            %    For the common normal, which intersects both Z-axis lines,
            %    there are two points, commonNormalStartPt, on the
            %    intersection of the frame1Zaxis-line and the common
            %    normal, and commonNormalEndPt, on the intersection of the
            %    common normal and the frame2Zaxis-line. The vector between
            %    these two points must be orthogonal to both the  frame1Zaxis
            %    and frame2Axis vectors, i.e.:
            %       dot((commonNormalEndPt - commonNormalStartPt), frame1ZaxisUnitVector) = 0;
            %       dot((commonNormalEndPt - commonNormalStartPt), frame2ZAxisUnitVector) = 0;
            %
            %   Because the points are constrained to be along the Z-axes,
            %   we can further define them using the equation of a line
            %   noted earlier:
            %      commonNormalStartPt = frame1Origin + x1 * frame1ZAxis;
            %      commonNormalEndPt   = frame2Origin + x2 * frame2ZAxis;
            %   These equations can be combined by subtracting the second
            %   from the first
            %      (commonNormalEndPt - commonNormalStartPt) = (frame2Origin - frame1Origin) + (x2 * frame2ZAxisUnitVector - x1 * frame1ZAxisUnitVector);
            %
            %    Expanding the dot products:
            %      dot(commonNormalEndPt - commonNormalStartPt, frame1ZAxis) 
            %         = dot(frame2Origin - frame1Origin, frame1ZAxisUnitVector) + x2 * dot(frame2ZAxisUnitVector, frame1ZAxisUnitVector) - x1 * dot(frame1ZAxisUnitVector, frame1ZAxisUnitVector)
            %         = dot(frame2Origin - frame1Origin, frame1ZAxisUnitVector) + x2 * dot(frame2ZAxisUnitVector, frame1ZAxisUnitVector) - x1 * 1 = 0
            %      dot(commonNormalEndPt - commonNormalStartPt, frame2ZAxis)
            %         = dot(frame2Origin - frame1Origin, frame2ZAxisUnitVector) + x2 * dot(frame2ZAxisUnitVector, frame2ZAxisUnitVector) - x1 * dot(frame1ZAxisUnitVector, frame2ZAxisUnitVector) 
            %         = dot(frame2Origin - frame1Origin, frame2ZAxisUnitVector) + x2 * 1 - x1 * dot(frame1ZAxisUnitVector, frame2ZAxisUnitVector) = 0
            %
            %   Rearranging to eliminate the two commonNormal points,
            %   results in two equations that may instead be solved for x1
            %   and x2:
            %      x1 - x2 * dot(frame2ZAxisUnitVector, frame1ZAxisUnitVector) = dot(frame2Origin - frame1Origin, frame1ZAxisUnitVector);
            %      x1 * dot(frame1ZAxisUnitVector, frame2ZAxisUnitVector) - x2 = dot(frame2Origin - frame1Origin, frame2ZAxisUnitVector);
            %
            %   Then the aim is to solve these two equations for x1 and x2,
            %   which can be achieved using matrix formulation Ax = b. In
            %   the special case where the two lines are parallel, the
            %   common normal is selected to intersect with the second
            %   frame's origin, which ensures that the DH parameters will
            %   not include a dOffset (all translation along Z will be
            %   contained in the d-parameter along the frame 1 Z axis).
            
            zAxesUnitVectorDotProduct = dot(frame2ZUnitVector, frame1ZUnitVector);
            
            % Check for parallel line case
            if isEqualWithinTolerance(abs(zAxesUnitVectorDotProduct), 1, tol)
                x = DHUtilities.findCommonNormalBetweenParallelDHZVectors(originDiff, zAxesUnitVectorDotProduct, frame2ZUnitVector);
            else
                A = [1, -zAxesUnitVectorDotProduct; zAxesUnitVectorDotProduct, -1];
                b = [dot(originDiff, frame1ZUnitVector); dot(originDiff, frame2ZUnitVector)];
                x = A\b;
            end
            
            commonNormalStartPt = frame1Origin + x(1) * frame1ZUnitVector;
            commonNormalEndPt = frame2Origin + x(2) * frame2ZUnitVector;           
        end
        
        function x = findCommonNormalBetweenParallelDHZVectors(originDiff, zAxesUnitVectorDotProduct, frame2ZAxisUnitVector)
            % In the case of parallel lines, there is not a unique normal point for
            % either line. The method chooses the origin of line B to be the
            % first point, and solve for the corresponding point on line A.
            % This point is favorable because it means the second DH frame
            % will be colocated with the current second frame (i.e. a frame
            % at origin B), and there will be no need for a d offset. This
            % is equivalent to x2 = 0;
            %
            % The second equation for x2 = 0 reduces to:
            %
            % x1  = dot(frame2Origin - frame1Origin, frame2ZAxisUnitVector)/dot(frame1ZAxisUnitVector, frame2ZAxisUnitVector);
            
            x = zeros(2, 1);
            x(1) = dot(originDiff, frame2ZAxisUnitVector)/zAxesUnitVectorDotProduct;
            
        end
        
        function commonNormalUnitVector = determineDHCommonNormalUnitVector(commonNormalStartPoint, commonNormalEndPoint, nextFrameZAxisInCurrentFrame, tol)
            %determineDHCommonNormalUnitVector Compute the unit vector indicating the direction of the common normal for two DH frames
            %   This method accepts the start and end points of the common
            %   normal between two DH frames, provided as two 3-element
            %   vectors, as well as a three-element vector indicating the
            %   direction of the second frame's Z axis in the current frame
            %   coordinates, and a tolerance. The method returns the unit
            %   vector corresponding to the common normal between two
            %   DH-frames. 
            %
            %   When the common normal has a non-zero length, this is
            %   simply the normalized common normal vector. However, if the
            %   length is zero, the unit vector is determined from the
            %   cross product of the two DH frame Z-axes, or if they are
            %   parallel, any vector orthogonal to Z would work, so the
            %   X-axis is chosen because it's the simplest solution
            %   (doesn't result in added thetaOffset rotation).
            
            % Make sure all inputs are column vectors
            commonNormalStartPoint = commonNormalStartPoint(:);
            commonNormalEndPoint = commonNormalEndPoint(:);
            nextFrameZAxisInCurrentFrame = nextFrameZAxisInCurrentFrame(:);
            
            % Compute a common normal unit vector from the cross product,
            % because if the two origins are the same, the common normal
            % will technically be [0 0 0], even though it obviously has a
            % non-zero unit vector
            if isEqualWithinTolerance(commonNormalStartPoint, commonNormalEndPoint, tol)
                % If the two frames share the same origin, the common
                % normal will have length zero, but its direction is still predefined, and the unit vector must
                % instead be obtained from the cross product
                currentFrameZAxis = [0 0 1]';
                commonNormalVector = cross(currentFrameZAxis, nextFrameZAxisInCurrentFrame);
                if isEqualWithinTolerance(commonNormalVector(:)', [0 0 0], tol)
                    % If the Z-axes are parallel, any choice of common
                    % normal
                    % direction is acceptable, so the easiest choice for
                    % the common normal direction is the current x-axis
                    commonNormalVector = [1 0 0]';
                end
            else
                commonNormalVector = commonNormalEndPoint - commonNormalStartPoint;
            end
            
            % Return the unit vector
            commonNormalUnitVector = commonNormalVector(:)/norm(commonNormalVector);
            
        end
        
        function rotationAboutAxis = computeRotationBetweenTwoVectorsAboutAxis(vector1, vector2, expAxis, tol)
            %computeRotationBetweenTwoVectorsAboutAxis Compute the rotation between two vectors about a specified expected axis of rotation
            %   This method accepts two 3-element column vectors, vector1 and vector2,
            %   the expected axis of rotation expAxis (also a 3-element
            %   vector), and a tolerance. The method computes the angle of
            %   rotation between the two vectors and the axis of rotation,
            %   verifies that the expected axis of rotation is a valid axis
            %   of rotation (either it is parallel to a unique rotation
            %   axis, or it is a valid choice among an infinite possible
            %   set of rotation axes, e.g. when the two vectors are
            %   parallel). This method then outputs the rotation about that
            %   expected axis. If the expected axis is not a valid axis of
            %   rotation for the two vectors, the method returns NaN.
            
            % Make sure the vectors are passed as column vectors
            vector1 = vector1(:);
            vector2 = vector2(:);
            
            % Compute the rotation between the two angels
            [rotationAngle, rotationAxis] = robotics.manip.internal.DHUtilities.computeRotationBetweenTwoVectors(vector1, vector2, tol);
            
            % Project the rotation onto the expected axis
            if isnan(rotationAxis)
                % If no rotation axis could be determined, the rotation is
                % either zero or pi, and any axis that is perpendicular to
                % the vectors will work
                if isEqualWithinTolerance(dot(expAxis, vector1), 0, tol) && isEqualWithinTolerance(dot(expAxis, vector2), 0, tol)
                    rotationAboutAxis = rotationAngle;
                else
                    % If the expected axis is not perpendicular to both
                    % input vectors, it is not a valid axis of rotation and
                    % NaN is returned
                    rotationAboutAxis = NaN;
                end
                    
            else
                % In any other case, the rotation axis is unique. In that
                % case, this method will verify that the actual and
                % expected rotation axes are correct, and that the sign of
                % rotation is correct for the expected axis
                expUnitRotationAxis = expAxis(:)/norm(expAxis);
                actUnitRotationAxis = rotationAxis(:)/norm(rotationAxis);
                expAndActUnitRotationAxesDotProd = dot(expUnitRotationAxis, actUnitRotationAxis);
                if isEqualWithinTolerance(abs(expAndActUnitRotationAxesDotProd), 1, tol)
                    rotationSign = expAndActUnitRotationAxesDotProd;
                    rotationAboutAxis = rotationSign*rotationAngle;
                else
                    % If the expected and actual rotation vectors don't
                    % match, the expected axis of rotation is not valid and
                    % NaN is returned
                    rotationAboutAxis = NaN;
                end
            end
        end
        
        function [rotationAngle, rotationAxis] = computeRotationBetweenTwoVectors(vector1, vector2, tol)
            %computeRotationBetweenTwoVectors Compute the rotation and axis of rotation between two vectors
            %   Given two three-element column vectors, this method
            %   computes the angle of rotation between them and the axis of
            %   rotation (a common normal). The method will return NaN for
            %   the rotation axis in the two cases where it cannot be
            %   uniquely determined:
            %      - When either of the vectors is zero, the solution is
            %        trivial and any rotation/axis combo would work.
            %      - When the vectors are parallel, the solution is zero or
            %        pi (depending on their relative direction), and any
            %        vector that is orthogonal to them may be the rotation
            %        axis.
            
            % First check if either vector is zero, in which case the angle
            % is zero and a rotation axis cannot be determined
            if isEqualWithinTolerance(vector2(:), [0 0 0]', tol) || isEqualWithinTolerance(vector1(:), [0 0 0]', tol)
                rotationAngle = 0;
                rotationAxis = NaN;
                return;
            end
            
            % Get unit vectors from each vector and compute the cross
            % product
            unitvector1 = vector1(:)/norm(vector1);
            unitvector2 = vector2(:)/norm(vector2);
            crossProductVector = cross(unitvector1, unitvector2);
            
            % Get the magnitude of rotation from the magnitude of the cross
            % product
            crossMagnitude = norm(crossProductVector);
            
            % If the magnitude is zero, the vectors are either the same or
            % in opposite directions
            if isEqualWithinTolerance(crossMagnitude, 0, tol)
                if isEqualWithinTolerance(unitvector1, unitvector2, tol)
                    rotationAngle = 0;
                else
                    rotationAngle = pi;
                end
                
                % The choice of rotation axis is not unique, but rather any
                % vector that is orthogonal to either vector (since they
                % are in-line)
                rotationAxis = NaN;
                return;
            end
            
            % Get the angle from the direction
            crossProductUnitVector = crossProductVector(:)/crossMagnitude;
            sinRotationAngle = dot(crossProductUnitVector, crossProductVector);
            cosRotationAngle = dot(unitvector1, unitvector2);
            
            rotationAngle = atan2(sinRotationAngle, cosRotationAngle);
            rotationAxis = crossProductUnitVector;
        end
        
        function T = computeTransformFromDHAndZScrews(thPredefToDH, a, alpha, d, nextDOffset, thDHToNextPredef)
            %computeTransformFromDHAndZScrews Compute the transform that results from the implementation of ZScrews and DH
            %   This method computes the transform the results from the
            %   following sequence, and their anticipated function:
            %
            %      1. rotx(thPredefToDH)  - the rotation about Z that 
            %                               rotates a frame's X-axis to an
            %                               x-axis that is a common normal
            %                               with the next frame's Z-axis
            %
            %      2. transz(d)           - Part of the DH transform; the
            %                               translation along the first
            %                               frame's Z-axis 
            %
            %      3. transx(a)           - Part of the DH transform; the
            %                               translation along the first
            %                               frame's X-axis
            %
            %      4. rotx(alpha)         - Part of the DH transform; the
            %                               rotation about the common
            %                               normal
            %
            %      5. transz(nextDOffset) - The translation along the
            %                               second frame's Z axis that
            %                               moves the frame from its
            %                               necessary position for DH to
            %                               its actual position along the
            %                               Z-axis
            %
            %      6. rotz(nextThetaOffsetGuess)   - The rotation about the
            %                                        second frame's Z axis
            %                                        that relates the frame
            %                                        from its necessary
            %                                        position for DH to its
            %                                        actual position
            %
            %   Taken together, this sequence produces a transform that
            %   relates a first and second frame in space.
            
            thetaOffsetRotation = axang2tform([0 0 1 thPredefToDH]);
            dTranslation = trvec2tform([0 0 d]);
            aTranslation = trvec2tform([a 0 0]);
            alphaRotation = axang2tform([1 0 0 alpha]);
            dOffsetTranslation = trvec2tform([0 0 nextDOffset]);
            nextThetaOffsetRotation = axang2tform([0 0 1 thDHToNextPredef]);
            
            dhFrame = dTranslation*aTranslation*alphaRotation;
            T = ...
                thetaOffsetRotation * ... % Rotate the frame into position so that the x-axis is the common normal
                dhFrame * ...             % DH translation and rotation
                dOffsetTranslation * ...  % Shift from the distal DH frame to the match the position of a final frame
                nextThetaOffsetRotation;  % Rotate from the distal DH frame to match the orientation of a final frame (about Z)
            
        end
    end
end

function isEquiv = isEqualWithinTolerance(mat1, mat2, tol)
%isEqualWithinTolerance Check if two matrices are equal within a set tolerance
%   This function checks whether a pair of scalars, vectors, or matrices
%   contain equal elements within a tolerance. The function doesn't check
%   against dimension, since the primary aim is just to verify each element
%   is equal within the tolerance.

% If the two matrices are vectors, make sure the dimensions match to
% account for implicit expansion (otherwise v1 = v1' will return false)
if isvector(mat1)
    mat1 = mat1(:);
end

if isvector(mat2)
    mat2 = mat2(:);
end

diffMat = abs(mat1 - mat2);
isEquiv = all(all(diffMat < tol));

end
