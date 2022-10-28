classdef TransformHelper
    %This class is for internal use only. It may be removed in the future.
    
    %TransformHelper Helper class for applying transformations.
    %   This class extracts the transformation defined by a given 
    %   geometry_msgs/TransformStamped message and provides functionality to
    %   apply it to entities like quaternions, points, and poses.
    %
    %   TFHELPER = ros.internal.TransformHelper(TFORMMSG) creates a transformation
    %   helper object TH. TFORMMSG is a ROS message of type geometry_msgs/TransformStamped 
    %   representing the transformation.
    %
    %   This class uses the robotics utility functions for transformations.
    %
    %   Currently, the timestamps of the input entities are ignored.
    %
    %   Example:
    %      % Create a helper object with a transformation
    %      % tformMsg is assumed to be a ROS message with type geometry_msgs/TransformStamped
    %      tfHelper = ros.internal.TransformHelper(tformMsg);
    %
    %      % Create an input point message
    %      pt = rosmessage('geometry_msgs/PointStamped');
    %      pt.Point.X = 5;
    %      pt.Point.Z = 2.4;
    %
    %      % Transform the point
    %      ptTransformed = transform(tfHelper, pt)
       
    %   Copyright 2014-2020 The MathWorks, Inc.
    
    properties (SetAccess = private)
        %TransformationMatrix - The transformation as a 4x4 homogeneous matrix
        %   This property will be updated automatically when a value is
        %   assigned to the TransformStamped property.
        TransformationMatrix
        
        %RotationMatrix - The rotational component of the transformation as matrix
        %   The rotation matrix is of size 3x3.
        %   This property will be updated automatically when a value is
        %   assigned to the TransformStamped property.
        RotationMatrix

        %TranslationVector - The translational component of the transformation
        %   The translation vector is of size 1x3.
        %   This property will be updated automatically when a value is
        %   assigned to the TransformStamped property.
        TranslationVector
        
        %Quaternion - The rotational component of the transformation as quaternion
        %   The quaternion is of size 1x4.
        %   This property will be updated automatically when a value is
        %   assigned to the TransformStamped property.
        Quaternion
    end
    
    properties (Access = private)
        %SourceFrameName - The name of the source frame
        SourceFrameName = ''
        
        %TargetFrameName - The name of the target frame
        TargetFrameName = ''
    end
    
    methods
        function obj = TransformHelper(tformMsg)
            %TransformHelper Construct the object
            
            validateattributes(tformMsg, ...
                               {'ros.msggen.geometry_msgs.TransformStamped', 'struct'}, ...
                               {'scalar'}, 'TransformHelper', 'tformMsg');
            
            % Store frame information
            obj.SourceFrameName = tformMsg.ChildFrameId;
            obj.TargetFrameName = tformMsg.Header.FrameId;
            
            % Extract quaternion rotation and rotation matrix
            obj.Quaternion = obj.getQuaternion(tformMsg.Transform.Rotation);
            obj.RotationMatrix = quat2rotm(obj.Quaternion);

            % Extract translation and construct full transformation matrix
            obj.TranslationVector = obj.getPosition(tformMsg.Transform.Translation, false);
            obj.TransformationMatrix = trvec2tform(obj.TranslationVector) * ...
                rotm2tform(obj.RotationMatrix);
        end
        
        function tfEntity = transform(obj, entity)
            %TRANSFORM Convenience function for transforming message entities
            %   TFENTITY = TRANSFORM(OBJ, ENTITY) applies the
            %   transformation to the input ROS message ENTITY and returns
            %   the transformed entity TFENTITY.
            %
            %   This function will determine the type of the input message
            %   and call the appropriate transformation method. If a
            %   particular message type cannot be handled by this object,
            %   an error will be displayed.
            %
            %   Supported message types include:
            %    - geometry_msgs/QuaternionStamped
            %    - geometry_msgs/Vector3Stamped
            %    - geometry_msgs/PointStamped
            %    - geometry_msgs/PoseStamped
            %    - sensor_msgs/PointCloud2
            %
            %   See also transformQuaternion, transformVector3,
            %   transformPoint, transformPose, transformPointCloud2.

            % Make sure the input is a scalar message
            validateattributes(entity, ...
                               {'ros.Message', 'struct'}, ...
                               {'scalar'}, 'transform', 'entity');
            
            % Based on the message type, call the appropriate
            % transformation function            
            switch entity.MessageType
                case {'geometry_msgs/QuaternionStamped'}
                    tfEntity = obj.transformQuaternion(entity);
                case {'geometry_msgs/Vector3Stamped'}
                    tfEntity = obj.transformVector3(entity);
                case {'geometry_msgs/PointStamped'}
                    tfEntity = obj.transformPoint(entity);
                case {'geometry_msgs/PoseStamped'}
                    tfEntity = obj.transformPose(entity);
                case 'sensor_msgs/PointCloud2'
                    tfEntity = obj.transformPointCloud2(entity);
                otherwise
                    error(message('ros:mlros:tf:MessageTypeNotSupported', ...
                        entity.MessageType, ['QuaternionStamped, Vector3Stamped, PointStamped, ', ...
                        'PoseStamped, PointCloud2']));
            end
        end
        
        function tfQuatMsg = transformQuaternion(obj, quatMsg)
            %transformQuaternion Apply transformation to quaternion
            %
            %   TFQUAT = transformQuaternion(OBJ, QUAT) applies the
            %   transformation to the input quaternion message QUAT and returns
            %   the rotated quaternion TFQUAT. QUAT is required to be a
            %   message of type geometry_msgs/QuaternionStamped.
            %
            %   See also transform.
            
            validateattributes(quatMsg, ...
                               {'ros.msggen.geometry_msgs.QuaternionStamped', 'struct'}, ...
                               {'scalar'}, 'transformQuaternion', 'quat');
            
            tfQuatMsg = obj.createTransformedEntity(quatMsg);
                        
            % Concatenate quaternions by multiplying them
            q = obj.Quaternion;
            r = obj.getQuaternion(quatMsg.Quaternion);
            outQuaternion = robotics.utils.internal.quatMultiply(q,r);
            
            % Write transformed quaternion to output message
            tfQuatMsg.Quaternion = obj.setQuaternion(tfQuatMsg.Quaternion, outQuaternion);
        end
        
        function tfVecMsg = transformVector3(obj, vecMsg)
            %transformVector3 Apply transformation to 3D vector
            %
            %   TFVECMSG = transformVector3(OBJ, VECMSG) applies the
            %   transformation to the input 3D vector message VECMSG and returns
            %   the transformed vector TFVECMSG. VECMSG is required to be a
            %   message of type geometry_msgs/Vector3Stamped.
            %
            %   See also transform.
            
            validateattributes(vecMsg, ...
                               {'ros.msggen.geometry_msgs.Vector3Stamped', 'struct'}, ...
                               {'scalar'}, 'transformVector3', 'vecMsg');
                
            tfVecMsg = obj.createTransformedEntity(vecMsg);
            
            % Apply transformation
            % Since this is a directional vector, only apply the rotational
            % component.
            inVec = vecMsg.Vector;
            outVec = obj.RotationMatrix * obj.getPosition(inVec, false)';
                        
            % Write transformed vector to output message
            tfVecMsg.Vector = obj.setPosition(tfVecMsg.Vector, outVec);
        end
        
        function tfPointMsg = transformPoint(obj, pointMsg)
            %transformPoint Apply transformation to 3D point
            %
            %   TFPOINTMSG = transformPoint(OBJ, POINTMSG) applies the
            %   transformation to the input 3D point message POINTMSG and returns
            %   the transformed point TFPOINTMSG. POINTMSG is required to be a
            %   message of type geometry_msgs/PointStamped.
            %
            %   See also transform.

            validateattributes(pointMsg, ...
                               {'ros.msggen.geometry_msgs.PointStamped', 'struct'}, ...
                               {'scalar'}, 'transformPoint', 'pointMsg');    
            
            tfPointMsg = obj.createTransformedEntity(pointMsg);
            
            % Apply transformation
            inPoint = pointMsg.Point;
            homPoint = obj.TransformationMatrix * obj.getPosition(inPoint, true)';
                        
            % Write transformed point to output message
            % Homogeneous scale factor is guaranteed to be 1, so we can read
            % the first three components directly.            
            tfPointMsg.Point = obj.setPosition(tfPointMsg.Point, homPoint);
        end
        
        function tfPoseMsg = transformPose(obj, poseMsg)
            %transformPose Apply transformation to pose
            %
            %   TFPOSEMSG = transformPoint(OBJ, POSEMSG) applies the
            %   transformation to the input pose POSEMSG and returns
            %   the transformed pose TFPOSEMSG. POSEMSG is required to be a
            %   message of type geometry_msgs/PoseStamped.
            %
            %   See also transform.
            
            validateattributes(poseMsg, ...
                               {'ros.msggen.geometry_msgs.PoseStamped', 'struct'}, ...
                               {'scalar'}, 'transformPose', 'poseMsg');       
            
            tfPoseMsg = obj.createTransformedEntity(poseMsg);
                        
            % Create transformation matrix for pose
            inQuaternion = obj.getQuaternion(poseMsg.Pose.Orientation);
            inPosition = obj.getPosition(poseMsg.Pose.Position, false);            
            poseTform = trvec2tform(inPosition) * quat2tform(inQuaternion);
            
            % Apply the transformation to the pose
            tfPose = obj.TransformationMatrix * poseTform;
            
            % Populate the output message
            outQuaternion = tform2quat(tfPose);
            tfPoseMsg.Pose.Orientation = obj.setQuaternion(tfPoseMsg.Pose.Orientation, outQuaternion);

            outPosition = tform2trvec(tfPose);
            tfPoseMsg.Pose.Position = obj.setPosition(tfPoseMsg.Pose.Position, outPosition);
        end
        
        function tfPtCloudMsg = transformPointCloud2(obj, ptCloudMsg)
            %transformPointCloud2 Apply transformation to point cloud
            %
            %   TFPTCLOUDMSG = transformPointCloud2(OBJ, PTCLOUDMSG) applies the
            %   transformation to every point in input point cloud PTCLOUDMSG 
            %   and returns the transformed point cloud TFPTCLOUDMSG.
            %   PTCLOUDMSG is required to be a message of type
            %   sensor_msgs/PointCloud2.
            %
            %   See also transform.
            
            validateattributes(ptCloudMsg, ...
                               {'ros.msg.sensor_msgs.PointCloud2', 'struct'}, ...
                               {'scalar'}, 'transformPointCloud2', 'ptCloudMsg');      
            
            if isa(ptCloudMsg, 'ros.Message')
                % Need to keep all meta data, so make a more thorough copy
                tfPtCloudMsg = ptCloudMsg.copy;
                tfPtCloudMsg.Header.FrameId = obj.TargetFrameName;

                % Get XYZ data. An error will be displayed if the 'x', 'y', and 'z' 
                % fields do not exist. Make sure to read unstructured data.
                if ptCloudMsg.PreserveStructureOnRead                
                    ptCloudMsg.PreserveStructureOnRead = false;
                    restoreProperty = onCleanup(@() restorePreserveStructureOnRead(ptCloudMsg));
                end
                xyz = ptCloudMsg.readXYZ;

                % Apply the transformation to the point cloud
                % It is more efficient to do post-multiply here
                homxyz = cart2hom(xyz);
                tfxyz = homxyz * obj.TransformationMatrix.';

                % Now, write the transformed data back to the message. Since
                % the (4,4) element of the TransformationMatrix is always 1.0,
                % we do not need to explicitly convert back to Cartesian
                % coordinates. The 4th element for all points is 1.0 already.
                tfPtCloudMsg.writeXYZ(tfxyz(:,1:3));
            else
                % No need to copy message for structure, apply frame at end
                xyz = rosReadXYZ(ptCloudMsg, 'PreserveStructureOnRead', false);
                homxyz = cart2hom(xyz);
                tfxyz = homxyz * obj.TransformationMatrix.';
                tfPtCloudMsg = ros.internal.SpecialMsgUtil.writeXYZ(ptCloudMsg, tfxyz(:,1:3));
                tfPtCloudMsg.Header.FrameId = obj.TargetFrameName;
            end
            
            function restorePreserveStructureOnRead(ptCloudMsg)
                ptCloudMsg.PreserveStructureOnRead = true;
            end
        end  
    end
    
    methods (Access = private)
        function tfEntity = createTransformedEntity(obj, entity)
            %createTransformedEntity Create an empty transformed entity
            %   This will also copy some data like the time stamp and
            %   FrameId from the original entity.
            
            if isa(entity, 'ros.Message')
                tfEntity = rosmessage(entity.MessageType, 'DataFormat', 'object');
            else
                tfEntity = rosmessage(entity.MessageType, 'DataFormat', 'struct');
            end

            % Copy header information from stamped message
            tfEntity.Header.Stamp = entity.Header.Stamp;
            tfEntity.Header.FrameId = obj.TargetFrameName;
        end        
    end
       
    methods (Static, Access = ?matlab.unittest.TestCase)
        function quat = getQuaternion(quatMsg)
            %getQuaternion Extract and normalize the quaternion from a ROS message
            %   The input message QUATMSG needs to be of message type
            %   geometry_msgs/Quaternion
            
            quat = [quatMsg.W quatMsg.X quatMsg.Y quatMsg.Z];
                        
            if norm(quat) < 1e-7
                % Handle the singularity of a zero-length quaternion
                quat = [1 0 0 0];
            else
                % Normalize the quaternion
                quat = robotics.internal.normalizeRows(quat);
            end
        end
        
        function quatMsg = setQuaternion(quatMsg, quat)
            %setQuaternion Set quaternion ROS message from quaternion vector
            %   The output message QUATMSG needs to be of message type
            %   geometry_msgs/Quaternion.
            %   The input QUAT is a vector with length 4 and is assumed to
            %   be of unit length.

            quatMsg.W = quat(1);
            quatMsg.X = quat(2);
            quatMsg.Y = quat(3);
            quatMsg.Z = quat(4);                        
        end        
        
        function posvec = getPosition(posMsg, homog)
            %getPosition Extract a position vector from an input ROS message
            %   The input message needs to have the X, Y, and Z fields. For
            %   example geometry_msgs/Vector3 would work.
            
            if homog
                posvec = [posMsg.X, posMsg.Y, posMsg.Z, 1];
            else
                posvec = [posMsg.X, posMsg.Y, posMsg.Z];
            end            
        end
        
        function posMsg = setPosition(posMsg, posvec)
            %setPosition Set ROS message from MATLAB vector
            %   The output message needs to have the X, Y, and Z fields.
            %   The input POSVEC is a MATLAB vector with length 3.

            posMsg.X = posvec(1);
            posMsg.Y = posvec(2);
            posMsg.Z = posvec(3);
        end           
    end   
end
