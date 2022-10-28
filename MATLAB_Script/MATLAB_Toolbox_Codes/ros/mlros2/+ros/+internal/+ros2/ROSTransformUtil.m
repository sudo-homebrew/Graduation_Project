classdef (Hidden) ROSTransformUtil < ros.internal.TransformUtil
%This class is for internal use only. It may be removed in the future.

%ROSTRANSFORMUTIL Utility class for ROS transformations
%   See also ROSAPPLYTRANSFORM

% Copyright 2020 The MathWorks, Inc.

%#codegen    
    methods (Access={?ros.internal.TransformUtil})
        function obj = ROSTransformUtil(tfmsg)
            validateattributes(tfmsg,{'struct'},{'nonempty','scalar'});
            obj.TargetFrameName = tfmsg.header.frame_id;
            [obj.TfQuaternion, ~] = obj.tfGetQuaternion(tfmsg.transform.rotation);
            obj.TfRotationMatrix = quat2rotm(obj.TfQuaternion);
            tfTranslationVector = obj.tfGetPosition(tfmsg.transform.translation,false);
            obj.TfTransformationMatrix = trvec2tform(tfTranslationVector) * ...
                rotm2tform(obj.TfRotationMatrix);
        end
    end
    
    methods
        function tfQuatMsg = transformQuaternion(obj, quatMsg)
            %transformQuaternion Apply transformation to quaternion
            %   This function applies the transformation to the input quaternion
            %   message QUATMSG and returns the rotated quaternion TFQUATMSG.
            %   QUATMSG is required to be a message of type
            %   geometry_msgs/QuaternionStamped.
            
            % Create a transformed message struct based on the input message
            
            coder.inline('never');            
            tfQuatMsg = quatMsg;
            if isfield(quatMsg, 'quaternion')
                % Concatenate quaternions by multiplying them
                q = obj.TfQuaternion;
                [r, quatType] = obj.tfGetQuaternion(quatMsg.quaternion);
                outQuaternion = robotics.utils.internal.quatMultiply(q,r);
                % Write transformed quaternion to output message
                tfQuatMsg.quaternion = obj.tfSetQuaternion(outQuaternion, quatType);
            end
        end
        
        function tfVecMsg = transformVector3(obj, vecMsg)
            %transformVector3 Apply transformation to 3D vector
            %   This function applies the transformation to the input 3D vector
            %   message VECMSG and returns the transformed vector TFVECMSG. VECMSG
            %   is required to be a message of type geometry_msgs/Vector3Stamped.
            
            % Create a transformed message struct based on the input message

            coder.inline('never');
            
            tfVecMsg = vecMsg;
            if isfield(vecMsg, 'vector')
                % Apply transformation
                % Since this is a directional vector, only apply the rotational
                % component.
                inVec = vecMsg.vector;
                [outVec, posType] = obj.tfGetPosition(inVec, false);
                outVec = obj.TfRotationMatrix * outVec';
                
                % Write transformed vector to output message
                tfVecMsg.vector = obj.tfSetPosition(outVec, posType);
            end
        end
        
        function tfPointMsg = transformPoint(obj, pointMsg)
            %transformPoint Apply transformation to 3D point
            %   This function applies the transformation to the input 3D point
            %   message POINTMSG and return the transformed point TFPOINTMSG.
            %   POINTMSG is required to be message of type
            %   geometry_msgs/PointStamped.
            
            % Create a transformed message struct based on the input message
            coder.inline('never');
            
            tfPointMsg = pointMsg;
            if isfield(pointMsg, 'point')
                % Apply transformation
                inPoint = pointMsg.point;
                [homogPoint, posType] = obj.tfGetPosition(inPoint, true);
                homogPoint = obj.TfTransformationMatrix * homogPoint';
                
                % Write transformed point to output message
                % Homogeneous scale factor is guaranteed to be 1, so we can read
                % the first three components directly.
                tfPointMsg.point = obj.tfSetPosition(homogPoint, posType);
            end
        end
        
        function tfPoseMsg = transformPose(obj, poseMsg)
            %transformPose Apply transformation to pose
            %   This function applies the transformation to the input pose POSEMSG
            %   and returns the transformed pose TFPOSEMSG. POSMSG is required to
            %   be a message of type geometry_msgs/PoseStamped.
            
            % Create a transformed message struct based on the input message
            coder.inline('never');
            tfPoseMsg = poseMsg;
            if isfield(poseMsg, 'pose')
                % Create transformation matrix for pose
                [inQuaternion, quatType] = obj.tfGetQuaternion(poseMsg.pose.orientation);
                [inPosition, posType] = obj.tfGetPosition(poseMsg.pose.position, false);
                poseTform = trvec2tform(inPosition) * quat2tform(inQuaternion);
                % Apply the transformation to the pose
                tfPose = obj.TfTransformationMatrix * poseTform;
                % Populate the output message
                outQuaternion = tform2quat(tfPose);
                tfPoseMsg.pose.orientation = obj.tfSetQuaternion(outQuaternion, quatType);
                outPosition = tform2trvec(tfPose);
                tfPoseMsg.pose.position = obj.tfSetPosition(outPosition, posType);
            end

        end
        
        function tfPcloudMsg = transformPointCloud2(obj, pcloudMsg)
            %transformPointCloud2 Apply transformation to point cloud
            %   This function applies the transformation to every point in the
            %   input point cloud PCLOUDMSG and returns the transformed point cloud
            %   TFPCLOUDMSG. PCLOUDMSG is required to be a message of type
            %   sensor_msgs/PointCloud2.
            
            % Create a transformed message struct based on the input message
            coder.inline('never');
            tfPcloudMsg = pcloudMsg;
            if isfield(pcloudMsg, 'point_step')                
                % Get XYZ data. An error will be displayed if the 'x', 'y;, and 'z'
                % fields do not exist.
                xyz = rosReadXYZ(pcloudMsg);
                
                % Apply the transformation to the point cloud
                % It is more efficient to do post-multiply here
                homoxyz = cart2hom(xyz);
                tfxyz = homoxyz * obj.TfTransformationMatrix.';
                
                % Now, write the transformed data back to the message. Since the
                % (4,4) element of the tfTransformationMatrix is always 1.0, we do
                % not need to explicitly convert back to Cartesian coordinates. The
                % 4th element for all points is 1.0 already.
                tfPcloudMsg = obj.MsgUtilObj.writeXYZ(tfPcloudMsg, tfxyz(:,1:3));
            end
        end
        function msg = setHeaderFrameId(obj, msg)
            msg.header.frame_id = obj.TargetFrameName;
        end        
    end
    
    methods (Static, Access=protected)
        function [quat, quatType] = tfGetQuaternion(quatMsg)
            %tfGetQuaternion Extract and normalize the quaternion from a ROS message
            %   The input message QUATMSG need to be of message type
            %   geometry_msgs/Quaternion
            coder.inline('never');
            quatType = quatMsg.MessageType;
            quat = [0 0 0 0];
            if isfield(quatMsg, 'w')
                quat = [quatMsg.w quatMsg.x quatMsg.y quatMsg.z];
                
                if norm(quat) < 1e-7
                    % Handle the singularity of a zero-length quaternion
                    quat = [1 0 0 0];
                else
                    % Normalize the quaternion
                    quat = robotics.internal.normalizeRows(quat);
                end
            end

        end
        
        function quatMsg = tfSetQuaternion(quat, quatType)
            %tfSetQuaternion Set quaternion message struct from quaternion vector
            %   The output message struct QUATMSG needs to be of message type
            %   geometry_msgs/Quaternion.
            %   The input QUAT is a vector with length 4 and is assumed to be of
            %   unit length.
            coder.inline('always');
            quatMsg.MessageType = quatType;
            quatMsg.x = quat(2);
            quatMsg.y = quat(3);
            quatMsg.z = quat(4);
            quatMsg.w = quat(1);
        end
        
        function [posvec, posType] = tfGetPosition(posMsg, homog)
            %tfGetPosition Extract a position vector from an input ROS message
            %   The input message needs to have the X, Y, and Z fields. For example
            %   geometry_msgs/Vector3 would work.
            coder.inline('always');
            if homog
                posvec = [posMsg.x posMsg.y posMsg.z 1];
            else
                posvec = [posMsg.x posMsg.y posMsg.z];
            end
            posType = posMsg.MessageType;
        end
        
        function posMsg = tfSetPosition(posvec, posType)
            %tfSetPosition Set position message struct from MATLAB vector
            %   The output message struct needs to have the x, y, and z fields.
            %   The input POSVEC is a MATLAB vector with length 3.
            coder.inline('always');
            posMsg.MessageType = posType;
            posMsg.x = posvec(1);
            posMsg.y = posvec(2);
            posMsg.z = posvec(3);
            
        end
    end
end
