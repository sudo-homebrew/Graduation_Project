classdef (Abstract) TransformUtil < handle
    %This class is for internal use only. It may be removed in the future.
    
    %TRANSFORMUTIL Utility class for ROS transformations
    %   See also ROSAPPLYTRANSFORM
    
    % Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    properties (Access=protected)
        TargetFrameName
        TfQuaternion
        TfRotationMatrix
        TfTransformationMatrix
        MsgUtilObj
    end
    
    methods (Static)
        function ret = getInstance(tfmsg)
            validateattributes(tfmsg,{'struct'},{'nonempty','scalar'});
            if isfield(tfmsg,'Transform')
                ret = ros.internal.ROSTransformUtil(tfmsg);
                ret.MsgUtilObj = ros.internal.SpecialMsgUtil;
            else
                ret = ros.internal.ros2.ROSTransformUtil(tfmsg);
                ret.MsgUtilObj = ros.internal.ros2.SpecialMsgUtil;
            end
        end
    end
    
    methods (Abstract)
        tfQuatMsg = transformQuaternion(obj, quatMsg)
        tfVecMsg = transformVector3(obj, vecMsg)
        tfPointMsg = transformPoint(obj, pointMsg)
        tfPoseMsg = transformPose(obj, poseMsg)
        tfPcloudMsg = transformPointCloud2(obj, pcloudMsg)
        msg = setHeaderFrameId(msg)        
    end
    
    methods (Abstract, Static, Access=protected)
        [quat, quatType] = tfGetQuaternion(quatMsg)
        quatMsg = tfSetQuaternion(quat, quatType)
        [posvec, posType] = tfGetPosition(posMsg, homog)
        posMsg = tfSetPosition(posvec, posType)
    end
end
