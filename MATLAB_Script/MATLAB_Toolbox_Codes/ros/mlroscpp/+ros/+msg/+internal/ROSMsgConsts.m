classdef ROSMsgConsts
    %ROSMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2020 The MathWorks, Inc.
    
    properties (Constant)
        actionlib_TestAction = 'actionlib/TestAction'
        actionlib_TestActionFeedback = 'actionlib/TestActionFeedback'
        actionlib_TestActionGoal = 'actionlib/TestActionGoal'
        actionlib_TestActionResult = 'actionlib/TestActionResult'
        actionlib_TestFeedback = 'actionlib/TestFeedback'
        actionlib_TestGoal = 'actionlib/TestGoal'
        actionlib_TestRequestActionGoal = 'actionlib/TestRequestActionGoal'
        actionlib_TestRequestActionResult = 'actionlib/TestRequestActionResult'
        actionlib_TestRequestFeedback = 'actionlib/TestRequestFeedback'
        actionlib_TestRequestGoal = 'actionlib/TestRequestGoal'
        actionlib_TestRequestResult = 'actionlib/TestRequestResult'
        actionlib_TestResult = 'actionlib/TestResult'
        actionlib_TwoIntsActionGoal = 'actionlib/TwoIntsActionGoal'
        actionlib_TwoIntsActionResult = 'actionlib/TwoIntsActionResult'
        actionlib_TwoIntsFeedback = 'actionlib/TwoIntsFeedback'
        actionlib_TwoIntsGoal = 'actionlib/TwoIntsGoal'
        actionlib_TwoIntsResult = 'actionlib/TwoIntsResult'
        actionlib_msgs_GoalID = 'actionlib_msgs/GoalID'
        actionlib_msgs_GoalStatus = 'actionlib_msgs/GoalStatus'
        actionlib_msgs_GoalStatusArray = 'actionlib_msgs/GoalStatusArray'
        control_msgs_FollowJointTrajectoryAction = 'control_msgs/FollowJointTrajectoryAction'
        control_msgs_FollowJointTrajectoryActionFeedback = 'control_msgs/FollowJointTrajectoryActionFeedback'
        control_msgs_FollowJointTrajectoryActionGoal = 'control_msgs/FollowJointTrajectoryActionGoal'
        control_msgs_FollowJointTrajectoryActionResult = 'control_msgs/FollowJointTrajectoryActionResult'
        control_msgs_FollowJointTrajectoryFeedback = 'control_msgs/FollowJointTrajectoryFeedback'
        control_msgs_FollowJointTrajectoryGoal = 'control_msgs/FollowJointTrajectoryGoal'
        control_msgs_FollowJointTrajectoryResult = 'control_msgs/FollowJointTrajectoryResult'
        control_msgs_GripperCommand = 'control_msgs/GripperCommand'
        control_msgs_GripperCommandAction = 'control_msgs/GripperCommandAction'
        control_msgs_GripperCommandActionFeedback = 'control_msgs/GripperCommandActionFeedback'
        control_msgs_GripperCommandActionGoal = 'control_msgs/GripperCommandActionGoal'
        control_msgs_GripperCommandActionResult = 'control_msgs/GripperCommandActionResult'
        control_msgs_GripperCommandFeedback = 'control_msgs/GripperCommandFeedback'
        control_msgs_GripperCommandGoal = 'control_msgs/GripperCommandGoal'
        control_msgs_GripperCommandResult = 'control_msgs/GripperCommandResult'
        control_msgs_JointControllerState = 'control_msgs/JointControllerState'
        control_msgs_JointJog = 'control_msgs/JointJog'
        control_msgs_JointTolerance = 'control_msgs/JointTolerance'
        control_msgs_JointTrajectoryActionGoal = 'control_msgs/JointTrajectoryActionGoal'
        control_msgs_JointTrajectoryControllerState = 'control_msgs/JointTrajectoryControllerState'
        control_msgs_JointTrajectoryFeedback = 'control_msgs/JointTrajectoryFeedback'
        control_msgs_JointTrajectoryGoal = 'control_msgs/JointTrajectoryGoal'
        control_msgs_PidState = 'control_msgs/PidState'
        control_msgs_PointHeadActionFeedback = 'control_msgs/PointHeadActionFeedback'
        control_msgs_PointHeadActionGoal = 'control_msgs/PointHeadActionGoal'
        control_msgs_PointHeadFeedback = 'control_msgs/PointHeadFeedback'
        control_msgs_PointHeadGoal = 'control_msgs/PointHeadGoal'
        control_msgs_PointHeadResult = 'control_msgs/PointHeadResult'
        control_msgs_QueryCalibrationStateRequest = 'control_msgs/QueryCalibrationStateRequest'
        control_msgs_QueryCalibrationStateResponse = 'control_msgs/QueryCalibrationStateResponse'
        control_msgs_QueryTrajectoryStateRequest = 'control_msgs/QueryTrajectoryStateRequest'
        control_msgs_QueryTrajectoryStateResponse = 'control_msgs/QueryTrajectoryStateResponse'
        control_msgs_SingleJointPositionActionFeedback = 'control_msgs/SingleJointPositionActionFeedback'
        control_msgs_SingleJointPositionActionGoal = 'control_msgs/SingleJointPositionActionGoal'
        control_msgs_SingleJointPositionFeedback = 'control_msgs/SingleJointPositionFeedback'
        control_msgs_SingleJointPositionGoal = 'control_msgs/SingleJointPositionGoal'
        control_msgs_SingleJointPositionResult = 'control_msgs/SingleJointPositionResult'
        diagnostic_msgs_AddDiagnosticsRequest = 'diagnostic_msgs/AddDiagnosticsRequest'
        diagnostic_msgs_AddDiagnosticsResponse = 'diagnostic_msgs/AddDiagnosticsResponse'
        diagnostic_msgs_DiagnosticArray = 'diagnostic_msgs/DiagnosticArray'
        diagnostic_msgs_DiagnosticStatus = 'diagnostic_msgs/DiagnosticStatus'
        diagnostic_msgs_KeyValue = 'diagnostic_msgs/KeyValue'
        diagnostic_msgs_SelfTestRequest = 'diagnostic_msgs/SelfTestRequest'
        diagnostic_msgs_SelfTestResponse = 'diagnostic_msgs/SelfTestResponse'
        dynamic_reconfigure_BoolParameter = 'dynamic_reconfigure/BoolParameter'
        dynamic_reconfigure_Config = 'dynamic_reconfigure/Config'
        dynamic_reconfigure_ConfigDescription = 'dynamic_reconfigure/ConfigDescription'
        dynamic_reconfigure_DoubleParameter = 'dynamic_reconfigure/DoubleParameter'
        dynamic_reconfigure_Group = 'dynamic_reconfigure/Group'
        dynamic_reconfigure_GroupState = 'dynamic_reconfigure/GroupState'
        dynamic_reconfigure_IntParameter = 'dynamic_reconfigure/IntParameter'
        dynamic_reconfigure_ParamDescription = 'dynamic_reconfigure/ParamDescription'
        dynamic_reconfigure_SensorLevels = 'dynamic_reconfigure/SensorLevels'
        dynamic_reconfigure_StrParameter = 'dynamic_reconfigure/StrParameter'
        geometry_msgs_Accel = 'geometry_msgs/Accel'
        geometry_msgs_AccelStamped = 'geometry_msgs/AccelStamped'
        geometry_msgs_AccelWithCovariance = 'geometry_msgs/AccelWithCovariance'
        geometry_msgs_AccelWithCovarianceStamped = 'geometry_msgs/AccelWithCovarianceStamped'
        geometry_msgs_Inertia = 'geometry_msgs/Inertia'
        geometry_msgs_InertiaStamped = 'geometry_msgs/InertiaStamped'
        geometry_msgs_Point = 'geometry_msgs/Point'
        geometry_msgs_Point32 = 'geometry_msgs/Point32'
        geometry_msgs_PointStamped = 'geometry_msgs/PointStamped'
        geometry_msgs_Polygon = 'geometry_msgs/Polygon'
        geometry_msgs_PolygonStamped = 'geometry_msgs/PolygonStamped'
        geometry_msgs_Pose = 'geometry_msgs/Pose'
        geometry_msgs_Pose2D = 'geometry_msgs/Pose2D'
        geometry_msgs_PoseArray = 'geometry_msgs/PoseArray'
        geometry_msgs_PoseStamped = 'geometry_msgs/PoseStamped'
        geometry_msgs_PoseWithCovariance = 'geometry_msgs/PoseWithCovariance'
        geometry_msgs_PoseWithCovarianceStamped = 'geometry_msgs/PoseWithCovarianceStamped'
        geometry_msgs_Quaternion = 'geometry_msgs/Quaternion'
        geometry_msgs_QuaternionStamped = 'geometry_msgs/QuaternionStamped'
        geometry_msgs_Transform = 'geometry_msgs/Transform'
        geometry_msgs_TransformStamped = 'geometry_msgs/TransformStamped'
        geometry_msgs_Twist = 'geometry_msgs/Twist'
        geometry_msgs_TwistStamped = 'geometry_msgs/TwistStamped'
        geometry_msgs_TwistWithCovariance = 'geometry_msgs/TwistWithCovariance'
        geometry_msgs_TwistWithCovarianceStamped = 'geometry_msgs/TwistWithCovarianceStamped'
        geometry_msgs_Vector3 = 'geometry_msgs/Vector3'
        geometry_msgs_Vector3Stamped = 'geometry_msgs/Vector3Stamped'
        geometry_msgs_Wrench = 'geometry_msgs/Wrench'
        geometry_msgs_WrenchStamped = 'geometry_msgs/WrenchStamped'
        map_msgs_GetMapROIRequest = 'map_msgs/GetMapROIRequest'
        map_msgs_GetMapROIResponse = 'map_msgs/GetMapROIResponse'
        map_msgs_GetPointMapROIRequest = 'map_msgs/GetPointMapROIRequest'
        map_msgs_GetPointMapROIResponse = 'map_msgs/GetPointMapROIResponse'
        map_msgs_GetPointMapRequest = 'map_msgs/GetPointMapRequest'
        map_msgs_GetPointMapResponse = 'map_msgs/GetPointMapResponse'
        map_msgs_OccupancyGridUpdate = 'map_msgs/OccupancyGridUpdate'
        map_msgs_PointCloud2Update = 'map_msgs/PointCloud2Update'
        map_msgs_ProjectedMap = 'map_msgs/ProjectedMap'
        map_msgs_ProjectedMapInfo = 'map_msgs/ProjectedMapInfo'
        map_msgs_ProjectedMapsInfoRequest = 'map_msgs/ProjectedMapsInfoRequest'
        map_msgs_ProjectedMapsInfoResponse = 'map_msgs/ProjectedMapsInfoResponse'
        map_msgs_SaveMapRequest = 'map_msgs/SaveMapRequest'
        map_msgs_SaveMapResponse = 'map_msgs/SaveMapResponse'
        map_msgs_SetMapProjectionsRequest = 'map_msgs/SetMapProjectionsRequest'
        map_msgs_SetMapProjectionsResponse = 'map_msgs/SetMapProjectionsResponse'
        nav_msgs_GetMapActionResult = 'nav_msgs/GetMapActionResult'
        nav_msgs_GetMapFeedback = 'nav_msgs/GetMapFeedback'
        nav_msgs_GetMapGoal = 'nav_msgs/GetMapGoal'
        nav_msgs_GetMapRequest = 'nav_msgs/GetMapRequest'
        nav_msgs_GetMapResponse = 'nav_msgs/GetMapResponse'
        nav_msgs_GetMapResult = 'nav_msgs/GetMapResult'
        nav_msgs_GetPlanRequest = 'nav_msgs/GetPlanRequest'
        nav_msgs_GetPlanResponse = 'nav_msgs/GetPlanResponse'
        nav_msgs_GridCells = 'nav_msgs/GridCells'
        nav_msgs_MapMetaData = 'nav_msgs/MapMetaData'
        nav_msgs_OccupancyGrid = 'nav_msgs/OccupancyGrid'
        nav_msgs_Odometry = 'nav_msgs/Odometry'
        nav_msgs_Path = 'nav_msgs/Path'
        nav_msgs_SetMapRequest = 'nav_msgs/SetMapRequest'
        nav_msgs_SetMapResponse = 'nav_msgs/SetMapResponse'
        roscpp_EmptyRequest = 'roscpp/EmptyRequest'
        roscpp_EmptyResponse = 'roscpp/EmptyResponse'
        roscpp_GetLoggersRequest = 'roscpp/GetLoggersRequest'
        roscpp_GetLoggersResponse = 'roscpp/GetLoggersResponse'
        roscpp_Logger = 'roscpp/Logger'
        roscpp_SetLoggerLevelRequest = 'roscpp/SetLoggerLevelRequest'
        roscpp_SetLoggerLevelResponse = 'roscpp/SetLoggerLevelResponse'
        roscpp_tutorials_TwoIntsRequest = 'roscpp_tutorials/TwoIntsRequest'
        roscpp_tutorials_TwoIntsResponse = 'roscpp_tutorials/TwoIntsResponse'
        rosgraph_msgs_Clock = 'rosgraph_msgs/Clock'
        rosgraph_msgs_Log = 'rosgraph_msgs/Log'
        rosgraph_msgs_TopicStatistics = 'rosgraph_msgs/TopicStatistics'
        sensor_msgs_BatteryState = 'sensor_msgs/BatteryState'
        sensor_msgs_CameraInfo = 'sensor_msgs/CameraInfo'
        sensor_msgs_ChannelFloat32 = 'sensor_msgs/ChannelFloat32'
        sensor_msgs_CompressedImage = 'sensor_msgs/CompressedImage'
        sensor_msgs_FluidPressure = 'sensor_msgs/FluidPressure'
        sensor_msgs_Illuminance = 'sensor_msgs/Illuminance'
        sensor_msgs_Image = 'sensor_msgs/Image'
        sensor_msgs_Imu = 'sensor_msgs/Imu'
        sensor_msgs_JointState = 'sensor_msgs/JointState'
        sensor_msgs_Joy = 'sensor_msgs/Joy'
        sensor_msgs_JoyFeedback = 'sensor_msgs/JoyFeedback'
        sensor_msgs_JoyFeedbackArray = 'sensor_msgs/JoyFeedbackArray'
        sensor_msgs_LaserEcho = 'sensor_msgs/LaserEcho'
        sensor_msgs_LaserScan = 'sensor_msgs/LaserScan'
        sensor_msgs_MagneticField = 'sensor_msgs/MagneticField'
        sensor_msgs_MultiDOFJointState = 'sensor_msgs/MultiDOFJointState'
        sensor_msgs_MultiEchoLaserScan = 'sensor_msgs/MultiEchoLaserScan'
        sensor_msgs_NavSatFix = 'sensor_msgs/NavSatFix'
        sensor_msgs_NavSatStatus = 'sensor_msgs/NavSatStatus'
        sensor_msgs_PointCloud = 'sensor_msgs/PointCloud'
        sensor_msgs_PointCloud2 = 'sensor_msgs/PointCloud2'
        sensor_msgs_PointField = 'sensor_msgs/PointField'
        sensor_msgs_Range = 'sensor_msgs/Range'
        sensor_msgs_RegionOfInterest = 'sensor_msgs/RegionOfInterest'
        sensor_msgs_RelativeHumidity = 'sensor_msgs/RelativeHumidity'
        sensor_msgs_SetCameraInfoRequest = 'sensor_msgs/SetCameraInfoRequest'
        sensor_msgs_SetCameraInfoResponse = 'sensor_msgs/SetCameraInfoResponse'
        sensor_msgs_Temperature = 'sensor_msgs/Temperature'
        sensor_msgs_TimeReference = 'sensor_msgs/TimeReference'
        shape_msgs_Mesh = 'shape_msgs/Mesh'
        shape_msgs_MeshTriangle = 'shape_msgs/MeshTriangle'
        shape_msgs_Plane = 'shape_msgs/Plane'
        shape_msgs_SolidPrimitive = 'shape_msgs/SolidPrimitive'
        smach_msgs_SmachContainerInitialStatusCmd = 'smach_msgs/SmachContainerInitialStatusCmd'
        smach_msgs_SmachContainerStatus = 'smach_msgs/SmachContainerStatus'
        smach_msgs_SmachContainerStructure = 'smach_msgs/SmachContainerStructure'
        std_msgs_Bool = 'std_msgs/Bool'
        std_msgs_Byte = 'std_msgs/Byte'
        std_msgs_Char = 'std_msgs/Char'
        std_msgs_ColorRGBA = 'std_msgs/ColorRGBA'
        std_msgs_Duration = 'std_msgs/Duration'
        std_msgs_Empty = 'std_msgs/Empty'
        std_msgs_Float32 = 'std_msgs/Float32'
        std_msgs_Float32MultiArray = 'std_msgs/Float32MultiArray'
        std_msgs_Float64 = 'std_msgs/Float64'
        std_msgs_Float64MultiArray = 'std_msgs/Float64MultiArray'
        std_msgs_Header = 'std_msgs/Header'
        std_msgs_Int16 = 'std_msgs/Int16'
        std_msgs_Int16MultiArray = 'std_msgs/Int16MultiArray'
        std_msgs_Int32 = 'std_msgs/Int32'
        std_msgs_Int32MultiArray = 'std_msgs/Int32MultiArray'
        std_msgs_Int64 = 'std_msgs/Int64'
        std_msgs_Int64MultiArray = 'std_msgs/Int64MultiArray'
        std_msgs_Int8 = 'std_msgs/Int8'
        std_msgs_Int8MultiArray = 'std_msgs/Int8MultiArray'
        std_msgs_MultiArrayDimension = 'std_msgs/MultiArrayDimension'
        std_msgs_MultiArrayLayout = 'std_msgs/MultiArrayLayout'
        std_msgs_String = 'std_msgs/String'
        std_msgs_Time = 'std_msgs/Time'
        std_msgs_UInt16 = 'std_msgs/UInt16'
        std_msgs_UInt16MultiArray = 'std_msgs/UInt16MultiArray'
        std_msgs_UInt32 = 'std_msgs/UInt32'
        std_msgs_UInt32MultiArray = 'std_msgs/UInt32MultiArray'
        std_msgs_UInt64 = 'std_msgs/UInt64'
        std_msgs_UInt64MultiArray = 'std_msgs/UInt64MultiArray'
        std_msgs_UInt8 = 'std_msgs/UInt8'
        std_msgs_UInt8MultiArray = 'std_msgs/UInt8MultiArray'
        std_srvs_EmptyRequest = 'std_srvs/EmptyRequest'
        std_srvs_EmptyResponse = 'std_srvs/EmptyResponse'
        std_srvs_SetBoolRequest = 'std_srvs/SetBoolRequest'
        std_srvs_SetBoolResponse = 'std_srvs/SetBoolResponse'
        std_srvs_TriggerRequest = 'std_srvs/TriggerRequest'
        std_srvs_TriggerResponse = 'std_srvs/TriggerResponse'
        stereo_msgs_DisparityImage = 'stereo_msgs/DisparityImage'
        tf2_msgs_FrameGraphRequest = 'tf2_msgs/FrameGraphRequest'
        tf2_msgs_FrameGraphResponse = 'tf2_msgs/FrameGraphResponse'
        tf2_msgs_LookupTransformActionGoal = 'tf2_msgs/LookupTransformActionGoal'
        tf2_msgs_LookupTransformFeedback = 'tf2_msgs/LookupTransformFeedback'
        tf2_msgs_LookupTransformGoal = 'tf2_msgs/LookupTransformGoal'
        tf2_msgs_TFMessage = 'tf2_msgs/TFMessage'
        topic_tools_DemuxAddRequest = 'topic_tools/DemuxAddRequest'
        topic_tools_DemuxAddResponse = 'topic_tools/DemuxAddResponse'
        topic_tools_DemuxDeleteRequest = 'topic_tools/DemuxDeleteRequest'
        topic_tools_DemuxDeleteResponse = 'topic_tools/DemuxDeleteResponse'
        topic_tools_DemuxListRequest = 'topic_tools/DemuxListRequest'
        topic_tools_DemuxListResponse = 'topic_tools/DemuxListResponse'
        topic_tools_DemuxSelectRequest = 'topic_tools/DemuxSelectRequest'
        topic_tools_DemuxSelectResponse = 'topic_tools/DemuxSelectResponse'
        topic_tools_MuxAddRequest = 'topic_tools/MuxAddRequest'
        topic_tools_MuxAddResponse = 'topic_tools/MuxAddResponse'
        topic_tools_MuxDeleteRequest = 'topic_tools/MuxDeleteRequest'
        topic_tools_MuxDeleteResponse = 'topic_tools/MuxDeleteResponse'
        topic_tools_MuxListRequest = 'topic_tools/MuxListRequest'
        topic_tools_MuxListResponse = 'topic_tools/MuxListResponse'
        topic_tools_MuxSelectRequest = 'topic_tools/MuxSelectRequest'
        topic_tools_MuxSelectResponse = 'topic_tools/MuxSelectResponse'
        trajectory_msgs_JointTrajectory = 'trajectory_msgs/JointTrajectory'
        trajectory_msgs_JointTrajectoryPoint = 'trajectory_msgs/JointTrajectoryPoint'
        trajectory_msgs_MultiDOFJointTrajectory = 'trajectory_msgs/MultiDOFJointTrajectory'
        trajectory_msgs_MultiDOFJointTrajectoryPoint = 'trajectory_msgs/MultiDOFJointTrajectoryPoint'
        visualization_msgs_ImageMarker = 'visualization_msgs/ImageMarker'
        visualization_msgs_InteractiveMarker = 'visualization_msgs/InteractiveMarker'
        visualization_msgs_InteractiveMarkerControl = 'visualization_msgs/InteractiveMarkerControl'
        visualization_msgs_InteractiveMarkerFeedback = 'visualization_msgs/InteractiveMarkerFeedback'
        visualization_msgs_InteractiveMarkerInit = 'visualization_msgs/InteractiveMarkerInit'
        visualization_msgs_InteractiveMarkerPose = 'visualization_msgs/InteractiveMarkerPose'
        visualization_msgs_InteractiveMarkerUpdate = 'visualization_msgs/InteractiveMarkerUpdate'
        visualization_msgs_Marker = 'visualization_msgs/Marker'
        visualization_msgs_MarkerArray = 'visualization_msgs/MarkerArray'
        visualization_msgs_MenuEntry = 'visualization_msgs/MenuEntry'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(279, 1);
                msgList{1} = 'actionlib/TestAction';
                msgList{2} = 'actionlib/TestActionFeedback';
                msgList{3} = 'actionlib/TestActionGoal';
                msgList{4} = 'actionlib/TestActionResult';
                msgList{5} = 'actionlib/TestFeedback';
                msgList{6} = 'actionlib/TestGoal';
                msgList{7} = 'actionlib/TestRequestAction';
                msgList{8} = 'actionlib/TestRequestActionFeedback';
                msgList{9} = 'actionlib/TestRequestActionGoal';
                msgList{10} = 'actionlib/TestRequestActionResult';
                msgList{11} = 'actionlib/TestRequestFeedback';
                msgList{12} = 'actionlib/TestRequestGoal';
                msgList{13} = 'actionlib/TestRequestResult';
                msgList{14} = 'actionlib/TestResult';
                msgList{15} = 'actionlib/TwoIntsAction';
                msgList{16} = 'actionlib/TwoIntsActionFeedback';
                msgList{17} = 'actionlib/TwoIntsActionGoal';
                msgList{18} = 'actionlib/TwoIntsActionResult';
                msgList{19} = 'actionlib/TwoIntsFeedback';
                msgList{20} = 'actionlib/TwoIntsGoal';
                msgList{21} = 'actionlib/TwoIntsResult';
                msgList{22} = 'actionlib_msgs/GoalID';
                msgList{23} = 'actionlib_msgs/GoalStatus';
                msgList{24} = 'actionlib_msgs/GoalStatusArray';
                msgList{25} = 'control_msgs/FollowJointTrajectoryAction';
                msgList{26} = 'control_msgs/FollowJointTrajectoryActionFeedback';
                msgList{27} = 'control_msgs/FollowJointTrajectoryActionGoal';
                msgList{28} = 'control_msgs/FollowJointTrajectoryActionResult';
                msgList{29} = 'control_msgs/FollowJointTrajectoryFeedback';
                msgList{30} = 'control_msgs/FollowJointTrajectoryGoal';
                msgList{31} = 'control_msgs/FollowJointTrajectoryResult';
                msgList{32} = 'control_msgs/GripperCommand';
                msgList{33} = 'control_msgs/GripperCommandAction';
                msgList{34} = 'control_msgs/GripperCommandActionFeedback';
                msgList{35} = 'control_msgs/GripperCommandActionGoal';
                msgList{36} = 'control_msgs/GripperCommandActionResult';
                msgList{37} = 'control_msgs/GripperCommandFeedback';
                msgList{38} = 'control_msgs/GripperCommandGoal';
                msgList{39} = 'control_msgs/GripperCommandResult';
                msgList{40} = 'control_msgs/JointControllerState';
                msgList{41} = 'control_msgs/JointJog';
                msgList{42} = 'control_msgs/JointTolerance';
                msgList{43} = 'control_msgs/JointTrajectoryAction';
                msgList{44} = 'control_msgs/JointTrajectoryActionFeedback';
                msgList{45} = 'control_msgs/JointTrajectoryActionGoal';
                msgList{46} = 'control_msgs/JointTrajectoryActionResult';
                msgList{47} = 'control_msgs/JointTrajectoryControllerState';
                msgList{48} = 'control_msgs/JointTrajectoryFeedback';
                msgList{49} = 'control_msgs/JointTrajectoryGoal';
                msgList{50} = 'control_msgs/JointTrajectoryResult';
                msgList{51} = 'control_msgs/PidState';
                msgList{52} = 'control_msgs/PointHeadAction';
                msgList{53} = 'control_msgs/PointHeadActionFeedback';
                msgList{54} = 'control_msgs/PointHeadActionGoal';
                msgList{55} = 'control_msgs/PointHeadActionResult';
                msgList{56} = 'control_msgs/PointHeadFeedback';
                msgList{57} = 'control_msgs/PointHeadGoal';
                msgList{58} = 'control_msgs/PointHeadResult';
                msgList{59} = 'control_msgs/QueryCalibrationStateRequest';
                msgList{60} = 'control_msgs/QueryCalibrationStateResponse';
                msgList{61} = 'control_msgs/QueryTrajectoryStateRequest';
                msgList{62} = 'control_msgs/QueryTrajectoryStateResponse';
                msgList{63} = 'control_msgs/SingleJointPositionAction';
                msgList{64} = 'control_msgs/SingleJointPositionActionFeedback';
                msgList{65} = 'control_msgs/SingleJointPositionActionGoal';
                msgList{66} = 'control_msgs/SingleJointPositionActionResult';
                msgList{67} = 'control_msgs/SingleJointPositionFeedback';
                msgList{68} = 'control_msgs/SingleJointPositionGoal';
                msgList{69} = 'control_msgs/SingleJointPositionResult';
                msgList{70} = 'diagnostic_msgs/AddDiagnosticsRequest';
                msgList{71} = 'diagnostic_msgs/AddDiagnosticsResponse';
                msgList{72} = 'diagnostic_msgs/DiagnosticArray';
                msgList{73} = 'diagnostic_msgs/DiagnosticStatus';
                msgList{74} = 'diagnostic_msgs/KeyValue';
                msgList{75} = 'diagnostic_msgs/SelfTestRequest';
                msgList{76} = 'diagnostic_msgs/SelfTestResponse';
                msgList{77} = 'dynamic_reconfigure/BoolParameter';
                msgList{78} = 'dynamic_reconfigure/Config';
                msgList{79} = 'dynamic_reconfigure/ConfigDescription';
                msgList{80} = 'dynamic_reconfigure/DoubleParameter';
                msgList{81} = 'dynamic_reconfigure/Group';
                msgList{82} = 'dynamic_reconfigure/GroupState';
                msgList{83} = 'dynamic_reconfigure/IntParameter';
                msgList{84} = 'dynamic_reconfigure/ParamDescription';
                msgList{85} = 'dynamic_reconfigure/ReconfigureRequest';
                msgList{86} = 'dynamic_reconfigure/ReconfigureResponse';
                msgList{87} = 'dynamic_reconfigure/SensorLevels';
                msgList{88} = 'dynamic_reconfigure/StrParameter';
                msgList{89} = 'geometry_msgs/Accel';
                msgList{90} = 'geometry_msgs/AccelStamped';
                msgList{91} = 'geometry_msgs/AccelWithCovariance';
                msgList{92} = 'geometry_msgs/AccelWithCovarianceStamped';
                msgList{93} = 'geometry_msgs/Inertia';
                msgList{94} = 'geometry_msgs/InertiaStamped';
                msgList{95} = 'geometry_msgs/Point';
                msgList{96} = 'geometry_msgs/Point32';
                msgList{97} = 'geometry_msgs/PointStamped';
                msgList{98} = 'geometry_msgs/Polygon';
                msgList{99} = 'geometry_msgs/PolygonStamped';
                msgList{100} = 'geometry_msgs/Pose';
                msgList{101} = 'geometry_msgs/Pose2D';
                msgList{102} = 'geometry_msgs/PoseArray';
                msgList{103} = 'geometry_msgs/PoseStamped';
                msgList{104} = 'geometry_msgs/PoseWithCovariance';
                msgList{105} = 'geometry_msgs/PoseWithCovarianceStamped';
                msgList{106} = 'geometry_msgs/Quaternion';
                msgList{107} = 'geometry_msgs/QuaternionStamped';
                msgList{108} = 'geometry_msgs/Transform';
                msgList{109} = 'geometry_msgs/TransformStamped';
                msgList{110} = 'geometry_msgs/Twist';
                msgList{111} = 'geometry_msgs/TwistStamped';
                msgList{112} = 'geometry_msgs/TwistWithCovariance';
                msgList{113} = 'geometry_msgs/TwistWithCovarianceStamped';
                msgList{114} = 'geometry_msgs/Vector3';
                msgList{115} = 'geometry_msgs/Vector3Stamped';
                msgList{116} = 'geometry_msgs/Wrench';
                msgList{117} = 'geometry_msgs/WrenchStamped';
                msgList{118} = 'map_msgs/GetMapROIRequest';
                msgList{119} = 'map_msgs/GetMapROIResponse';
                msgList{120} = 'map_msgs/GetPointMapROIRequest';
                msgList{121} = 'map_msgs/GetPointMapROIResponse';
                msgList{122} = 'map_msgs/GetPointMapRequest';
                msgList{123} = 'map_msgs/GetPointMapResponse';
                msgList{124} = 'map_msgs/OccupancyGridUpdate';
                msgList{125} = 'map_msgs/PointCloud2Update';
                msgList{126} = 'map_msgs/ProjectedMap';
                msgList{127} = 'map_msgs/ProjectedMapInfo';
                msgList{128} = 'map_msgs/ProjectedMapsInfoRequest';
                msgList{129} = 'map_msgs/ProjectedMapsInfoResponse';
                msgList{130} = 'map_msgs/SaveMapRequest';
                msgList{131} = 'map_msgs/SaveMapResponse';
                msgList{132} = 'map_msgs/SetMapProjectionsRequest';
                msgList{133} = 'map_msgs/SetMapProjectionsResponse';
                msgList{134} = 'nav_msgs/GetMapAction';
                msgList{135} = 'nav_msgs/GetMapActionFeedback';
                msgList{136} = 'nav_msgs/GetMapActionGoal';
                msgList{137} = 'nav_msgs/GetMapActionResult';
                msgList{138} = 'nav_msgs/GetMapFeedback';
                msgList{139} = 'nav_msgs/GetMapGoal';
                msgList{140} = 'nav_msgs/GetMapRequest';
                msgList{141} = 'nav_msgs/GetMapResponse';
                msgList{142} = 'nav_msgs/GetMapResult';
                msgList{143} = 'nav_msgs/GetPlanRequest';
                msgList{144} = 'nav_msgs/GetPlanResponse';
                msgList{145} = 'nav_msgs/GridCells';
                msgList{146} = 'nav_msgs/MapMetaData';
                msgList{147} = 'nav_msgs/OccupancyGrid';
                msgList{148} = 'nav_msgs/Odometry';
                msgList{149} = 'nav_msgs/Path';
                msgList{150} = 'nav_msgs/SetMapRequest';
                msgList{151} = 'nav_msgs/SetMapResponse';
                msgList{152} = 'roscpp/EmptyRequest';
                msgList{153} = 'roscpp/EmptyResponse';
                msgList{154} = 'roscpp/GetLoggersRequest';
                msgList{155} = 'roscpp/GetLoggersResponse';
                msgList{156} = 'roscpp/Logger';
                msgList{157} = 'roscpp/SetLoggerLevelRequest';
                msgList{158} = 'roscpp/SetLoggerLevelResponse';
                msgList{159} = 'roscpp_tutorials/TwoIntsRequest';
                msgList{160} = 'roscpp_tutorials/TwoIntsResponse';
                msgList{161} = 'rosgraph_msgs/Clock';
                msgList{162} = 'rosgraph_msgs/Log';
                msgList{163} = 'rosgraph_msgs/TopicStatistics';
                msgList{164} = 'sensor_msgs/BatteryState';
                msgList{165} = 'sensor_msgs/CameraInfo';
                msgList{166} = 'sensor_msgs/ChannelFloat32';
                msgList{167} = 'sensor_msgs/CompressedImage';
                msgList{168} = 'sensor_msgs/FluidPressure';
                msgList{169} = 'sensor_msgs/Illuminance';
                msgList{170} = 'sensor_msgs/Image';
                msgList{171} = 'sensor_msgs/Imu';
                msgList{172} = 'sensor_msgs/JointState';
                msgList{173} = 'sensor_msgs/Joy';
                msgList{174} = 'sensor_msgs/JoyFeedback';
                msgList{175} = 'sensor_msgs/JoyFeedbackArray';
                msgList{176} = 'sensor_msgs/LaserEcho';
                msgList{177} = 'sensor_msgs/LaserScan';
                msgList{178} = 'sensor_msgs/MagneticField';
                msgList{179} = 'sensor_msgs/MultiDOFJointState';
                msgList{180} = 'sensor_msgs/MultiEchoLaserScan';
                msgList{181} = 'sensor_msgs/NavSatFix';
                msgList{182} = 'sensor_msgs/NavSatStatus';
                msgList{183} = 'sensor_msgs/PointCloud';
                msgList{184} = 'sensor_msgs/PointCloud2';
                msgList{185} = 'sensor_msgs/PointField';
                msgList{186} = 'sensor_msgs/Range';
                msgList{187} = 'sensor_msgs/RegionOfInterest';
                msgList{188} = 'sensor_msgs/RelativeHumidity';
                msgList{189} = 'sensor_msgs/SetCameraInfoRequest';
                msgList{190} = 'sensor_msgs/SetCameraInfoResponse';
                msgList{191} = 'sensor_msgs/Temperature';
                msgList{192} = 'sensor_msgs/TimeReference';
                msgList{193} = 'shape_msgs/Mesh';
                msgList{194} = 'shape_msgs/MeshTriangle';
                msgList{195} = 'shape_msgs/Plane';
                msgList{196} = 'shape_msgs/SolidPrimitive';
                msgList{197} = 'smach_msgs/SmachContainerInitialStatusCmd';
                msgList{198} = 'smach_msgs/SmachContainerStatus';
                msgList{199} = 'smach_msgs/SmachContainerStructure';
                msgList{200} = 'std_msgs/Bool';
                msgList{201} = 'std_msgs/Byte';
                msgList{202} = 'std_msgs/ByteMultiArray';
                msgList{203} = 'std_msgs/Char';
                msgList{204} = 'std_msgs/ColorRGBA';
                msgList{205} = 'std_msgs/Duration';
                msgList{206} = 'std_msgs/Empty';
                msgList{207} = 'std_msgs/Float32';
                msgList{208} = 'std_msgs/Float32MultiArray';
                msgList{209} = 'std_msgs/Float64';
                msgList{210} = 'std_msgs/Float64MultiArray';
                msgList{211} = 'std_msgs/Header';
                msgList{212} = 'std_msgs/Int16';
                msgList{213} = 'std_msgs/Int16MultiArray';
                msgList{214} = 'std_msgs/Int32';
                msgList{215} = 'std_msgs/Int32MultiArray';
                msgList{216} = 'std_msgs/Int64';
                msgList{217} = 'std_msgs/Int64MultiArray';
                msgList{218} = 'std_msgs/Int8';
                msgList{219} = 'std_msgs/Int8MultiArray';
                msgList{220} = 'std_msgs/MultiArrayDimension';
                msgList{221} = 'std_msgs/MultiArrayLayout';
                msgList{222} = 'std_msgs/String';
                msgList{223} = 'std_msgs/Time';
                msgList{224} = 'std_msgs/UInt16';
                msgList{225} = 'std_msgs/UInt16MultiArray';
                msgList{226} = 'std_msgs/UInt32';
                msgList{227} = 'std_msgs/UInt32MultiArray';
                msgList{228} = 'std_msgs/UInt64';
                msgList{229} = 'std_msgs/UInt64MultiArray';
                msgList{230} = 'std_msgs/UInt8';
                msgList{231} = 'std_msgs/UInt8MultiArray';
                msgList{232} = 'std_srvs/EmptyRequest';
                msgList{233} = 'std_srvs/EmptyResponse';
                msgList{234} = 'std_srvs/SetBoolRequest';
                msgList{235} = 'std_srvs/SetBoolResponse';
                msgList{236} = 'std_srvs/TriggerRequest';
                msgList{237} = 'std_srvs/TriggerResponse';
                msgList{238} = 'stereo_msgs/DisparityImage';
                msgList{239} = 'tf2_msgs/FrameGraphRequest';
                msgList{240} = 'tf2_msgs/FrameGraphResponse';
                msgList{241} = 'tf2_msgs/LookupTransformAction';
                msgList{242} = 'tf2_msgs/LookupTransformActionFeedback';
                msgList{243} = 'tf2_msgs/LookupTransformActionGoal';
                msgList{244} = 'tf2_msgs/LookupTransformActionResult';
                msgList{245} = 'tf2_msgs/LookupTransformFeedback';
                msgList{246} = 'tf2_msgs/LookupTransformGoal';
                msgList{247} = 'tf2_msgs/LookupTransformResult';
                msgList{248} = 'tf2_msgs/TF2Error';
                msgList{249} = 'tf2_msgs/TFMessage';
                msgList{250} = 'topic_tools/DemuxAddRequest';
                msgList{251} = 'topic_tools/DemuxAddResponse';
                msgList{252} = 'topic_tools/DemuxDeleteRequest';
                msgList{253} = 'topic_tools/DemuxDeleteResponse';
                msgList{254} = 'topic_tools/DemuxListRequest';
                msgList{255} = 'topic_tools/DemuxListResponse';
                msgList{256} = 'topic_tools/DemuxSelectRequest';
                msgList{257} = 'topic_tools/DemuxSelectResponse';
                msgList{258} = 'topic_tools/MuxAddRequest';
                msgList{259} = 'topic_tools/MuxAddResponse';
                msgList{260} = 'topic_tools/MuxDeleteRequest';
                msgList{261} = 'topic_tools/MuxDeleteResponse';
                msgList{262} = 'topic_tools/MuxListRequest';
                msgList{263} = 'topic_tools/MuxListResponse';
                msgList{264} = 'topic_tools/MuxSelectRequest';
                msgList{265} = 'topic_tools/MuxSelectResponse';
                msgList{266} = 'trajectory_msgs/JointTrajectory';
                msgList{267} = 'trajectory_msgs/JointTrajectoryPoint';
                msgList{268} = 'trajectory_msgs/MultiDOFJointTrajectory';
                msgList{269} = 'trajectory_msgs/MultiDOFJointTrajectoryPoint';
                msgList{270} = 'visualization_msgs/ImageMarker';
                msgList{271} = 'visualization_msgs/InteractiveMarker';
                msgList{272} = 'visualization_msgs/InteractiveMarkerControl';
                msgList{273} = 'visualization_msgs/InteractiveMarkerFeedback';
                msgList{274} = 'visualization_msgs/InteractiveMarkerInit';
                msgList{275} = 'visualization_msgs/InteractiveMarkerPose';
                msgList{276} = 'visualization_msgs/InteractiveMarkerUpdate';
                msgList{277} = 'visualization_msgs/Marker';
                msgList{278} = 'visualization_msgs/MarkerArray';
                msgList{279} = 'visualization_msgs/MenuEntry';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(31, 1);
                svcList{1} = 'control_msgs/QueryCalibrationState';
                svcList{2} = 'control_msgs/QueryTrajectoryState';
                svcList{3} = 'diagnostic_msgs/AddDiagnostics';
                svcList{4} = 'diagnostic_msgs/SelfTest';
                svcList{5} = 'dynamic_reconfigure/Reconfigure';
                svcList{6} = 'map_msgs/GetMapROI';
                svcList{7} = 'map_msgs/GetPointMap';
                svcList{8} = 'map_msgs/GetPointMapROI';
                svcList{9} = 'map_msgs/ProjectedMapsInfo';
                svcList{10} = 'map_msgs/SaveMap';
                svcList{11} = 'map_msgs/SetMapProjections';
                svcList{12} = 'nav_msgs/GetMap';
                svcList{13} = 'nav_msgs/GetPlan';
                svcList{14} = 'nav_msgs/SetMap';
                svcList{15} = 'roscpp/Empty';
                svcList{16} = 'roscpp/GetLoggers';
                svcList{17} = 'roscpp/SetLoggerLevel';
                svcList{18} = 'roscpp_tutorials/TwoInts';
                svcList{19} = 'sensor_msgs/SetCameraInfo';
                svcList{20} = 'std_srvs/Empty';
                svcList{21} = 'std_srvs/SetBool';
                svcList{22} = 'std_srvs/Trigger';
                svcList{23} = 'tf2_msgs/FrameGraph';
                svcList{24} = 'topic_tools/DemuxAdd';
                svcList{25} = 'topic_tools/DemuxDelete';
                svcList{26} = 'topic_tools/DemuxList';
                svcList{27} = 'topic_tools/DemuxSelect';
                svcList{28} = 'topic_tools/MuxAdd';
                svcList{29} = 'topic_tools/MuxDelete';
                svcList{30} = 'topic_tools/MuxList';
                svcList{31} = 'topic_tools/MuxSelect';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(10, 1);
                actList{1} = 'actionlib/Test';
                actList{2} = 'actionlib/TestRequest';
                actList{3} = 'actionlib/TwoInts';
                actList{4} = 'control_msgs/GripperCommand';
                actList{5} = 'control_msgs/FollowJointTrajectory';
                actList{6} = 'control_msgs/JointTrajectory';
                actList{7} = 'control_msgs/PointHead';
                actList{8} = 'control_msgs/SingleJointPosition';
                actList{9} = 'nav_msgs/GetMap';
                actList{10} = 'tf2_msgs/LookupTransform';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
