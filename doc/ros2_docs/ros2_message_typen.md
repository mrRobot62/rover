# ROS2 Message Typen

| Message                          | Python Beispiel                              | Nutzungsart                       | Weitere Infos                                 |
| -------------------------------- | -------------------------------------------- | --------------------------------- | --------------------------------------------- |
| `std_msgs/msg/Bool`              | `from std_msgs.msg import Bool`              | Ein-Bit-Flag                      | `data: bool`                                  |
| `std_msgs/msg/Int32`             | `from std_msgs.msg import Int32`             | Integer-Nachricht                 | `data: int`                                   |
| `std_msgs/msg/Float32`           | `from std_msgs.msg import Float32`           | Gleitkommazahl                    | `data: float`                                 |
| `std_msgs/msg/String`            | `from std_msgs.msg import String`            | Textnachricht                     | `data: str`                                   |
| `std_msgs/msg/Empty`             | `from std_msgs.msg import Empty`             | Signal-Only-Nachricht             | kein `data`                                   |
| `std_msgs/msg/Header`            | `from std_msgs.msg import Header`            | Zeitstempel, Frame-ID             | Felder: `stamp`, `frame_id`                   |
| `std_msgs/msg/UInt8MultiArray`   | `from std_msgs.msg import UInt8MultiArray`   | Byte-Array                        | `layout`, `data: List[int]`                   |
| `std_msgs/msg/Float32MultiArray` | `from std_msgs.msg import Float32MultiArray` | Float-Array                       | `layout`, `data: List[float]`                 |
| `nav_msgs/msg/GridCells`         | `from nav_msgs.msg import GridCells`         | Raster-Zellenliste                | Felder: `header`, `cell_width`, `cells`       |
| `nav_msgs/msg/MapMetaData`       | `from nav_msgs.msg import MapMetaData`       | Karten-Metadaten                  | `map_load_time`, `resolution`, `height`, etc. |
| `nav_msgs/msg/OccupancyGrid`     | `from nav_msgs.msg import OccupancyGrid`     | 2D-Belegungsraster                | Header + MapMetaData + `data: List[int]`      |
| `nav_msgs/msg/Odometry`          | `from nav_msgs.msg import Odometry`          | Positions- & Geschwindigkeitsinfo | Felder: `pose`, `twist`                       |
| `nav_msgs/msg/Path`              | `from nav_msgs.msg import Path`              | Folgen von Posen                  | `poses: List[PoseStamped]`                    |



## Details zu bestimmten Message-Typ abrufen
Befehl: `ros2 interface show nav_msgs/msg/Odometry`

```
bernd@ros2pi5:~/ros2_ws$ ros2 interface show nav_msgs/msg/Odometry
# This represents an estimate of a position and velocity in free space.
# The pose in this message should be specified in the coordinate frame given by header.frame_id
# The twist in this message should be specified in the coordinate frame given by the child_frame_id

# Includes the frame id of the pose parent.
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

# Frame id the pose points to. The twist is in this coordinate frame.
string child_frame_id

# Estimated pose that is typically relative to a fixed world frame.
geometry_msgs/PoseWithCovariance pose
	Pose pose
		Point position
			float64 x
			float64 y
			float64 z
		Quaternion orientation
			float64 x 0
			float64 y 0
			float64 z 0
			float64 w 1
	float64[36] covariance

# Estimated linear and angular velocity relative to child_frame_id.
geometry_msgs/TwistWithCovariance twist
	Twist twist
		Vector3  linear
			float64 x
			float64 y
			float64 z
		Vector3  angular
			float64 x
			float64 y
			float64 z
	float64[36] covariance
bernd@ros2pi5:~/ros2_ws$ 
```

## komplette Übersicht
Befehl: `ros2 interface list`

```
bernd@ros2pi5:~/ros2_ws$ ros2 interface list
Messages:
    ackermann_msgs/msg/AckermannDrive
    ackermann_msgs/msg/AckermannDriveStamped
    action_msgs/msg/GoalInfo
    action_msgs/msg/GoalStatus
    action_msgs/msg/GoalStatusArray
    actionlib_msgs/msg/GoalID
    actionlib_msgs/msg/GoalStatus
    actionlib_msgs/msg/GoalStatusArray
    actuator_msgs/msg/Actuators
    actuator_msgs/msg/ActuatorsAngularPosition
    actuator_msgs/msg/ActuatorsAngularVelocity
    actuator_msgs/msg/ActuatorsLinearPosition
    actuator_msgs/msg/ActuatorsLinearVelocity
    actuator_msgs/msg/ActuatorsNormalized
    actuator_msgs/msg/ActuatorsPosition
    actuator_msgs/msg/ActuatorsVelocity
    bond/msg/Constants
    bond/msg/Status
    builtin_interfaces/msg/Duration
    builtin_interfaces/msg/Time
    cartographer_ros_msgs/msg/BagfileProgress
    cartographer_ros_msgs/msg/HistogramBucket
    cartographer_ros_msgs/msg/LandmarkEntry
    cartographer_ros_msgs/msg/LandmarkList
    cartographer_ros_msgs/msg/Metric
    cartographer_ros_msgs/msg/MetricFamily
    cartographer_ros_msgs/msg/MetricLabel
    cartographer_ros_msgs/msg/StatusCode
    cartographer_ros_msgs/msg/StatusResponse
    cartographer_ros_msgs/msg/SubmapEntry
    cartographer_ros_msgs/msg/SubmapList
    cartographer_ros_msgs/msg/SubmapTexture
    cartographer_ros_msgs/msg/TrajectoryStates
    control_msgs/msg/AdmittanceControllerState
    control_msgs/msg/DynamicInterfaceGroupValues
    control_msgs/msg/DynamicInterfaceValues
    control_msgs/msg/DynamicJointState
    control_msgs/msg/GripperCommand
    control_msgs/msg/InterfaceValue
    control_msgs/msg/JointComponentTolerance
    control_msgs/msg/JointControllerState
    control_msgs/msg/JointJog
    control_msgs/msg/JointTolerance
    control_msgs/msg/JointTrajectoryControllerState
    control_msgs/msg/MecanumDriveControllerState
    control_msgs/msg/MultiDOFCommand
    control_msgs/msg/MultiDOFStateStamped
    control_msgs/msg/PidState
    control_msgs/msg/SingleDOFState
    control_msgs/msg/SingleDOFStateStamped
    control_msgs/msg/SpeedScalingFactor
    control_msgs/msg/SteeringControllerStatus
    controller_manager_msgs/msg/ChainConnection
    controller_manager_msgs/msg/ControllerManagerActivity
    controller_manager_msgs/msg/ControllerState
    controller_manager_msgs/msg/HardwareComponentState
    controller_manager_msgs/msg/HardwareInterface
    controller_manager_msgs/msg/NamedLifecycleState
    diagnostic_msgs/msg/DiagnosticArray
    diagnostic_msgs/msg/DiagnosticStatus
    diagnostic_msgs/msg/KeyValue
    example_interfaces/msg/Bool
    example_interfaces/msg/Byte
    example_interfaces/msg/ByteMultiArray
    example_interfaces/msg/Char
    example_interfaces/msg/Empty
    example_interfaces/msg/Float32
    example_interfaces/msg/Float32MultiArray
    example_interfaces/msg/Float64
    example_interfaces/msg/Float64MultiArray
    example_interfaces/msg/Int16
    example_interfaces/msg/Int16MultiArray
    example_interfaces/msg/Int32
    example_interfaces/msg/Int32MultiArray
    example_interfaces/msg/Int64
    example_interfaces/msg/Int64MultiArray
    example_interfaces/msg/Int8
    example_interfaces/msg/Int8MultiArray
    example_interfaces/msg/MultiArrayDimension
    example_interfaces/msg/MultiArrayLayout
    example_interfaces/msg/String
    example_interfaces/msg/UInt16
    example_interfaces/msg/UInt16MultiArray
    example_interfaces/msg/UInt32
    example_interfaces/msg/UInt32MultiArray
    example_interfaces/msg/UInt64
    example_interfaces/msg/UInt64MultiArray
    example_interfaces/msg/UInt8
    example_interfaces/msg/UInt8MultiArray
    example_interfaces/msg/WString
    geographic_msgs/msg/BoundingBox
    geographic_msgs/msg/GeoPath
    geographic_msgs/msg/GeoPoint
    geographic_msgs/msg/GeoPointStamped
    geographic_msgs/msg/GeoPose
    geographic_msgs/msg/GeoPoseStamped
    geographic_msgs/msg/GeoPoseWithCovariance
    geographic_msgs/msg/GeoPoseWithCovarianceStamped
    geographic_msgs/msg/GeographicMap
    geographic_msgs/msg/GeographicMapChanges
    geographic_msgs/msg/KeyValue
    geographic_msgs/msg/MapFeature
    geographic_msgs/msg/RouteNetwork
    geographic_msgs/msg/RoutePath
    geographic_msgs/msg/RouteSegment
    geographic_msgs/msg/WayPoint
    geometry_msgs/msg/Accel
    geometry_msgs/msg/AccelStamped
    geometry_msgs/msg/AccelWithCovariance
    geometry_msgs/msg/AccelWithCovarianceStamped
    geometry_msgs/msg/Inertia
    geometry_msgs/msg/InertiaStamped
    geometry_msgs/msg/Point
    geometry_msgs/msg/Point32
    geometry_msgs/msg/PointStamped
    geometry_msgs/msg/Polygon
    geometry_msgs/msg/PolygonInstance
    geometry_msgs/msg/PolygonInstanceStamped
    geometry_msgs/msg/PolygonStamped
    geometry_msgs/msg/Pose
    geometry_msgs/msg/Pose2D
    geometry_msgs/msg/PoseArray
    geometry_msgs/msg/PoseStamped
    geometry_msgs/msg/PoseWithCovariance
    geometry_msgs/msg/PoseWithCovarianceStamped
    geometry_msgs/msg/Quaternion
    geometry_msgs/msg/QuaternionStamped
    geometry_msgs/msg/Transform
    geometry_msgs/msg/TransformStamped
    geometry_msgs/msg/Twist
    geometry_msgs/msg/TwistStamped
    geometry_msgs/msg/TwistWithCovariance
    geometry_msgs/msg/TwistWithCovarianceStamped
    geometry_msgs/msg/Vector3
    geometry_msgs/msg/Vector3Stamped
    geometry_msgs/msg/VelocityStamped
    geometry_msgs/msg/Wrench
    geometry_msgs/msg/WrenchStamped
    gps_msgs/msg/GPSFix
    gps_msgs/msg/GPSStatus
    graph_msgs/msg/Edges
    graph_msgs/msg/GeometryGraph
    lifecycle_msgs/msg/State
    lifecycle_msgs/msg/Transition
    lifecycle_msgs/msg/TransitionDescription
    lifecycle_msgs/msg/TransitionEvent
    map_msgs/msg/OccupancyGridUpdate
    map_msgs/msg/PointCloud2Update
    map_msgs/msg/ProjectedMap
    map_msgs/msg/ProjectedMapInfo
    micro_ros_msgs/msg/Entity
    micro_ros_msgs/msg/Graph
    micro_ros_msgs/msg/Node
    nav2_msgs/msg/BehaviorTreeLog
    nav2_msgs/msg/BehaviorTreeStatusChange
    nav2_msgs/msg/CollisionDetectorState
    nav2_msgs/msg/CollisionMonitorState
    nav2_msgs/msg/Costmap
    nav2_msgs/msg/CostmapFilterInfo
    nav2_msgs/msg/CostmapMetaData
    nav2_msgs/msg/CostmapUpdate
    nav2_msgs/msg/MissedWaypoint
    nav2_msgs/msg/Particle
    nav2_msgs/msg/ParticleCloud
    nav2_msgs/msg/SpeedLimit
    nav2_msgs/msg/VoxelGrid
    nav_msgs/msg/Goals
    nav_msgs/msg/GridCells
    nav_msgs/msg/MapMetaData
    nav_msgs/msg/OccupancyGrid
    nav_msgs/msg/Odometry
    nav_msgs/msg/Path
    object_recognition_msgs/msg/ObjectInformation
    object_recognition_msgs/msg/ObjectType
    object_recognition_msgs/msg/RecognizedObject
    object_recognition_msgs/msg/RecognizedObjectArray
    object_recognition_msgs/msg/Table
    object_recognition_msgs/msg/TableArray
    octomap_msgs/msg/Octomap
    octomap_msgs/msg/OctomapWithPose
    pal_statistics_msgs/msg/Statistic
    pal_statistics_msgs/msg/Statistics
    pal_statistics_msgs/msg/StatisticsNames
    pal_statistics_msgs/msg/StatisticsValues
    pcl_msgs/msg/ModelCoefficients
    pcl_msgs/msg/PointIndices
    pcl_msgs/msg/PolygonMesh
    pcl_msgs/msg/Vertices
    pendulum_msgs/msg/JointCommand
    pendulum_msgs/msg/JointState
    pendulum_msgs/msg/RttestResults
    rcl_interfaces/msg/FloatingPointRange
    rcl_interfaces/msg/IntegerRange
    rcl_interfaces/msg/ListParametersResult
    rcl_interfaces/msg/Log
    rcl_interfaces/msg/LoggerLevel
    rcl_interfaces/msg/Parameter
    rcl_interfaces/msg/ParameterDescriptor
    rcl_interfaces/msg/ParameterEvent
    rcl_interfaces/msg/ParameterEventDescriptors
    rcl_interfaces/msg/ParameterType
    rcl_interfaces/msg/ParameterValue
    rcl_interfaces/msg/SetLoggerLevelsResult
    rcl_interfaces/msg/SetParametersResult
    rmw_dds_common/msg/Gid
    rmw_dds_common/msg/NodeEntitiesInfo
    rmw_dds_common/msg/ParticipantEntitiesInfo
    ros_gz_interfaces/msg/Altimeter
    ros_gz_interfaces/msg/Contact
    ros_gz_interfaces/msg/Contacts
    ros_gz_interfaces/msg/Dataframe
    ros_gz_interfaces/msg/Entity
    ros_gz_interfaces/msg/EntityFactory
    ros_gz_interfaces/msg/EntityWrench
    ros_gz_interfaces/msg/Float32Array
    ros_gz_interfaces/msg/GuiCamera
    ros_gz_interfaces/msg/JointWrench
    ros_gz_interfaces/msg/Light
    ros_gz_interfaces/msg/LogicalCameraImage
    ros_gz_interfaces/msg/LogicalCameraImageModel
    ros_gz_interfaces/msg/MaterialColor
    ros_gz_interfaces/msg/ParamVec
    ros_gz_interfaces/msg/SensorNoise
    ros_gz_interfaces/msg/StringVec
    ros_gz_interfaces/msg/TrackVisual
    ros_gz_interfaces/msg/VideoRecord
    ros_gz_interfaces/msg/WorldControl
    ros_gz_interfaces/msg/WorldReset
    rosapi_msgs/msg/TypeDef
    rosbag2_interfaces/msg/ReadSplitEvent
    rosbag2_interfaces/msg/WriteSplitEvent
    rosbridge_msgs/msg/ConnectedClient
    rosbridge_msgs/msg/ConnectedClients
    rosgraph_msgs/msg/Clock
    rover_interfaces/msg/Battery
    rover_interfaces/msg/LEDMessage
    sensor_msgs/msg/BatteryState
    sensor_msgs/msg/CameraInfo
    sensor_msgs/msg/ChannelFloat32
    sensor_msgs/msg/CompressedImage
    sensor_msgs/msg/FluidPressure
    sensor_msgs/msg/Illuminance
    sensor_msgs/msg/Image
    sensor_msgs/msg/Imu
    sensor_msgs/msg/JointState
    sensor_msgs/msg/Joy
    sensor_msgs/msg/JoyFeedback
    sensor_msgs/msg/JoyFeedbackArray
    sensor_msgs/msg/LaserEcho
    sensor_msgs/msg/LaserScan
    sensor_msgs/msg/MagneticField
    sensor_msgs/msg/MultiDOFJointState
    sensor_msgs/msg/MultiEchoLaserScan
    sensor_msgs/msg/NavSatFix
    sensor_msgs/msg/NavSatStatus
    sensor_msgs/msg/PointCloud
    sensor_msgs/msg/PointCloud2
    sensor_msgs/msg/PointField
    sensor_msgs/msg/Range
    sensor_msgs/msg/RegionOfInterest
    sensor_msgs/msg/RelativeHumidity
    sensor_msgs/msg/Temperature
    sensor_msgs/msg/TimeReference
    service_msgs/msg/ServiceEventInfo
    shape_msgs/msg/Mesh
    shape_msgs/msg/MeshTriangle
    shape_msgs/msg/Plane
    shape_msgs/msg/SolidPrimitive
    statistics_msgs/msg/MetricsMessage
    statistics_msgs/msg/StatisticDataPoint
    statistics_msgs/msg/StatisticDataType
    std_msgs/msg/Bool
    std_msgs/msg/Byte
    std_msgs/msg/ByteMultiArray
    std_msgs/msg/Char
    std_msgs/msg/ColorRGBA
    std_msgs/msg/Empty
    std_msgs/msg/Float32
    std_msgs/msg/Float32MultiArray
    std_msgs/msg/Float64
    std_msgs/msg/Float64MultiArray
    std_msgs/msg/Header
    std_msgs/msg/Int16
    std_msgs/msg/Int16MultiArray
    std_msgs/msg/Int32
    std_msgs/msg/Int32MultiArray
    std_msgs/msg/Int64
    std_msgs/msg/Int64MultiArray
    std_msgs/msg/Int8
    std_msgs/msg/Int8MultiArray
    std_msgs/msg/MultiArrayDimension
    std_msgs/msg/MultiArrayLayout
    std_msgs/msg/String
    std_msgs/msg/UInt16
    std_msgs/msg/UInt16MultiArray
    std_msgs/msg/UInt32
    std_msgs/msg/UInt32MultiArray
    std_msgs/msg/UInt64
    std_msgs/msg/UInt64MultiArray
    std_msgs/msg/UInt8
    std_msgs/msg/UInt8MultiArray
    stereo_msgs/msg/DisparityImage
    tf2_msgs/msg/TF2Error
    tf2_msgs/msg/TFMessage
    theora_image_transport/msg/Packet
    trajectory_msgs/msg/JointTrajectory
    trajectory_msgs/msg/JointTrajectoryPoint
    trajectory_msgs/msg/MultiDOFJointTrajectory
    trajectory_msgs/msg/MultiDOFJointTrajectoryPoint
    turtlesim/msg/Color
    turtlesim/msg/Pose
    type_description_interfaces/msg/Field
    type_description_interfaces/msg/FieldType
    type_description_interfaces/msg/IndividualTypeDescription
    type_description_interfaces/msg/KeyValue
    type_description_interfaces/msg/TypeDescription
    type_description_interfaces/msg/TypeSource
    unique_identifier_msgs/msg/UUID
    vision_msgs/msg/BoundingBox2D
    vision_msgs/msg/BoundingBox2DArray
    vision_msgs/msg/BoundingBox3D
    vision_msgs/msg/BoundingBox3DArray
    vision_msgs/msg/Classification
    vision_msgs/msg/Detection2D
    vision_msgs/msg/Detection2DArray
    vision_msgs/msg/Detection3D
    vision_msgs/msg/Detection3DArray
    vision_msgs/msg/LabelInfo
    vision_msgs/msg/ObjectHypothesis
    vision_msgs/msg/ObjectHypothesisWithPose
    vision_msgs/msg/Point2D
    vision_msgs/msg/Pose2D
    vision_msgs/msg/VisionClass
    vision_msgs/msg/VisionInfo
    visualization_msgs/msg/ImageMarker
    visualization_msgs/msg/InteractiveMarker
    visualization_msgs/msg/InteractiveMarkerControl
    visualization_msgs/msg/InteractiveMarkerFeedback
    visualization_msgs/msg/InteractiveMarkerInit
    visualization_msgs/msg/InteractiveMarkerPose
    visualization_msgs/msg/InteractiveMarkerUpdate
    visualization_msgs/msg/Marker
    visualization_msgs/msg/MarkerArray
    visualization_msgs/msg/MenuEntry
    visualization_msgs/msg/MeshFile
    visualization_msgs/msg/UVCoordinate
Services:
    action_msgs/srv/CancelGoal
    cartographer_ros_msgs/srv/FinishTrajectory
    cartographer_ros_msgs/srv/GetTrajectoryStates
    cartographer_ros_msgs/srv/ReadMetrics
    cartographer_ros_msgs/srv/StartTrajectory
    cartographer_ros_msgs/srv/SubmapQuery
    cartographer_ros_msgs/srv/TrajectoryQuery
    cartographer_ros_msgs/srv/WriteState
    composition_interfaces/srv/ListNodes
    composition_interfaces/srv/LoadNode
    composition_interfaces/srv/UnloadNode
    control_msgs/srv/QueryCalibrationState
    control_msgs/srv/QueryTrajectoryState
    controller_manager_msgs/srv/ConfigureController
    controller_manager_msgs/srv/ListControllerTypes
    controller_manager_msgs/srv/ListControllers
    controller_manager_msgs/srv/ListHardwareComponents
    controller_manager_msgs/srv/ListHardwareInterfaces
    controller_manager_msgs/srv/LoadController
    controller_manager_msgs/srv/ReloadControllerLibraries
    controller_manager_msgs/srv/SetHardwareComponentState
    controller_manager_msgs/srv/SwitchController
    controller_manager_msgs/srv/UnloadController
    diagnostic_msgs/srv/AddDiagnostics
    diagnostic_msgs/srv/SelfTest
    example_interfaces/srv/AddTwoInts
    example_interfaces/srv/SetBool
    example_interfaces/srv/Trigger
    geographic_msgs/srv/GetGeoPath
    geographic_msgs/srv/GetGeographicMap
    geographic_msgs/srv/GetRoutePlan
    geographic_msgs/srv/UpdateGeographicMap
    lifecycle_msgs/srv/ChangeState
    lifecycle_msgs/srv/GetAvailableStates
    lifecycle_msgs/srv/GetAvailableTransitions
    lifecycle_msgs/srv/GetState
    logging_demo/srv/ConfigLogger
    map_msgs/srv/GetMapROI
    map_msgs/srv/GetPointMap
    map_msgs/srv/GetPointMapROI
    map_msgs/srv/ProjectedMapsInfo
    map_msgs/srv/SaveMap
    map_msgs/srv/SetMapProjections
    nav2_msgs/srv/ClearCostmapAroundRobot
    nav2_msgs/srv/ClearCostmapExceptRegion
    nav2_msgs/srv/ClearEntireCostmap
    nav2_msgs/srv/GetCost
    nav2_msgs/srv/GetCostmap
    nav2_msgs/srv/IsPathValid
    nav2_msgs/srv/LoadMap
    nav2_msgs/srv/ManageLifecycleNodes
    nav2_msgs/srv/ReloadDockDatabase
    nav2_msgs/srv/SaveMap
    nav2_msgs/srv/SetInitialPose
    nav_msgs/srv/GetMap
    nav_msgs/srv/GetPlan
    nav_msgs/srv/LoadMap
    nav_msgs/srv/SetMap
    object_recognition_msgs/srv/GetObjectInformation
    octomap_msgs/srv/BoundingBoxQuery
    octomap_msgs/srv/GetOctomap
    pcl_msgs/srv/UpdateFilename
    rcl_interfaces/srv/DescribeParameters
    rcl_interfaces/srv/GetLoggerLevels
    rcl_interfaces/srv/GetParameterTypes
    rcl_interfaces/srv/GetParameters
    rcl_interfaces/srv/ListParameters
    rcl_interfaces/srv/SetLoggerLevels
    rcl_interfaces/srv/SetParameters
    rcl_interfaces/srv/SetParametersAtomically
    ros_gz_interfaces/srv/ControlWorld
    ros_gz_interfaces/srv/DeleteEntity
    ros_gz_interfaces/srv/SetEntityPose
    ros_gz_interfaces/srv/SpawnEntity
    rosapi_msgs/srv/ActionFeedbackDetails
    rosapi_msgs/srv/ActionGoalDetails
    rosapi_msgs/srv/ActionResultDetails
    rosapi_msgs/srv/DeleteParam
    rosapi_msgs/srv/GetActionServers
    rosapi_msgs/srv/GetParam
    rosapi_msgs/srv/GetParamNames
    rosapi_msgs/srv/GetROSVersion
    rosapi_msgs/srv/GetTime
    rosapi_msgs/srv/HasParam
    rosapi_msgs/srv/Interfaces
    rosapi_msgs/srv/MessageDetails
    rosapi_msgs/srv/NodeDetails
    rosapi_msgs/srv/Nodes
    rosapi_msgs/srv/Publishers
    rosapi_msgs/srv/ServiceNode
    rosapi_msgs/srv/ServiceProviders
    rosapi_msgs/srv/ServiceRequestDetails
    rosapi_msgs/srv/ServiceResponseDetails
    rosapi_msgs/srv/ServiceType
    rosapi_msgs/srv/Services
    rosapi_msgs/srv/ServicesForType
    rosapi_msgs/srv/SetParam
    rosapi_msgs/srv/Subscribers
    rosapi_msgs/srv/TopicType
    rosapi_msgs/srv/Topics
    rosapi_msgs/srv/TopicsAndRawTypes
    rosapi_msgs/srv/TopicsForType
    rosbag2_interfaces/srv/Burst
    rosbag2_interfaces/srv/GetRate
    rosbag2_interfaces/srv/IsPaused
    rosbag2_interfaces/srv/Pause
    rosbag2_interfaces/srv/Play
    rosbag2_interfaces/srv/PlayNext
    rosbag2_interfaces/srv/Resume
    rosbag2_interfaces/srv/Seek
    rosbag2_interfaces/srv/SetRate
    rosbag2_interfaces/srv/Snapshot
    rosbag2_interfaces/srv/SplitBagfile
    rosbag2_interfaces/srv/Stop
    rosbag2_interfaces/srv/TogglePaused
    sensor_msgs/srv/SetCameraInfo
    std_srvs/srv/Empty
    std_srvs/srv/SetBool
    std_srvs/srv/Trigger
    tf2_msgs/srv/FrameGraph
    topic_tools_interfaces/srv/DemuxAdd
    topic_tools_interfaces/srv/DemuxDelete
    topic_tools_interfaces/srv/DemuxList
    topic_tools_interfaces/srv/DemuxSelect
    topic_tools_interfaces/srv/MuxAdd
    topic_tools_interfaces/srv/MuxDelete
    topic_tools_interfaces/srv/MuxList
    topic_tools_interfaces/srv/MuxSelect
    turtlesim/srv/Kill
    turtlesim/srv/SetPen
    turtlesim/srv/Spawn
    turtlesim/srv/TeleportAbsolute
    turtlesim/srv/TeleportRelative
    type_description_interfaces/srv/GetTypeDescription
    visualization_msgs/srv/GetInteractiveMarkers
Actions:
    action_tutorials_interfaces/action/Fibonacci
    control_msgs/action/FollowJointTrajectory
    control_msgs/action/GripperCommand
    control_msgs/action/JointTrajectory
    control_msgs/action/ParallelGripperCommand
    control_msgs/action/PointHead
    control_msgs/action/SingleJointPosition
    example_interfaces/action/Fibonacci
    nav2_msgs/action/AssistedTeleop
    nav2_msgs/action/BackUp
    nav2_msgs/action/ComputePathThroughPoses
    nav2_msgs/action/ComputePathToPose
    nav2_msgs/action/DockRobot
    nav2_msgs/action/DriveOnHeading
    nav2_msgs/action/DummyBehavior
    nav2_msgs/action/FollowGPSWaypoints
    nav2_msgs/action/FollowPath
    nav2_msgs/action/FollowWaypoints
    nav2_msgs/action/NavigateThroughPoses
    nav2_msgs/action/NavigateToPose
    nav2_msgs/action/SmoothPath
    nav2_msgs/action/Spin
    nav2_msgs/action/UndockRobot
    nav2_msgs/action/Wait
    object_recognition_msgs/action/ObjectRecognition
    tf2_msgs/action/LookupTransform
    turtlesim/action/RotateAbsolute
```
