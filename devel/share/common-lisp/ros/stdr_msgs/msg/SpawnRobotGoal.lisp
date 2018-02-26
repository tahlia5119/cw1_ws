; Auto-generated. Do not edit!


(cl:in-package stdr_msgs-msg)


;//! \htmlinclude SpawnRobotGoal.msg.html

(cl:defclass <SpawnRobotGoal> (roslisp-msg-protocol:ros-message)
  ((description
    :reader description
    :initarg :description
    :type stdr_msgs-msg:RobotMsg
    :initform (cl:make-instance 'stdr_msgs-msg:RobotMsg)))
)

(cl:defclass SpawnRobotGoal (<SpawnRobotGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SpawnRobotGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SpawnRobotGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stdr_msgs-msg:<SpawnRobotGoal> is deprecated: use stdr_msgs-msg:SpawnRobotGoal instead.")))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <SpawnRobotGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stdr_msgs-msg:description-val is deprecated.  Use stdr_msgs-msg:description instead.")
  (description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SpawnRobotGoal>) ostream)
  "Serializes a message object of type '<SpawnRobotGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'description) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SpawnRobotGoal>) istream)
  "Deserializes a message object of type '<SpawnRobotGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'description) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SpawnRobotGoal>)))
  "Returns string type for a message object of type '<SpawnRobotGoal>"
  "stdr_msgs/SpawnRobotGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SpawnRobotGoal)))
  "Returns string type for a message object of type 'SpawnRobotGoal"
  "stdr_msgs/SpawnRobotGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SpawnRobotGoal>)))
  "Returns md5sum for a message object of type '<SpawnRobotGoal>"
  "918af704950f3898fd078737c940764a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SpawnRobotGoal)))
  "Returns md5sum for a message object of type 'SpawnRobotGoal"
  "918af704950f3898fd078737c940764a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SpawnRobotGoal>)))
  "Returns full string definition for message of type '<SpawnRobotGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%stdr_msgs/RobotMsg description~%~%================================================================================~%MSG: stdr_msgs/RobotMsg~%geometry_msgs/Pose2D initialPose~%~%stdr_msgs/FootprintMsg footprint~%~%stdr_msgs/LaserSensorMsg[] laserSensors~%stdr_msgs/SonarSensorMsg[] sonarSensors~%stdr_msgs/RfidSensorMsg[] rfidSensors~%stdr_msgs/CO2SensorMsg[] co2Sensors~%stdr_msgs/SoundSensorMsg[] soundSensors~%stdr_msgs/ThermalSensorMsg[] thermalSensors~%~%stdr_msgs/KinematicMsg kinematicModel~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%================================================================================~%MSG: stdr_msgs/FootprintMsg~%geometry_msgs/Point[] points~%float32 radius # for circular footprints~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: stdr_msgs/LaserSensorMsg~%# Laser Sensor description~%float32 maxAngle~%float32 minAngle~%float32 maxRange~%float32 minRange~%int32 numRays~%~%stdr_msgs/Noise noise~%~%float32 frequency~%~%string frame_id~%geometry_msgs/Pose2D pose # sensor pose, relative to robot center~%~%================================================================================~%MSG: stdr_msgs/Noise~%bool noise~%float32 noiseMean~%float32 noiseStd~%~%================================================================================~%MSG: stdr_msgs/SonarSensorMsg~%# Rfid sensor description~%float32 maxRange~%float32 minRange~%float32 coneAngle~%~%float32 frequency~%stdr_msgs/Noise noise~%~%string frame_id~%geometry_msgs/Pose2D pose # sensor pose, relative to robot center~%~%================================================================================~%MSG: stdr_msgs/RfidSensorMsg~%# Rfid sensor description~%float32 maxRange~%float32 angleSpan~%float32 signalCutoff~%~%float32 frequency~%~%string frame_id~%geometry_msgs/Pose2D pose # sensor pose, relative to robot center~%~%================================================================================~%MSG: stdr_msgs/CO2SensorMsg~%# Sensor description~%~%float32 maxRange~%float32 frequency~%string frame_id~%~%# sensor pose, relative to robot center~%geometry_msgs/Pose2D pose ~%~%================================================================================~%MSG: stdr_msgs/SoundSensorMsg~%# Sensor description~%~%float32 maxRange~%float32 frequency~%float32 angleSpan~%string frame_id~%~%# sensor pose, relative to robot center~%geometry_msgs/Pose2D pose ~%~%================================================================================~%MSG: stdr_msgs/ThermalSensorMsg~%# Sensor description~%~%float32 maxRange~%float32 frequency~%float32 angleSpan~%string frame_id~%~%# sensor pose, relative to robot center~%geometry_msgs/Pose2D pose ~%~%================================================================================~%MSG: stdr_msgs/KinematicMsg~%# Container for the kinematic model parameters. The parameters selected/~%# are quite general. For a more accurate motion model a per-kinematic model~%# approach should be followed.~%# Parameters are in the form a_C_D, where C is affected by D.~%# ux is the linear speed~%# uy is the lateral speed (for omni vehicles)~%# w is the angular speed~%# g is a cofficient that directly affects the angular speed~%# For more information check the MotionController::sampleVelocities function.~%~%string type~%float32 a_ux_ux~%float32 a_ux_uy~%float32 a_ux_w~%float32 a_uy_ux~%float32 a_uy_uy~%float32 a_uy_w~%float32 a_w_ux~%float32 a_w_uy~%float32 a_w_w~%float32 a_g_ux~%float32 a_g_uy~%float32 a_g_w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SpawnRobotGoal)))
  "Returns full string definition for message of type 'SpawnRobotGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%stdr_msgs/RobotMsg description~%~%================================================================================~%MSG: stdr_msgs/RobotMsg~%geometry_msgs/Pose2D initialPose~%~%stdr_msgs/FootprintMsg footprint~%~%stdr_msgs/LaserSensorMsg[] laserSensors~%stdr_msgs/SonarSensorMsg[] sonarSensors~%stdr_msgs/RfidSensorMsg[] rfidSensors~%stdr_msgs/CO2SensorMsg[] co2Sensors~%stdr_msgs/SoundSensorMsg[] soundSensors~%stdr_msgs/ThermalSensorMsg[] thermalSensors~%~%stdr_msgs/KinematicMsg kinematicModel~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%================================================================================~%MSG: stdr_msgs/FootprintMsg~%geometry_msgs/Point[] points~%float32 radius # for circular footprints~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: stdr_msgs/LaserSensorMsg~%# Laser Sensor description~%float32 maxAngle~%float32 minAngle~%float32 maxRange~%float32 minRange~%int32 numRays~%~%stdr_msgs/Noise noise~%~%float32 frequency~%~%string frame_id~%geometry_msgs/Pose2D pose # sensor pose, relative to robot center~%~%================================================================================~%MSG: stdr_msgs/Noise~%bool noise~%float32 noiseMean~%float32 noiseStd~%~%================================================================================~%MSG: stdr_msgs/SonarSensorMsg~%# Rfid sensor description~%float32 maxRange~%float32 minRange~%float32 coneAngle~%~%float32 frequency~%stdr_msgs/Noise noise~%~%string frame_id~%geometry_msgs/Pose2D pose # sensor pose, relative to robot center~%~%================================================================================~%MSG: stdr_msgs/RfidSensorMsg~%# Rfid sensor description~%float32 maxRange~%float32 angleSpan~%float32 signalCutoff~%~%float32 frequency~%~%string frame_id~%geometry_msgs/Pose2D pose # sensor pose, relative to robot center~%~%================================================================================~%MSG: stdr_msgs/CO2SensorMsg~%# Sensor description~%~%float32 maxRange~%float32 frequency~%string frame_id~%~%# sensor pose, relative to robot center~%geometry_msgs/Pose2D pose ~%~%================================================================================~%MSG: stdr_msgs/SoundSensorMsg~%# Sensor description~%~%float32 maxRange~%float32 frequency~%float32 angleSpan~%string frame_id~%~%# sensor pose, relative to robot center~%geometry_msgs/Pose2D pose ~%~%================================================================================~%MSG: stdr_msgs/ThermalSensorMsg~%# Sensor description~%~%float32 maxRange~%float32 frequency~%float32 angleSpan~%string frame_id~%~%# sensor pose, relative to robot center~%geometry_msgs/Pose2D pose ~%~%================================================================================~%MSG: stdr_msgs/KinematicMsg~%# Container for the kinematic model parameters. The parameters selected/~%# are quite general. For a more accurate motion model a per-kinematic model~%# approach should be followed.~%# Parameters are in the form a_C_D, where C is affected by D.~%# ux is the linear speed~%# uy is the lateral speed (for omni vehicles)~%# w is the angular speed~%# g is a cofficient that directly affects the angular speed~%# For more information check the MotionController::sampleVelocities function.~%~%string type~%float32 a_ux_ux~%float32 a_ux_uy~%float32 a_ux_w~%float32 a_uy_ux~%float32 a_uy_uy~%float32 a_uy_w~%float32 a_w_ux~%float32 a_w_uy~%float32 a_w_w~%float32 a_g_ux~%float32 a_g_uy~%float32 a_g_w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SpawnRobotGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SpawnRobotGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'SpawnRobotGoal
    (cl:cons ':description (description msg))
))