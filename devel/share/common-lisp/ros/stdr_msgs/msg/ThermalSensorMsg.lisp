; Auto-generated. Do not edit!


(cl:in-package stdr_msgs-msg)


;//! \htmlinclude ThermalSensorMsg.msg.html

(cl:defclass <ThermalSensorMsg> (roslisp-msg-protocol:ros-message)
  ((maxRange
    :reader maxRange
    :initarg :maxRange
    :type cl:float
    :initform 0.0)
   (frequency
    :reader frequency
    :initarg :frequency
    :type cl:float
    :initform 0.0)
   (angleSpan
    :reader angleSpan
    :initarg :angleSpan
    :type cl:float
    :initform 0.0)
   (frame_id
    :reader frame_id
    :initarg :frame_id
    :type cl:string
    :initform "")
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D)))
)

(cl:defclass ThermalSensorMsg (<ThermalSensorMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ThermalSensorMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ThermalSensorMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name stdr_msgs-msg:<ThermalSensorMsg> is deprecated: use stdr_msgs-msg:ThermalSensorMsg instead.")))

(cl:ensure-generic-function 'maxRange-val :lambda-list '(m))
(cl:defmethod maxRange-val ((m <ThermalSensorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stdr_msgs-msg:maxRange-val is deprecated.  Use stdr_msgs-msg:maxRange instead.")
  (maxRange m))

(cl:ensure-generic-function 'frequency-val :lambda-list '(m))
(cl:defmethod frequency-val ((m <ThermalSensorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stdr_msgs-msg:frequency-val is deprecated.  Use stdr_msgs-msg:frequency instead.")
  (frequency m))

(cl:ensure-generic-function 'angleSpan-val :lambda-list '(m))
(cl:defmethod angleSpan-val ((m <ThermalSensorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stdr_msgs-msg:angleSpan-val is deprecated.  Use stdr_msgs-msg:angleSpan instead.")
  (angleSpan m))

(cl:ensure-generic-function 'frame_id-val :lambda-list '(m))
(cl:defmethod frame_id-val ((m <ThermalSensorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stdr_msgs-msg:frame_id-val is deprecated.  Use stdr_msgs-msg:frame_id instead.")
  (frame_id m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <ThermalSensorMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader stdr_msgs-msg:pose-val is deprecated.  Use stdr_msgs-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ThermalSensorMsg>) ostream)
  "Serializes a message object of type '<ThermalSensorMsg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'maxRange))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angleSpan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'frame_id))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ThermalSensorMsg>) istream)
  "Deserializes a message object of type '<ThermalSensorMsg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxRange) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frequency) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angleSpan) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ThermalSensorMsg>)))
  "Returns string type for a message object of type '<ThermalSensorMsg>"
  "stdr_msgs/ThermalSensorMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ThermalSensorMsg)))
  "Returns string type for a message object of type 'ThermalSensorMsg"
  "stdr_msgs/ThermalSensorMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ThermalSensorMsg>)))
  "Returns md5sum for a message object of type '<ThermalSensorMsg>"
  "fea53ce0501404c91ee4c18275946ca8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ThermalSensorMsg)))
  "Returns md5sum for a message object of type 'ThermalSensorMsg"
  "fea53ce0501404c91ee4c18275946ca8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ThermalSensorMsg>)))
  "Returns full string definition for message of type '<ThermalSensorMsg>"
  (cl:format cl:nil "# Sensor description~%~%float32 maxRange~%float32 frequency~%float32 angleSpan~%string frame_id~%~%# sensor pose, relative to robot center~%geometry_msgs/Pose2D pose ~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ThermalSensorMsg)))
  "Returns full string definition for message of type 'ThermalSensorMsg"
  (cl:format cl:nil "# Sensor description~%~%float32 maxRange~%float32 frequency~%float32 angleSpan~%string frame_id~%~%# sensor pose, relative to robot center~%geometry_msgs/Pose2D pose ~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ThermalSensorMsg>))
  (cl:+ 0
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'frame_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ThermalSensorMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ThermalSensorMsg
    (cl:cons ':maxRange (maxRange msg))
    (cl:cons ':frequency (frequency msg))
    (cl:cons ':angleSpan (angleSpan msg))
    (cl:cons ':frame_id (frame_id msg))
    (cl:cons ':pose (pose msg))
))
