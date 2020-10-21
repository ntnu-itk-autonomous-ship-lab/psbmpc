; Auto-generated. Do not edit!


(cl:in-package asv_msgs-msg)


;//! \htmlinclude Offset.msg.html

(cl:defclass <Offset> (roslisp-msg-protocol:ros-message)
  ((P_ca
    :reader P_ca
    :initarg :P_ca
    :type cl:float
    :initform 0.0)
   (Chi_ca
    :reader Chi_ca
    :initarg :Chi_ca
    :type cl:float
    :initform 0.0))
)

(cl:defclass Offset (<Offset>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Offset>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Offset)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name asv_msgs-msg:<Offset> is deprecated: use asv_msgs-msg:Offset instead.")))

(cl:ensure-generic-function 'P_ca-val :lambda-list '(m))
(cl:defmethod P_ca-val ((m <Offset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader asv_msgs-msg:P_ca-val is deprecated.  Use asv_msgs-msg:P_ca instead.")
  (P_ca m))

(cl:ensure-generic-function 'Chi_ca-val :lambda-list '(m))
(cl:defmethod Chi_ca-val ((m <Offset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader asv_msgs-msg:Chi_ca-val is deprecated.  Use asv_msgs-msg:Chi_ca instead.")
  (Chi_ca m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Offset>) ostream)
  "Serializes a message object of type '<Offset>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'P_ca))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Chi_ca))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Offset>) istream)
  "Deserializes a message object of type '<Offset>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'P_ca) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Chi_ca) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Offset>)))
  "Returns string type for a message object of type '<Offset>"
  "asv_msgs/Offset")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Offset)))
  "Returns string type for a message object of type 'Offset"
  "asv_msgs/Offset")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Offset>)))
  "Returns md5sum for a message object of type '<Offset>"
  "67c79bd0079a016b59fb0b6f517f39b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Offset)))
  "Returns md5sum for a message object of type 'Offset"
  "67c79bd0079a016b59fb0b6f517f39b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Offset>)))
  "Returns full string definition for message of type '<Offset>"
  (cl:format cl:nil "float64 P_ca~%float64 Chi_ca~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Offset)))
  "Returns full string definition for message of type 'Offset"
  (cl:format cl:nil "float64 P_ca~%float64 Chi_ca~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Offset>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Offset>))
  "Converts a ROS message object to a list"
  (cl:list 'Offset
    (cl:cons ':P_ca (P_ca msg))
    (cl:cons ':Chi_ca (Chi_ca msg))
))
