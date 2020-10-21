; Auto-generated. Do not edit!


(cl:in-package asv_msgs-msg)


;//! \htmlinclude ShipMetaData.msg.html

(cl:defclass <ShipMetaData> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0))
)

(cl:defclass ShipMetaData (<ShipMetaData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ShipMetaData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ShipMetaData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name asv_msgs-msg:<ShipMetaData> is deprecated: use asv_msgs-msg:ShipMetaData instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ShipMetaData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader asv_msgs-msg:id-val is deprecated.  Use asv_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ShipMetaData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader asv_msgs-msg:name-val is deprecated.  Use asv_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <ShipMetaData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader asv_msgs-msg:radius-val is deprecated.  Use asv_msgs-msg:radius instead.")
  (radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ShipMetaData>) ostream)
  "Serializes a message object of type '<ShipMetaData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ShipMetaData>) istream)
  "Deserializes a message object of type '<ShipMetaData>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ShipMetaData>)))
  "Returns string type for a message object of type '<ShipMetaData>"
  "asv_msgs/ShipMetaData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ShipMetaData)))
  "Returns string type for a message object of type 'ShipMetaData"
  "asv_msgs/ShipMetaData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ShipMetaData>)))
  "Returns md5sum for a message object of type '<ShipMetaData>"
  "a664eef656fd4f22be60ea17b5e54ee3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ShipMetaData)))
  "Returns md5sum for a message object of type 'ShipMetaData"
  "a664eef656fd4f22be60ea17b5e54ee3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ShipMetaData>)))
  "Returns full string definition for message of type '<ShipMetaData>"
  (cl:format cl:nil "uint8 id~%string name~%float32 radius~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ShipMetaData)))
  "Returns full string definition for message of type 'ShipMetaData"
  (cl:format cl:nil "uint8 id~%string name~%float32 radius~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ShipMetaData>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'name))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ShipMetaData>))
  "Converts a ROS message object to a list"
  (cl:list 'ShipMetaData
    (cl:cons ':id (id msg))
    (cl:cons ':name (name msg))
    (cl:cons ':radius (radius msg))
))
