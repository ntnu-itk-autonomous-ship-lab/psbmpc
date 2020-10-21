; Auto-generated. Do not edit!


(cl:in-package asv_msgs-msg)


;//! \htmlinclude StateArray.msg.html

(cl:defclass <StateArray> (roslisp-msg-protocol:ros-message)
  ((states
    :reader states
    :initarg :states
    :type (cl:vector asv_msgs-msg:State)
   :initform (cl:make-array 0 :element-type 'asv_msgs-msg:State :initial-element (cl:make-instance 'asv_msgs-msg:State))))
)

(cl:defclass StateArray (<StateArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StateArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StateArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name asv_msgs-msg:<StateArray> is deprecated: use asv_msgs-msg:StateArray instead.")))

(cl:ensure-generic-function 'states-val :lambda-list '(m))
(cl:defmethod states-val ((m <StateArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader asv_msgs-msg:states-val is deprecated.  Use asv_msgs-msg:states instead.")
  (states m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StateArray>) ostream)
  "Serializes a message object of type '<StateArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'states))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'states))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StateArray>) istream)
  "Deserializes a message object of type '<StateArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'states) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'states)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'asv_msgs-msg:State))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StateArray>)))
  "Returns string type for a message object of type '<StateArray>"
  "asv_msgs/StateArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StateArray)))
  "Returns string type for a message object of type 'StateArray"
  "asv_msgs/StateArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StateArray>)))
  "Returns md5sum for a message object of type '<StateArray>"
  "05e43d15ff5eab094661932878032f08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StateArray)))
  "Returns md5sum for a message object of type 'StateArray"
  "05e43d15ff5eab094661932878032f08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StateArray>)))
  "Returns full string definition for message of type '<StateArray>"
  (cl:format cl:nil "# An array of states~%State[] states~%================================================================================~%MSG: asv_msgs/State~%# The full state of a 3DOF surface vessel~%# eta: (x,y,psi)~%# nu:  (u,v,r)~%ShipMetaData header~%float64 x~%float64 y~%float64 psi~%float64 u~%float64 v~%float64 r~%================================================================================~%MSG: asv_msgs/ShipMetaData~%uint8 id~%string name~%float32 radius~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StateArray)))
  "Returns full string definition for message of type 'StateArray"
  (cl:format cl:nil "# An array of states~%State[] states~%================================================================================~%MSG: asv_msgs/State~%# The full state of a 3DOF surface vessel~%# eta: (x,y,psi)~%# nu:  (u,v,r)~%ShipMetaData header~%float64 x~%float64 y~%float64 psi~%float64 u~%float64 v~%float64 r~%================================================================================~%MSG: asv_msgs/ShipMetaData~%uint8 id~%string name~%float32 radius~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StateArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StateArray>))
  "Converts a ROS message object to a list"
  (cl:list 'StateArray
    (cl:cons ':states (states msg))
))
