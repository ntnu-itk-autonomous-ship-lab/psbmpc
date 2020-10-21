; Auto-generated. Do not edit!


(cl:in-package asv_msgs-msg)


;//! \htmlinclude WPArray.msg.html

(cl:defclass <WPArray> (roslisp-msg-protocol:ros-message)
  ((wp_xy
    :reader wp_xy
    :initarg :wp_xy
    :type (cl:vector asv_msgs-msg:WP)
   :initform (cl:make-array 0 :element-type 'asv_msgs-msg:WP :initial-element (cl:make-instance 'asv_msgs-msg:WP))))
)

(cl:defclass WPArray (<WPArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WPArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WPArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name asv_msgs-msg:<WPArray> is deprecated: use asv_msgs-msg:WPArray instead.")))

(cl:ensure-generic-function 'wp_xy-val :lambda-list '(m))
(cl:defmethod wp_xy-val ((m <WPArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader asv_msgs-msg:wp_xy-val is deprecated.  Use asv_msgs-msg:wp_xy instead.")
  (wp_xy m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WPArray>) ostream)
  "Serializes a message object of type '<WPArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'wp_xy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'wp_xy))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WPArray>) istream)
  "Deserializes a message object of type '<WPArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'wp_xy) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'wp_xy)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'asv_msgs-msg:WP))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WPArray>)))
  "Returns string type for a message object of type '<WPArray>"
  "asv_msgs/WPArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WPArray)))
  "Returns string type for a message object of type 'WPArray"
  "asv_msgs/WPArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WPArray>)))
  "Returns md5sum for a message object of type '<WPArray>"
  "311aa1759c0b5850ba11ceafcb7a7e61")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WPArray)))
  "Returns md5sum for a message object of type 'WPArray"
  "311aa1759c0b5850ba11ceafcb7a7e61")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WPArray>)))
  "Returns full string definition for message of type '<WPArray>"
  (cl:format cl:nil "# WP array containing (x,y) ~%WP[] wp_xy~%~%================================================================================~%MSG: asv_msgs/WP~%# Waypoint position~%# WP: (x,y)~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WPArray)))
  "Returns full string definition for message of type 'WPArray"
  (cl:format cl:nil "# WP array containing (x,y) ~%WP[] wp_xy~%~%================================================================================~%MSG: asv_msgs/WP~%# Waypoint position~%# WP: (x,y)~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WPArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'wp_xy) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WPArray>))
  "Converts a ROS message object to a list"
  (cl:list 'WPArray
    (cl:cons ':wp_xy (wp_xy msg))
))
