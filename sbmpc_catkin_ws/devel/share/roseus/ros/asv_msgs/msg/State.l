;; Auto-generated. Do not edit!


(when (boundp 'asv_msgs::State)
  (if (not (find-package "ASV_MSGS"))
    (make-package "ASV_MSGS"))
  (shadow 'State (find-package "ASV_MSGS")))
(unless (find-package "ASV_MSGS::STATE")
  (make-package "ASV_MSGS::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html


(defclass asv_msgs::State
  :super ros::object
  :slots (_header _x _y _psi _u _v _r ))

(defmethod asv_msgs::State
  (:init
   (&key
    ((:header __header) (instance asv_msgs::ShipMetaData :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:psi __psi) 0.0)
    ((:u __u) 0.0)
    ((:v __v) 0.0)
    ((:r __r) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _psi (float __psi))
   (setq _u (float __u))
   (setq _v (float __v))
   (setq _r (float __r))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:u
   (&optional __u)
   (if __u (setq _u __u)) _u)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:r
   (&optional __r)
   (if __r (setq _r __r)) _r)
  (:serialization-length
   ()
   (+
    ;; asv_msgs/ShipMetaData _header
    (send _header :serialization-length)
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _psi
    8
    ;; float64 _u
    8
    ;; float64 _v
    8
    ;; float64 _r
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; asv_msgs/ShipMetaData _header
       (send _header :serialize s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _psi
       (sys::poke _psi (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _u
       (sys::poke _u (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _v
       (sys::poke _v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _r
       (sys::poke _r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; asv_msgs/ShipMetaData _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _psi
     (setq _psi (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _u
     (setq _u (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _v
     (setq _v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _r
     (setq _r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get asv_msgs::State :md5sum-) "772f6702c4f085d1def153b9e006f238")
(setf (get asv_msgs::State :datatype-) "asv_msgs/State")
(setf (get asv_msgs::State :definition-)
      "# The full state of a 3DOF surface vessel
# eta: (x,y,psi)
# nu:  (u,v,r)
ShipMetaData header
float64 x
float64 y
float64 psi
float64 u
float64 v
float64 r
================================================================================
MSG: asv_msgs/ShipMetaData
uint8 id
string name
float32 radius
")



(provide :asv_msgs/State "772f6702c4f085d1def153b9e006f238")


