;; Auto-generated. Do not edit!


(when (boundp 'asv_msgs::StateArray)
  (if (not (find-package "ASV_MSGS"))
    (make-package "ASV_MSGS"))
  (shadow 'StateArray (find-package "ASV_MSGS")))
(unless (find-package "ASV_MSGS::STATEARRAY")
  (make-package "ASV_MSGS::STATEARRAY"))

(in-package "ROS")
;;//! \htmlinclude StateArray.msg.html


(defclass asv_msgs::StateArray
  :super ros::object
  :slots (_states ))

(defmethod asv_msgs::StateArray
  (:init
   (&key
    ((:states __states) (let (r) (dotimes (i 0) (push (instance asv_msgs::State :init) r)) r))
    )
   (send-super :init)
   (setq _states __states)
   self)
  (:states
   (&rest __states)
   (if (keywordp (car __states))
       (send* _states __states)
     (progn
       (if __states (setq _states (car __states)))
       _states)))
  (:serialization-length
   ()
   (+
    ;; asv_msgs/State[] _states
    (apply #'+ (send-all _states :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; asv_msgs/State[] _states
     (write-long (length _states) s)
     (dolist (elem _states)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; asv_msgs/State[] _states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _states (let (r) (dotimes (i n) (push (instance asv_msgs::State :init) r)) r))
     (dolist (elem- _states)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get asv_msgs::StateArray :md5sum-) "05e43d15ff5eab094661932878032f08")
(setf (get asv_msgs::StateArray :datatype-) "asv_msgs/StateArray")
(setf (get asv_msgs::StateArray :definition-)
      "# An array of states
State[] states
================================================================================
MSG: asv_msgs/State
# The full state of a 3DOF surface vessel
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



(provide :asv_msgs/StateArray "05e43d15ff5eab094661932878032f08")


