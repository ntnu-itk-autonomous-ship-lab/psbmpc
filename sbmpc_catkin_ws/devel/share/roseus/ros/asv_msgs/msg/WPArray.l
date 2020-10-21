;; Auto-generated. Do not edit!


(when (boundp 'asv_msgs::WPArray)
  (if (not (find-package "ASV_MSGS"))
    (make-package "ASV_MSGS"))
  (shadow 'WPArray (find-package "ASV_MSGS")))
(unless (find-package "ASV_MSGS::WPARRAY")
  (make-package "ASV_MSGS::WPARRAY"))

(in-package "ROS")
;;//! \htmlinclude WPArray.msg.html


(defclass asv_msgs::WPArray
  :super ros::object
  :slots (_wp_xy ))

(defmethod asv_msgs::WPArray
  (:init
   (&key
    ((:wp_xy __wp_xy) (let (r) (dotimes (i 0) (push (instance asv_msgs::WP :init) r)) r))
    )
   (send-super :init)
   (setq _wp_xy __wp_xy)
   self)
  (:wp_xy
   (&rest __wp_xy)
   (if (keywordp (car __wp_xy))
       (send* _wp_xy __wp_xy)
     (progn
       (if __wp_xy (setq _wp_xy (car __wp_xy)))
       _wp_xy)))
  (:serialization-length
   ()
   (+
    ;; asv_msgs/WP[] _wp_xy
    (apply #'+ (send-all _wp_xy :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; asv_msgs/WP[] _wp_xy
     (write-long (length _wp_xy) s)
     (dolist (elem _wp_xy)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; asv_msgs/WP[] _wp_xy
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _wp_xy (let (r) (dotimes (i n) (push (instance asv_msgs::WP :init) r)) r))
     (dolist (elem- _wp_xy)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get asv_msgs::WPArray :md5sum-) "311aa1759c0b5850ba11ceafcb7a7e61")
(setf (get asv_msgs::WPArray :datatype-) "asv_msgs/WPArray")
(setf (get asv_msgs::WPArray :definition-)
      "# WP array containing (x,y) 
WP[] wp_xy

================================================================================
MSG: asv_msgs/WP
# Waypoint position
# WP: (x,y)
float64 x
float64 y

")



(provide :asv_msgs/WPArray "311aa1759c0b5850ba11ceafcb7a7e61")


