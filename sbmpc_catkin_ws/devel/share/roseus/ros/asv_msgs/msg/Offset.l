;; Auto-generated. Do not edit!


(when (boundp 'asv_msgs::Offset)
  (if (not (find-package "ASV_MSGS"))
    (make-package "ASV_MSGS"))
  (shadow 'Offset (find-package "ASV_MSGS")))
(unless (find-package "ASV_MSGS::OFFSET")
  (make-package "ASV_MSGS::OFFSET"))

(in-package "ROS")
;;//! \htmlinclude Offset.msg.html


(defclass asv_msgs::Offset
  :super ros::object
  :slots (_P_ca _Chi_ca ))

(defmethod asv_msgs::Offset
  (:init
   (&key
    ((:P_ca __P_ca) 0.0)
    ((:Chi_ca __Chi_ca) 0.0)
    )
   (send-super :init)
   (setq _P_ca (float __P_ca))
   (setq _Chi_ca (float __Chi_ca))
   self)
  (:P_ca
   (&optional __P_ca)
   (if __P_ca (setq _P_ca __P_ca)) _P_ca)
  (:Chi_ca
   (&optional __Chi_ca)
   (if __Chi_ca (setq _Chi_ca __Chi_ca)) _Chi_ca)
  (:serialization-length
   ()
   (+
    ;; float64 _P_ca
    8
    ;; float64 _Chi_ca
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _P_ca
       (sys::poke _P_ca (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Chi_ca
       (sys::poke _Chi_ca (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _P_ca
     (setq _P_ca (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Chi_ca
     (setq _Chi_ca (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get asv_msgs::Offset :md5sum-) "67c79bd0079a016b59fb0b6f517f39b2")
(setf (get asv_msgs::Offset :datatype-) "asv_msgs/Offset")
(setf (get asv_msgs::Offset :definition-)
      "float64 P_ca
float64 Chi_ca

")



(provide :asv_msgs/Offset "67c79bd0079a016b59fb0b6f517f39b2")


