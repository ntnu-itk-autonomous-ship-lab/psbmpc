;; Auto-generated. Do not edit!


(when (boundp 'asv_msgs::ShipMetaData)
  (if (not (find-package "ASV_MSGS"))
    (make-package "ASV_MSGS"))
  (shadow 'ShipMetaData (find-package "ASV_MSGS")))
(unless (find-package "ASV_MSGS::SHIPMETADATA")
  (make-package "ASV_MSGS::SHIPMETADATA"))

(in-package "ROS")
;;//! \htmlinclude ShipMetaData.msg.html


(defclass asv_msgs::ShipMetaData
  :super ros::object
  :slots (_id _name _radius ))

(defmethod asv_msgs::ShipMetaData
  (:init
   (&key
    ((:id __id) 0)
    ((:name __name) "")
    ((:radius __radius) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _name (string __name))
   (setq _radius (float __radius))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ;; string _name
    4 (length _name)
    ;; float32 _radius
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float32 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _radius
     (setq _radius (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get asv_msgs::ShipMetaData :md5sum-) "a664eef656fd4f22be60ea17b5e54ee3")
(setf (get asv_msgs::ShipMetaData :datatype-) "asv_msgs/ShipMetaData")
(setf (get asv_msgs::ShipMetaData :definition-)
      "uint8 id
string name
float32 radius
")



(provide :asv_msgs/ShipMetaData "a664eef656fd4f22be60ea17b5e54ee3")


