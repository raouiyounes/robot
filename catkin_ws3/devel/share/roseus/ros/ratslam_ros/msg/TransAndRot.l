;; Auto-generated. Do not edit!


(when (boundp 'ratslam_ros::TransAndRot)
  (if (not (find-package "RATSLAM_ROS"))
    (make-package "RATSLAM_ROS"))
  (shadow 'TransAndRot (find-package "RATSLAM_ROS")))
(unless (find-package "RATSLAM_ROS::TRANSANDROT")
  (make-package "RATSLAM_ROS::TRANSANDROT"))

(in-package "ROS")
;;//! \htmlinclude TransAndRot.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ratslam_ros::TransAndRot
  :super ros::object
  :slots (_header _t _R ))

(defmethod ratslam_ros::TransAndRot
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:t __t) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:R __R) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _t __t)
   (setq _R __R)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:R
   (&optional __R)
   (if __R (setq _R __R)) _R)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _t
    (* 8    (length _t)) 4
    ;; float64[] _R
    (* 8    (length _R)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _t
     (write-long (length _t) s)
     (dotimes (i (length _t))
       (sys::poke (elt _t i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _R
     (write-long (length _R) s)
     (dotimes (i (length _R))
       (sys::poke (elt _R i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _t
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _t (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _t i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _R
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _R (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _R i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get ratslam_ros::TransAndRot :md5sum-) "01e1fb5fba3618d68a00aa3d51d727ce")
(setf (get ratslam_ros::TransAndRot :datatype-) "ratslam_ros/TransAndRot")
(setf (get ratslam_ros::TransAndRot :definition-)
      "Header header
float64[] t
float64[] R

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :ratslam_ros/TransAndRot "01e1fb5fba3618d68a00aa3d51d727ce")


