;; Auto-generated. Do not edit!


(when (boundp 'sentry_userdefinition::SetGlobalMap)
  (if (not (find-package "SENTRY_USERDEFINITION"))
    (make-package "SENTRY_USERDEFINITION"))
  (shadow 'SetGlobalMap (find-package "SENTRY_USERDEFINITION")))
(unless (find-package "SENTRY_USERDEFINITION::SETGLOBALMAP")
  (make-package "SENTRY_USERDEFINITION::SETGLOBALMAP"))
(unless (find-package "SENTRY_USERDEFINITION::SETGLOBALMAPREQUEST")
  (make-package "SENTRY_USERDEFINITION::SETGLOBALMAPREQUEST"))
(unless (find-package "SENTRY_USERDEFINITION::SETGLOBALMAPRESPONSE")
  (make-package "SENTRY_USERDEFINITION::SETGLOBALMAPRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))




(defclass sentry_userdefinition::SetGlobalMapRequest
  :super ros::object
  :slots (_global_map ))

(defmethod sentry_userdefinition::SetGlobalMapRequest
  (:init
   (&key
    ((:global_map __global_map) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _global_map __global_map)
   self)
  (:global_map
   (&rest __global_map)
   (if (keywordp (car __global_map))
       (send* _global_map __global_map)
     (progn
       (if __global_map (setq _global_map (car __global_map)))
       _global_map)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/PointCloud2 _global_map
    (send _global_map :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/PointCloud2 _global_map
       (send _global_map :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/PointCloud2 _global_map
     (send _global_map :deserialize buf ptr-) (incf ptr- (send _global_map :serialization-length))
   ;;
   self)
  )

(defclass sentry_userdefinition::SetGlobalMapResponse
  :super ros::object
  :slots ())

(defmethod sentry_userdefinition::SetGlobalMapResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass sentry_userdefinition::SetGlobalMap
  :super ros::object
  :slots ())

(setf (get sentry_userdefinition::SetGlobalMap :md5sum-) "5ed403eb1abbaa077410b645f248da4e")
(setf (get sentry_userdefinition::SetGlobalMap :datatype-) "sentry_userdefinition/SetGlobalMap")
(setf (get sentry_userdefinition::SetGlobalMap :request) sentry_userdefinition::SetGlobalMapRequest)
(setf (get sentry_userdefinition::SetGlobalMap :response) sentry_userdefinition::SetGlobalMapResponse)

(defmethod sentry_userdefinition::SetGlobalMapRequest
  (:response () (instance sentry_userdefinition::SetGlobalMapResponse :init)))

(setf (get sentry_userdefinition::SetGlobalMapRequest :md5sum-) "5ed403eb1abbaa077410b645f248da4e")
(setf (get sentry_userdefinition::SetGlobalMapRequest :datatype-) "sentry_userdefinition/SetGlobalMapRequest")
(setf (get sentry_userdefinition::SetGlobalMapRequest :definition-)
      "sensor_msgs/PointCloud2 global_map

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

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
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
---
")

(setf (get sentry_userdefinition::SetGlobalMapResponse :md5sum-) "5ed403eb1abbaa077410b645f248da4e")
(setf (get sentry_userdefinition::SetGlobalMapResponse :datatype-) "sentry_userdefinition/SetGlobalMapResponse")
(setf (get sentry_userdefinition::SetGlobalMapResponse :definition-)
      "sensor_msgs/PointCloud2 global_map

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

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
string frame_id

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
---
")



(provide :sentry_userdefinition/SetGlobalMap "5ed403eb1abbaa077410b645f248da4e")


