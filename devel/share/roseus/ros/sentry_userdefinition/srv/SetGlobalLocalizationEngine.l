;; Auto-generated. Do not edit!


(when (boundp 'sentry_userdefinition::SetGlobalLocalizationEngine)
  (if (not (find-package "SENTRY_USERDEFINITION"))
    (make-package "SENTRY_USERDEFINITION"))
  (shadow 'SetGlobalLocalizationEngine (find-package "SENTRY_USERDEFINITION")))
(unless (find-package "SENTRY_USERDEFINITION::SETGLOBALLOCALIZATIONENGINE")
  (make-package "SENTRY_USERDEFINITION::SETGLOBALLOCALIZATIONENGINE"))
(unless (find-package "SENTRY_USERDEFINITION::SETGLOBALLOCALIZATIONENGINEREQUEST")
  (make-package "SENTRY_USERDEFINITION::SETGLOBALLOCALIZATIONENGINEREQUEST"))
(unless (find-package "SENTRY_USERDEFINITION::SETGLOBALLOCALIZATIONENGINERESPONSE")
  (make-package "SENTRY_USERDEFINITION::SETGLOBALLOCALIZATIONENGINERESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))




(defclass sentry_userdefinition::SetGlobalLocalizationEngineRequest
  :super ros::object
  :slots (_engine_name ))

(defmethod sentry_userdefinition::SetGlobalLocalizationEngineRequest
  (:init
   (&key
    ((:engine_name __engine_name) (instance std_msgs::String :init))
    )
   (send-super :init)
   (setq _engine_name __engine_name)
   self)
  (:engine_name
   (&rest __engine_name)
   (if (keywordp (car __engine_name))
       (send* _engine_name __engine_name)
     (progn
       (if __engine_name (setq _engine_name (car __engine_name)))
       _engine_name)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/String _engine_name
    (send _engine_name :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/String _engine_name
       (send _engine_name :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/String _engine_name
     (send _engine_name :deserialize buf ptr-) (incf ptr- (send _engine_name :serialization-length))
   ;;
   self)
  )

(defclass sentry_userdefinition::SetGlobalLocalizationEngineResponse
  :super ros::object
  :slots ())

(defmethod sentry_userdefinition::SetGlobalLocalizationEngineResponse
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

(defclass sentry_userdefinition::SetGlobalLocalizationEngine
  :super ros::object
  :slots ())

(setf (get sentry_userdefinition::SetGlobalLocalizationEngine :md5sum-) "f71639c3aeb058fa00da5a20981c23c9")
(setf (get sentry_userdefinition::SetGlobalLocalizationEngine :datatype-) "sentry_userdefinition/SetGlobalLocalizationEngine")
(setf (get sentry_userdefinition::SetGlobalLocalizationEngine :request) sentry_userdefinition::SetGlobalLocalizationEngineRequest)
(setf (get sentry_userdefinition::SetGlobalLocalizationEngine :response) sentry_userdefinition::SetGlobalLocalizationEngineResponse)

(defmethod sentry_userdefinition::SetGlobalLocalizationEngineRequest
  (:response () (instance sentry_userdefinition::SetGlobalLocalizationEngineResponse :init)))

(setf (get sentry_userdefinition::SetGlobalLocalizationEngineRequest :md5sum-) "f71639c3aeb058fa00da5a20981c23c9")
(setf (get sentry_userdefinition::SetGlobalLocalizationEngineRequest :datatype-) "sentry_userdefinition/SetGlobalLocalizationEngineRequest")
(setf (get sentry_userdefinition::SetGlobalLocalizationEngineRequest :definition-)
      "std_msgs/String engine_name

================================================================================
MSG: std_msgs/String
string data
---
")

(setf (get sentry_userdefinition::SetGlobalLocalizationEngineResponse :md5sum-) "f71639c3aeb058fa00da5a20981c23c9")
(setf (get sentry_userdefinition::SetGlobalLocalizationEngineResponse :datatype-) "sentry_userdefinition/SetGlobalLocalizationEngineResponse")
(setf (get sentry_userdefinition::SetGlobalLocalizationEngineResponse :definition-)
      "std_msgs/String engine_name

================================================================================
MSG: std_msgs/String
string data
---
")



(provide :sentry_userdefinition/SetGlobalLocalizationEngine "f71639c3aeb058fa00da5a20981c23c9")


