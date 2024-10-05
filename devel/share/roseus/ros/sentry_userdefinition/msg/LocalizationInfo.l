;; Auto-generated. Do not edit!


(when (boundp 'sentry_userdefinition::LocalizationInfo)
  (if (not (find-package "SENTRY_USERDEFINITION"))
    (make-package "SENTRY_USERDEFINITION"))
  (shadow 'LocalizationInfo (find-package "SENTRY_USERDEFINITION")))
(unless (find-package "SENTRY_USERDEFINITION::LOCALIZATIONINFO")
  (make-package "SENTRY_USERDEFINITION::LOCALIZATIONINFO"))

(in-package "ROS")
;;//! \htmlinclude LocalizationInfo.msg.html


(defclass sentry_userdefinition::LocalizationInfo
  :super ros::object
  :slots (_if_relocation _point_cloud_quantity _tranDist _angleDist _angle_apeed _score _if_match_success ))

(defmethod sentry_userdefinition::LocalizationInfo
  (:init
   (&key
    ((:if_relocation __if_relocation) nil)
    ((:point_cloud_quantity __point_cloud_quantity) 0.0)
    ((:tranDist __tranDist) 0.0)
    ((:angleDist __angleDist) 0.0)
    ((:angle_apeed __angle_apeed) 0.0)
    ((:score __score) 0.0)
    ((:if_match_success __if_match_success) nil)
    )
   (send-super :init)
   (setq _if_relocation __if_relocation)
   (setq _point_cloud_quantity (float __point_cloud_quantity))
   (setq _tranDist (float __tranDist))
   (setq _angleDist (float __angleDist))
   (setq _angle_apeed (float __angle_apeed))
   (setq _score (float __score))
   (setq _if_match_success __if_match_success)
   self)
  (:if_relocation
   (&optional (__if_relocation :null))
   (if (not (eq __if_relocation :null)) (setq _if_relocation __if_relocation)) _if_relocation)
  (:point_cloud_quantity
   (&optional __point_cloud_quantity)
   (if __point_cloud_quantity (setq _point_cloud_quantity __point_cloud_quantity)) _point_cloud_quantity)
  (:tranDist
   (&optional __tranDist)
   (if __tranDist (setq _tranDist __tranDist)) _tranDist)
  (:angleDist
   (&optional __angleDist)
   (if __angleDist (setq _angleDist __angleDist)) _angleDist)
  (:angle_apeed
   (&optional __angle_apeed)
   (if __angle_apeed (setq _angle_apeed __angle_apeed)) _angle_apeed)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:if_match_success
   (&optional (__if_match_success :null))
   (if (not (eq __if_match_success :null)) (setq _if_match_success __if_match_success)) _if_match_success)
  (:serialization-length
   ()
   (+
    ;; bool _if_relocation
    1
    ;; float64 _point_cloud_quantity
    8
    ;; float64 _tranDist
    8
    ;; float64 _angleDist
    8
    ;; float64 _angle_apeed
    8
    ;; float64 _score
    8
    ;; bool _if_match_success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _if_relocation
       (if _if_relocation (write-byte -1 s) (write-byte 0 s))
     ;; float64 _point_cloud_quantity
       (sys::poke _point_cloud_quantity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _tranDist
       (sys::poke _tranDist (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angleDist
       (sys::poke _angleDist (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angle_apeed
       (sys::poke _angle_apeed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _score
       (sys::poke _score (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _if_match_success
       (if _if_match_success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _if_relocation
     (setq _if_relocation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _point_cloud_quantity
     (setq _point_cloud_quantity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _tranDist
     (setq _tranDist (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angleDist
     (setq _angleDist (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angle_apeed
     (setq _angle_apeed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _score
     (setq _score (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _if_match_success
     (setq _if_match_success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get sentry_userdefinition::LocalizationInfo :md5sum-) "4480d6179e334a6455057a4ec084d3ef")
(setf (get sentry_userdefinition::LocalizationInfo :datatype-) "sentry_userdefinition/LocalizationInfo")
(setf (get sentry_userdefinition::LocalizationInfo :definition-)
      "bool if_relocation
float64 point_cloud_quantity
float64 tranDist
float64 angleDist
float64 angle_apeed
float64 score
bool if_match_success

")



(provide :sentry_userdefinition/LocalizationInfo "4480d6179e334a6455057a4ec084d3ef")


