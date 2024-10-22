
(cl:in-package :asdf)

(defsystem "sentry_userdefinition-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LocalizationInfo" :depends-on ("_package_LocalizationInfo"))
    (:file "_package_LocalizationInfo" :depends-on ("_package"))
    (:file "ScanMatchingStatus" :depends-on ("_package_ScanMatchingStatus"))
    (:file "_package_ScanMatchingStatus" :depends-on ("_package"))
  ))