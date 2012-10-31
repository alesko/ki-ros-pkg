
(in-package :asdf)

(defsystem "amtec-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "FastrakPose" :depends-on ("_package"))
    (:file "_package_FastrakPose" :depends-on ("_package"))
    (:file "AmtecState" :depends-on ("_package"))
    (:file "_package_AmtecState" :depends-on ("_package"))
    ))
