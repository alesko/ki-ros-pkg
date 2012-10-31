
(in-package :asdf)

(defsystem "fastrak-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "StartPublishing" :depends-on ("_package"))
    (:file "_package_StartPublishing" :depends-on ("_package"))
    (:file "GetPose" :depends-on ("_package"))
    (:file "_package_GetPose" :depends-on ("_package"))
    ))
