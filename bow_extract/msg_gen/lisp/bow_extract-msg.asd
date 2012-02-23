
(cl:in-package :asdf)

(defsystem "bow_extract-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "cvKeypointVec" :depends-on ("_package_cvKeypointVec"))
    (:file "_package_cvKeypointVec" :depends-on ("_package"))
    (:file "cvKeypoint" :depends-on ("_package_cvKeypoint"))
    (:file "_package_cvKeypoint" :depends-on ("_package"))
  ))