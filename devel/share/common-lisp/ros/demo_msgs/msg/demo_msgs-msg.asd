
(cl:in-package :asdf)

(defsystem "demo_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "carry" :depends-on ("_package_carry"))
    (:file "_package_carry" :depends-on ("_package"))
  ))