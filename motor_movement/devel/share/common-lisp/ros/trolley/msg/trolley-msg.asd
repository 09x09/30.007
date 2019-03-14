
(cl:in-package :asdf)

(defsystem "trolley-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Wheels" :depends-on ("_package_Wheels"))
    (:file "_package_Wheels" :depends-on ("_package"))
  ))