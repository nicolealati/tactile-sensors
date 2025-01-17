
(cl:in-package :asdf)

(defsystem "sensor_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Forcesensor" :depends-on ("_package_Forcesensor"))
    (:file "_package_Forcesensor" :depends-on ("_package"))
    (:file "Piezosensor" :depends-on ("_package_Piezosensor"))
    (:file "_package_Piezosensor" :depends-on ("_package"))
  ))