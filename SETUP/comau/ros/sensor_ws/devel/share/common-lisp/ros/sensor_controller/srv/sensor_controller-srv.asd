
(cl:in-package :asdf)

(defsystem "sensor_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Integral" :depends-on ("_package_Integral"))
    (:file "_package_Integral" :depends-on ("_package"))
    (:file "Record" :depends-on ("_package_Record"))
    (:file "_package_Record" :depends-on ("_package"))
    (:file "Tare" :depends-on ("_package_Tare"))
    (:file "_package_Tare" :depends-on ("_package"))
    (:file "Tared" :depends-on ("_package_Tared"))
    (:file "_package_Tared" :depends-on ("_package"))
  ))