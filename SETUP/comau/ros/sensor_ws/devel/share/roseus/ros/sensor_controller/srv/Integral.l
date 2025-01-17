;; Auto-generated. Do not edit!


(when (boundp 'sensor_controller::Integral)
  (if (not (find-package "SENSOR_CONTROLLER"))
    (make-package "SENSOR_CONTROLLER"))
  (shadow 'Integral (find-package "SENSOR_CONTROLLER")))
(unless (find-package "SENSOR_CONTROLLER::INTEGRAL")
  (make-package "SENSOR_CONTROLLER::INTEGRAL"))
(unless (find-package "SENSOR_CONTROLLER::INTEGRALREQUEST")
  (make-package "SENSOR_CONTROLLER::INTEGRALREQUEST"))
(unless (find-package "SENSOR_CONTROLLER::INTEGRALRESPONSE")
  (make-package "SENSOR_CONTROLLER::INTEGRALRESPONSE"))

(in-package "ROS")





(defclass sensor_controller::IntegralRequest
  :super ros::object
  :slots (_start ))

(defmethod sensor_controller::IntegralRequest
  (:init
   (&key
    ((:start __start) nil)
    )
   (send-super :init)
   (setq _start __start)
   self)
  (:start
   (&optional (__start :null))
   (if (not (eq __start :null)) (setq _start __start)) _start)
  (:serialization-length
   ()
   (+
    ;; bool _start
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _start
       (if _start (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _start
     (setq _start (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass sensor_controller::IntegralResponse
  :super ros::object
  :slots (_success ))

(defmethod sensor_controller::IntegralResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass sensor_controller::Integral
  :super ros::object
  :slots ())

(setf (get sensor_controller::Integral :md5sum-) "bbb7ba84302b6f35af5466a95cd7ac90")
(setf (get sensor_controller::Integral :datatype-) "sensor_controller/Integral")
(setf (get sensor_controller::Integral :request) sensor_controller::IntegralRequest)
(setf (get sensor_controller::Integral :response) sensor_controller::IntegralResponse)

(defmethod sensor_controller::IntegralRequest
  (:response () (instance sensor_controller::IntegralResponse :init)))

(setf (get sensor_controller::IntegralRequest :md5sum-) "bbb7ba84302b6f35af5466a95cd7ac90")
(setf (get sensor_controller::IntegralRequest :datatype-) "sensor_controller/IntegralRequest")
(setf (get sensor_controller::IntegralRequest :definition-)
      "bool start
---
bool success
")

(setf (get sensor_controller::IntegralResponse :md5sum-) "bbb7ba84302b6f35af5466a95cd7ac90")
(setf (get sensor_controller::IntegralResponse :datatype-) "sensor_controller/IntegralResponse")
(setf (get sensor_controller::IntegralResponse :definition-)
      "bool start
---
bool success
")



(provide :sensor_controller/Integral "bbb7ba84302b6f35af5466a95cd7ac90")


