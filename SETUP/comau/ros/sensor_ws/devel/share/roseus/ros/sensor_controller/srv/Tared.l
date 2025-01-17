;; Auto-generated. Do not edit!


(when (boundp 'sensor_controller::Tared)
  (if (not (find-package "SENSOR_CONTROLLER"))
    (make-package "SENSOR_CONTROLLER"))
  (shadow 'Tared (find-package "SENSOR_CONTROLLER")))
(unless (find-package "SENSOR_CONTROLLER::TARED")
  (make-package "SENSOR_CONTROLLER::TARED"))
(unless (find-package "SENSOR_CONTROLLER::TAREDREQUEST")
  (make-package "SENSOR_CONTROLLER::TAREDREQUEST"))
(unless (find-package "SENSOR_CONTROLLER::TAREDRESPONSE")
  (make-package "SENSOR_CONTROLLER::TAREDRESPONSE"))

(in-package "ROS")





(defclass sensor_controller::TaredRequest
  :super ros::object
  :slots (_tared ))

(defmethod sensor_controller::TaredRequest
  (:init
   (&key
    ((:tared __tared) nil)
    )
   (send-super :init)
   (setq _tared __tared)
   self)
  (:tared
   (&optional (__tared :null))
   (if (not (eq __tared :null)) (setq _tared __tared)) _tared)
  (:serialization-length
   ()
   (+
    ;; bool _tared
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _tared
       (if _tared (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _tared
     (setq _tared (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass sensor_controller::TaredResponse
  :super ros::object
  :slots (_success ))

(defmethod sensor_controller::TaredResponse
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

(defclass sensor_controller::Tared
  :super ros::object
  :slots ())

(setf (get sensor_controller::Tared :md5sum-) "0341f1ede20bbed8a60c8f16b685bca1")
(setf (get sensor_controller::Tared :datatype-) "sensor_controller/Tared")
(setf (get sensor_controller::Tared :request) sensor_controller::TaredRequest)
(setf (get sensor_controller::Tared :response) sensor_controller::TaredResponse)

(defmethod sensor_controller::TaredRequest
  (:response () (instance sensor_controller::TaredResponse :init)))

(setf (get sensor_controller::TaredRequest :md5sum-) "0341f1ede20bbed8a60c8f16b685bca1")
(setf (get sensor_controller::TaredRequest :datatype-) "sensor_controller/TaredRequest")
(setf (get sensor_controller::TaredRequest :definition-)
      "bool tared
---
bool success

")

(setf (get sensor_controller::TaredResponse :md5sum-) "0341f1ede20bbed8a60c8f16b685bca1")
(setf (get sensor_controller::TaredResponse :datatype-) "sensor_controller/TaredResponse")
(setf (get sensor_controller::TaredResponse :definition-)
      "bool tared
---
bool success

")



(provide :sensor_controller/Tared "0341f1ede20bbed8a60c8f16b685bca1")


