;; Auto-generated. Do not edit!


(when (boundp 'sensor_controller::Tare)
  (if (not (find-package "SENSOR_CONTROLLER"))
    (make-package "SENSOR_CONTROLLER"))
  (shadow 'Tare (find-package "SENSOR_CONTROLLER")))
(unless (find-package "SENSOR_CONTROLLER::TARE")
  (make-package "SENSOR_CONTROLLER::TARE"))
(unless (find-package "SENSOR_CONTROLLER::TAREREQUEST")
  (make-package "SENSOR_CONTROLLER::TAREREQUEST"))
(unless (find-package "SENSOR_CONTROLLER::TARERESPONSE")
  (make-package "SENSOR_CONTROLLER::TARERESPONSE"))

(in-package "ROS")





(defclass sensor_controller::TareRequest
  :super ros::object
  :slots (_tare ))

(defmethod sensor_controller::TareRequest
  (:init
   (&key
    ((:tare __tare) nil)
    )
   (send-super :init)
   (setq _tare __tare)
   self)
  (:tare
   (&optional (__tare :null))
   (if (not (eq __tare :null)) (setq _tare __tare)) _tare)
  (:serialization-length
   ()
   (+
    ;; bool _tare
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _tare
       (if _tare (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _tare
     (setq _tare (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass sensor_controller::TareResponse
  :super ros::object
  :slots (_success ))

(defmethod sensor_controller::TareResponse
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

(defclass sensor_controller::Tare
  :super ros::object
  :slots ())

(setf (get sensor_controller::Tare :md5sum-) "87154d23082b63112945f24c169eef56")
(setf (get sensor_controller::Tare :datatype-) "sensor_controller/Tare")
(setf (get sensor_controller::Tare :request) sensor_controller::TareRequest)
(setf (get sensor_controller::Tare :response) sensor_controller::TareResponse)

(defmethod sensor_controller::TareRequest
  (:response () (instance sensor_controller::TareResponse :init)))

(setf (get sensor_controller::TareRequest :md5sum-) "87154d23082b63112945f24c169eef56")
(setf (get sensor_controller::TareRequest :datatype-) "sensor_controller/TareRequest")
(setf (get sensor_controller::TareRequest :definition-)
      "bool tare
---
bool success
")

(setf (get sensor_controller::TareResponse :md5sum-) "87154d23082b63112945f24c169eef56")
(setf (get sensor_controller::TareResponse :datatype-) "sensor_controller/TareResponse")
(setf (get sensor_controller::TareResponse :definition-)
      "bool tare
---
bool success
")



(provide :sensor_controller/Tare "87154d23082b63112945f24c169eef56")


