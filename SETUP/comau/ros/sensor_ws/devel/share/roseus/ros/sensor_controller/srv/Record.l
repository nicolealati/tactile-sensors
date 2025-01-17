;; Auto-generated. Do not edit!


(when (boundp 'sensor_controller::Record)
  (if (not (find-package "SENSOR_CONTROLLER"))
    (make-package "SENSOR_CONTROLLER"))
  (shadow 'Record (find-package "SENSOR_CONTROLLER")))
(unless (find-package "SENSOR_CONTROLLER::RECORD")
  (make-package "SENSOR_CONTROLLER::RECORD"))
(unless (find-package "SENSOR_CONTROLLER::RECORDREQUEST")
  (make-package "SENSOR_CONTROLLER::RECORDREQUEST"))
(unless (find-package "SENSOR_CONTROLLER::RECORDRESPONSE")
  (make-package "SENSOR_CONTROLLER::RECORDRESPONSE"))

(in-package "ROS")





(defclass sensor_controller::RecordRequest
  :super ros::object
  :slots (_start ))

(defmethod sensor_controller::RecordRequest
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

(defclass sensor_controller::RecordResponse
  :super ros::object
  :slots (_success ))

(defmethod sensor_controller::RecordResponse
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

(defclass sensor_controller::Record
  :super ros::object
  :slots ())

(setf (get sensor_controller::Record :md5sum-) "bbb7ba84302b6f35af5466a95cd7ac90")
(setf (get sensor_controller::Record :datatype-) "sensor_controller/Record")
(setf (get sensor_controller::Record :request) sensor_controller::RecordRequest)
(setf (get sensor_controller::Record :response) sensor_controller::RecordResponse)

(defmethod sensor_controller::RecordRequest
  (:response () (instance sensor_controller::RecordResponse :init)))

(setf (get sensor_controller::RecordRequest :md5sum-) "bbb7ba84302b6f35af5466a95cd7ac90")
(setf (get sensor_controller::RecordRequest :datatype-) "sensor_controller/RecordRequest")
(setf (get sensor_controller::RecordRequest :definition-)
      "bool start
---
bool success
")

(setf (get sensor_controller::RecordResponse :md5sum-) "bbb7ba84302b6f35af5466a95cd7ac90")
(setf (get sensor_controller::RecordResponse :datatype-) "sensor_controller/RecordResponse")
(setf (get sensor_controller::RecordResponse :definition-)
      "bool start
---
bool success
")



(provide :sensor_controller/Record "bbb7ba84302b6f35af5466a95cd7ac90")


