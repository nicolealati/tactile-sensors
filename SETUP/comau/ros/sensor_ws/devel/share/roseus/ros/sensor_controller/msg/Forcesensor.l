;; Auto-generated. Do not edit!


(when (boundp 'sensor_controller::Forcesensor)
  (if (not (find-package "SENSOR_CONTROLLER"))
    (make-package "SENSOR_CONTROLLER"))
  (shadow 'Forcesensor (find-package "SENSOR_CONTROLLER")))
(unless (find-package "SENSOR_CONTROLLER::FORCESENSOR")
  (make-package "SENSOR_CONTROLLER::FORCESENSOR"))

(in-package "ROS")
;;//! \htmlinclude Forcesensor.msg.html


(defclass sensor_controller::Forcesensor
  :super ros::object
  :slots (_thumb _index _middle _ring _little ))

(defmethod sensor_controller::Forcesensor
  (:init
   (&key
    ((:thumb __thumb) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:index __index) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:middle __middle) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:ring __ring) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:little __little) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _thumb __thumb)
   (setq _index __index)
   (setq _middle __middle)
   (setq _ring __ring)
   (setq _little __little)
   self)
  (:thumb
   (&optional __thumb)
   (if __thumb (setq _thumb __thumb)) _thumb)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:middle
   (&optional __middle)
   (if __middle (setq _middle __middle)) _middle)
  (:ring
   (&optional __ring)
   (if __ring (setq _ring __ring)) _ring)
  (:little
   (&optional __little)
   (if __little (setq _little __little)) _little)
  (:serialization-length
   ()
   (+
    ;; float64[] _thumb
    (* 8    (length _thumb)) 4
    ;; float64[] _index
    (* 8    (length _index)) 4
    ;; float64[] _middle
    (* 8    (length _middle)) 4
    ;; float64[] _ring
    (* 8    (length _ring)) 4
    ;; float64[] _little
    (* 8    (length _little)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _thumb
     (write-long (length _thumb) s)
     (dotimes (i (length _thumb))
       (sys::poke (elt _thumb i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _index
     (write-long (length _index) s)
     (dotimes (i (length _index))
       (sys::poke (elt _index i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _middle
     (write-long (length _middle) s)
     (dotimes (i (length _middle))
       (sys::poke (elt _middle i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _ring
     (write-long (length _ring) s)
     (dotimes (i (length _ring))
       (sys::poke (elt _ring i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _little
     (write-long (length _little) s)
     (dotimes (i (length _little))
       (sys::poke (elt _little i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _thumb
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _thumb (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _thumb i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _index
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _index (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _index i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _middle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _middle (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _middle i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _ring
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ring (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _ring i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _little
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _little (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _little i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get sensor_controller::Forcesensor :md5sum-) "e47c7f48cddbe1ac7a44256833731770")
(setf (get sensor_controller::Forcesensor :datatype-) "sensor_controller/Forcesensor")
(setf (get sensor_controller::Forcesensor :definition-)
      "float64[] thumb
float64[] index
float64[] middle
float64[] ring
float64[] little
")



(provide :sensor_controller/Forcesensor "e47c7f48cddbe1ac7a44256833731770")


