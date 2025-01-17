; Auto-generated. Do not edit!


(cl:in-package sensor_controller-srv)


;//! \htmlinclude Tare-request.msg.html

(cl:defclass <Tare-request> (roslisp-msg-protocol:ros-message)
  ((tare
    :reader tare
    :initarg :tare
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Tare-request (<Tare-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tare-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tare-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_controller-srv:<Tare-request> is deprecated: use sensor_controller-srv:Tare-request instead.")))

(cl:ensure-generic-function 'tare-val :lambda-list '(m))
(cl:defmethod tare-val ((m <Tare-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_controller-srv:tare-val is deprecated.  Use sensor_controller-srv:tare instead.")
  (tare m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tare-request>) ostream)
  "Serializes a message object of type '<Tare-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tare) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tare-request>) istream)
  "Deserializes a message object of type '<Tare-request>"
    (cl:setf (cl:slot-value msg 'tare) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tare-request>)))
  "Returns string type for a service object of type '<Tare-request>"
  "sensor_controller/TareRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tare-request)))
  "Returns string type for a service object of type 'Tare-request"
  "sensor_controller/TareRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tare-request>)))
  "Returns md5sum for a message object of type '<Tare-request>"
  "87154d23082b63112945f24c169eef56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tare-request)))
  "Returns md5sum for a message object of type 'Tare-request"
  "87154d23082b63112945f24c169eef56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tare-request>)))
  "Returns full string definition for message of type '<Tare-request>"
  (cl:format cl:nil "bool tare~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tare-request)))
  "Returns full string definition for message of type 'Tare-request"
  (cl:format cl:nil "bool tare~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tare-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tare-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Tare-request
    (cl:cons ':tare (tare msg))
))
;//! \htmlinclude Tare-response.msg.html

(cl:defclass <Tare-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Tare-response (<Tare-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tare-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tare-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_controller-srv:<Tare-response> is deprecated: use sensor_controller-srv:Tare-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Tare-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_controller-srv:success-val is deprecated.  Use sensor_controller-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tare-response>) ostream)
  "Serializes a message object of type '<Tare-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tare-response>) istream)
  "Deserializes a message object of type '<Tare-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tare-response>)))
  "Returns string type for a service object of type '<Tare-response>"
  "sensor_controller/TareResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tare-response)))
  "Returns string type for a service object of type 'Tare-response"
  "sensor_controller/TareResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tare-response>)))
  "Returns md5sum for a message object of type '<Tare-response>"
  "87154d23082b63112945f24c169eef56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tare-response)))
  "Returns md5sum for a message object of type 'Tare-response"
  "87154d23082b63112945f24c169eef56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tare-response>)))
  "Returns full string definition for message of type '<Tare-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tare-response)))
  "Returns full string definition for message of type 'Tare-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tare-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tare-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Tare-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Tare)))
  'Tare-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Tare)))
  'Tare-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tare)))
  "Returns string type for a service object of type '<Tare>"
  "sensor_controller/Tare")