; Auto-generated. Do not edit!


(cl:in-package sensor_controller-srv)


;//! \htmlinclude Tared-request.msg.html

(cl:defclass <Tared-request> (roslisp-msg-protocol:ros-message)
  ((tared
    :reader tared
    :initarg :tared
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Tared-request (<Tared-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tared-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tared-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_controller-srv:<Tared-request> is deprecated: use sensor_controller-srv:Tared-request instead.")))

(cl:ensure-generic-function 'tared-val :lambda-list '(m))
(cl:defmethod tared-val ((m <Tared-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_controller-srv:tared-val is deprecated.  Use sensor_controller-srv:tared instead.")
  (tared m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tared-request>) ostream)
  "Serializes a message object of type '<Tared-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tared) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tared-request>) istream)
  "Deserializes a message object of type '<Tared-request>"
    (cl:setf (cl:slot-value msg 'tared) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tared-request>)))
  "Returns string type for a service object of type '<Tared-request>"
  "sensor_controller/TaredRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tared-request)))
  "Returns string type for a service object of type 'Tared-request"
  "sensor_controller/TaredRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tared-request>)))
  "Returns md5sum for a message object of type '<Tared-request>"
  "0341f1ede20bbed8a60c8f16b685bca1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tared-request)))
  "Returns md5sum for a message object of type 'Tared-request"
  "0341f1ede20bbed8a60c8f16b685bca1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tared-request>)))
  "Returns full string definition for message of type '<Tared-request>"
  (cl:format cl:nil "bool tared~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tared-request)))
  "Returns full string definition for message of type 'Tared-request"
  (cl:format cl:nil "bool tared~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tared-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tared-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Tared-request
    (cl:cons ':tared (tared msg))
))
;//! \htmlinclude Tared-response.msg.html

(cl:defclass <Tared-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Tared-response (<Tared-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Tared-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Tared-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_controller-srv:<Tared-response> is deprecated: use sensor_controller-srv:Tared-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Tared-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_controller-srv:success-val is deprecated.  Use sensor_controller-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Tared-response>) ostream)
  "Serializes a message object of type '<Tared-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Tared-response>) istream)
  "Deserializes a message object of type '<Tared-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Tared-response>)))
  "Returns string type for a service object of type '<Tared-response>"
  "sensor_controller/TaredResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tared-response)))
  "Returns string type for a service object of type 'Tared-response"
  "sensor_controller/TaredResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Tared-response>)))
  "Returns md5sum for a message object of type '<Tared-response>"
  "0341f1ede20bbed8a60c8f16b685bca1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Tared-response)))
  "Returns md5sum for a message object of type 'Tared-response"
  "0341f1ede20bbed8a60c8f16b685bca1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Tared-response>)))
  "Returns full string definition for message of type '<Tared-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Tared-response)))
  "Returns full string definition for message of type 'Tared-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Tared-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Tared-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Tared-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Tared)))
  'Tared-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Tared)))
  'Tared-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Tared)))
  "Returns string type for a service object of type '<Tared>"
  "sensor_controller/Tared")