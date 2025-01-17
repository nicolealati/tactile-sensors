; Auto-generated. Do not edit!


(cl:in-package sensor_controller-srv)


;//! \htmlinclude Integral-request.msg.html

(cl:defclass <Integral-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Integral-request (<Integral-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Integral-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Integral-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_controller-srv:<Integral-request> is deprecated: use sensor_controller-srv:Integral-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <Integral-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_controller-srv:start-val is deprecated.  Use sensor_controller-srv:start instead.")
  (start m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Integral-request>) ostream)
  "Serializes a message object of type '<Integral-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Integral-request>) istream)
  "Deserializes a message object of type '<Integral-request>"
    (cl:setf (cl:slot-value msg 'start) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Integral-request>)))
  "Returns string type for a service object of type '<Integral-request>"
  "sensor_controller/IntegralRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Integral-request)))
  "Returns string type for a service object of type 'Integral-request"
  "sensor_controller/IntegralRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Integral-request>)))
  "Returns md5sum for a message object of type '<Integral-request>"
  "bbb7ba84302b6f35af5466a95cd7ac90")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Integral-request)))
  "Returns md5sum for a message object of type 'Integral-request"
  "bbb7ba84302b6f35af5466a95cd7ac90")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Integral-request>)))
  "Returns full string definition for message of type '<Integral-request>"
  (cl:format cl:nil "bool start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Integral-request)))
  "Returns full string definition for message of type 'Integral-request"
  (cl:format cl:nil "bool start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Integral-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Integral-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Integral-request
    (cl:cons ':start (start msg))
))
;//! \htmlinclude Integral-response.msg.html

(cl:defclass <Integral-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Integral-response (<Integral-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Integral-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Integral-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_controller-srv:<Integral-response> is deprecated: use sensor_controller-srv:Integral-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Integral-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_controller-srv:success-val is deprecated.  Use sensor_controller-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Integral-response>) ostream)
  "Serializes a message object of type '<Integral-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Integral-response>) istream)
  "Deserializes a message object of type '<Integral-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Integral-response>)))
  "Returns string type for a service object of type '<Integral-response>"
  "sensor_controller/IntegralResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Integral-response)))
  "Returns string type for a service object of type 'Integral-response"
  "sensor_controller/IntegralResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Integral-response>)))
  "Returns md5sum for a message object of type '<Integral-response>"
  "bbb7ba84302b6f35af5466a95cd7ac90")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Integral-response)))
  "Returns md5sum for a message object of type 'Integral-response"
  "bbb7ba84302b6f35af5466a95cd7ac90")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Integral-response>)))
  "Returns full string definition for message of type '<Integral-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Integral-response)))
  "Returns full string definition for message of type 'Integral-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Integral-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Integral-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Integral-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Integral)))
  'Integral-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Integral)))
  'Integral-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Integral)))
  "Returns string type for a service object of type '<Integral>"
  "sensor_controller/Integral")