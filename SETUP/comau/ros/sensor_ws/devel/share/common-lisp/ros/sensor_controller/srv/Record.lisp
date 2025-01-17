; Auto-generated. Do not edit!


(cl:in-package sensor_controller-srv)


;//! \htmlinclude Record-request.msg.html

(cl:defclass <Record-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Record-request (<Record-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Record-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Record-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_controller-srv:<Record-request> is deprecated: use sensor_controller-srv:Record-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <Record-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_controller-srv:start-val is deprecated.  Use sensor_controller-srv:start instead.")
  (start m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Record-request>) ostream)
  "Serializes a message object of type '<Record-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Record-request>) istream)
  "Deserializes a message object of type '<Record-request>"
    (cl:setf (cl:slot-value msg 'start) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Record-request>)))
  "Returns string type for a service object of type '<Record-request>"
  "sensor_controller/RecordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Record-request)))
  "Returns string type for a service object of type 'Record-request"
  "sensor_controller/RecordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Record-request>)))
  "Returns md5sum for a message object of type '<Record-request>"
  "bbb7ba84302b6f35af5466a95cd7ac90")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Record-request)))
  "Returns md5sum for a message object of type 'Record-request"
  "bbb7ba84302b6f35af5466a95cd7ac90")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Record-request>)))
  "Returns full string definition for message of type '<Record-request>"
  (cl:format cl:nil "bool start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Record-request)))
  "Returns full string definition for message of type 'Record-request"
  (cl:format cl:nil "bool start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Record-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Record-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Record-request
    (cl:cons ':start (start msg))
))
;//! \htmlinclude Record-response.msg.html

(cl:defclass <Record-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Record-response (<Record-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Record-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Record-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensor_controller-srv:<Record-response> is deprecated: use sensor_controller-srv:Record-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Record-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensor_controller-srv:success-val is deprecated.  Use sensor_controller-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Record-response>) ostream)
  "Serializes a message object of type '<Record-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Record-response>) istream)
  "Deserializes a message object of type '<Record-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Record-response>)))
  "Returns string type for a service object of type '<Record-response>"
  "sensor_controller/RecordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Record-response)))
  "Returns string type for a service object of type 'Record-response"
  "sensor_controller/RecordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Record-response>)))
  "Returns md5sum for a message object of type '<Record-response>"
  "bbb7ba84302b6f35af5466a95cd7ac90")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Record-response)))
  "Returns md5sum for a message object of type 'Record-response"
  "bbb7ba84302b6f35af5466a95cd7ac90")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Record-response>)))
  "Returns full string definition for message of type '<Record-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Record-response)))
  "Returns full string definition for message of type 'Record-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Record-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Record-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Record-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Record)))
  'Record-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Record)))
  'Record-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Record)))
  "Returns string type for a service object of type '<Record>"
  "sensor_controller/Record")