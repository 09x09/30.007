; Auto-generated. Do not edit!


(cl:in-package trolley-msg)


;//! \htmlinclude Wheels.msg.html

(cl:defclass <Wheels> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:float
    :initform 0.0)
   (right
    :reader right
    :initarg :right
    :type cl:float
    :initform 0.0))
)

(cl:defclass Wheels (<Wheels>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Wheels>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Wheels)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name trolley-msg:<Wheels> is deprecated: use trolley-msg:Wheels instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <Wheels>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trolley-msg:left-val is deprecated.  Use trolley-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <Wheels>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader trolley-msg:right-val is deprecated.  Use trolley-msg:right instead.")
  (right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Wheels>) ostream)
  "Serializes a message object of type '<Wheels>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Wheels>) istream)
  "Deserializes a message object of type '<Wheels>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Wheels>)))
  "Returns string type for a message object of type '<Wheels>"
  "trolley/Wheels")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Wheels)))
  "Returns string type for a message object of type 'Wheels"
  "trolley/Wheels")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Wheels>)))
  "Returns md5sum for a message object of type '<Wheels>"
  "50c2436c38cded221d061b57126c4e40")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Wheels)))
  "Returns md5sum for a message object of type 'Wheels"
  "50c2436c38cded221d061b57126c4e40")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Wheels>)))
  "Returns full string definition for message of type '<Wheels>"
  (cl:format cl:nil "#Wheels.msgs~%~%float64 left~%float64 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Wheels)))
  "Returns full string definition for message of type 'Wheels"
  (cl:format cl:nil "#Wheels.msgs~%~%float64 left~%float64 right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Wheels>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Wheels>))
  "Converts a ROS message object to a list"
  (cl:list 'Wheels
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
))
