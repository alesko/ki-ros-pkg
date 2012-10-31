; Auto-generated. Do not edit!


(in-package amtec-srv)


;//! \htmlinclude TargetAcceleration-request.msg.html

(defclass <TargetAcceleration-request> (ros-message)
  ((acceleration_pan
    :reader acceleration_pan-val
    :initarg :acceleration_pan
    :type float
    :initform 0.0)
   (acceleration_tilt
    :reader acceleration_tilt-val
    :initarg :acceleration_tilt
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <TargetAcceleration-request>) ostream)
  "Serializes a message object of type '<TargetAcceleration-request>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'acceleration_pan))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'acceleration_tilt))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <TargetAcceleration-request>) istream)
  "Deserializes a message object of type '<TargetAcceleration-request>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'acceleration_pan) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'acceleration_tilt) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<TargetAcceleration-request>)))
  "Returns string type for a service object of type '<TargetAcceleration-request>"
  "amtec/TargetAccelerationRequest")
(defmethod md5sum ((type (eql '<TargetAcceleration-request>)))
  "Returns md5sum for a message object of type '<TargetAcceleration-request>"
  "d342e69bfc77177d150eaee5d0f71f06")
(defmethod message-definition ((type (eql '<TargetAcceleration-request>)))
  "Returns full string definition for message of type '<TargetAcceleration-request>"
  (format nil "float64 acceleration_pan             # set target pan velocity~%float64 acceleration_tilt            # set target tilt velocity~%~%"))
(defmethod serialization-length ((msg <TargetAcceleration-request>))
  (+ 0
     8
     8
))
(defmethod ros-message-to-list ((msg <TargetAcceleration-request>))
  "Converts a ROS message object to a list"
  (list '<TargetAcceleration-request>
    (cons ':acceleration_pan (acceleration_pan-val msg))
    (cons ':acceleration_tilt (acceleration_tilt-val msg))
))
;//! \htmlinclude TargetAcceleration-response.msg.html

(defclass <TargetAcceleration-response> (ros-message)
  ()
)
(defmethod serialize ((msg <TargetAcceleration-response>) ostream)
  "Serializes a message object of type '<TargetAcceleration-response>"
)
(defmethod deserialize ((msg <TargetAcceleration-response>) istream)
  "Deserializes a message object of type '<TargetAcceleration-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<TargetAcceleration-response>)))
  "Returns string type for a service object of type '<TargetAcceleration-response>"
  "amtec/TargetAccelerationResponse")
(defmethod md5sum ((type (eql '<TargetAcceleration-response>)))
  "Returns md5sum for a message object of type '<TargetAcceleration-response>"
  "d342e69bfc77177d150eaee5d0f71f06")
(defmethod message-definition ((type (eql '<TargetAcceleration-response>)))
  "Returns full string definition for message of type '<TargetAcceleration-response>"
  (format nil "~%"))
(defmethod serialization-length ((msg <TargetAcceleration-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <TargetAcceleration-response>))
  "Converts a ROS message object to a list"
  (list '<TargetAcceleration-response>
))
(defmethod service-request-type ((msg (eql 'TargetAcceleration)))
  '<TargetAcceleration-request>)
(defmethod service-response-type ((msg (eql 'TargetAcceleration)))
  '<TargetAcceleration-response>)
(defmethod ros-datatype ((msg (eql 'TargetAcceleration)))
  "Returns string type for a service object of type '<TargetAcceleration>"
  "amtec/TargetAcceleration")
