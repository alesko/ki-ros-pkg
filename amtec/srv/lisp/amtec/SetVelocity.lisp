; Auto-generated. Do not edit!


(in-package amtec-srv)


;//! \htmlinclude SetVelocity-request.msg.html

(defclass <SetVelocity-request> (ros-message)
  ((velocity_pan
    :reader velocity_pan-val
    :initarg :velocity_pan
    :type float
    :initform 0.0)
   (velocity_tilt
    :reader velocity_tilt-val
    :initarg :velocity_tilt
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <SetVelocity-request>) ostream)
  "Serializes a message object of type '<SetVelocity-request>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'velocity_pan))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'velocity_tilt))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <SetVelocity-request>) istream)
  "Deserializes a message object of type '<SetVelocity-request>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'velocity_pan) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'velocity_tilt) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<SetVelocity-request>)))
  "Returns string type for a service object of type '<SetVelocity-request>"
  "amtec/SetVelocityRequest")
(defmethod md5sum ((type (eql '<SetVelocity-request>)))
  "Returns md5sum for a message object of type '<SetVelocity-request>"
  "2adb301f21428b03215ce423fc357684")
(defmethod message-definition ((type (eql '<SetVelocity-request>)))
  "Returns full string definition for message of type '<SetVelocity-request>"
  (format nil "float64 velocity_pan             # set pan velocity~%float64 velocity_tilt            # set tilt velocity~%~%"))
(defmethod serialization-length ((msg <SetVelocity-request>))
  (+ 0
     8
     8
))
(defmethod ros-message-to-list ((msg <SetVelocity-request>))
  "Converts a ROS message object to a list"
  (list '<SetVelocity-request>
    (cons ':velocity_pan (velocity_pan-val msg))
    (cons ':velocity_tilt (velocity_tilt-val msg))
))
;//! \htmlinclude SetVelocity-response.msg.html

(defclass <SetVelocity-response> (ros-message)
  ()
)
(defmethod serialize ((msg <SetVelocity-response>) ostream)
  "Serializes a message object of type '<SetVelocity-response>"
)
(defmethod deserialize ((msg <SetVelocity-response>) istream)
  "Deserializes a message object of type '<SetVelocity-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<SetVelocity-response>)))
  "Returns string type for a service object of type '<SetVelocity-response>"
  "amtec/SetVelocityResponse")
(defmethod md5sum ((type (eql '<SetVelocity-response>)))
  "Returns md5sum for a message object of type '<SetVelocity-response>"
  "2adb301f21428b03215ce423fc357684")
(defmethod message-definition ((type (eql '<SetVelocity-response>)))
  "Returns full string definition for message of type '<SetVelocity-response>"
  (format nil "~%"))
(defmethod serialization-length ((msg <SetVelocity-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <SetVelocity-response>))
  "Converts a ROS message object to a list"
  (list '<SetVelocity-response>
))
(defmethod service-request-type ((msg (eql 'SetVelocity)))
  '<SetVelocity-request>)
(defmethod service-response-type ((msg (eql 'SetVelocity)))
  '<SetVelocity-response>)
(defmethod ros-datatype ((msg (eql 'SetVelocity)))
  "Returns string type for a service object of type '<SetVelocity>"
  "amtec/SetVelocity")
