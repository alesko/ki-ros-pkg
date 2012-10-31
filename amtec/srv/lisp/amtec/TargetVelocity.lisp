; Auto-generated. Do not edit!


(in-package amtec-srv)


;//! \htmlinclude TargetVelocity-request.msg.html

(defclass <TargetVelocity-request> (ros-message)
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
(defmethod serialize ((msg <TargetVelocity-request>) ostream)
  "Serializes a message object of type '<TargetVelocity-request>"
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
(defmethod deserialize ((msg <TargetVelocity-request>) istream)
  "Deserializes a message object of type '<TargetVelocity-request>"
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
(defmethod ros-datatype ((msg (eql '<TargetVelocity-request>)))
  "Returns string type for a service object of type '<TargetVelocity-request>"
  "amtec/TargetVelocityRequest")
(defmethod md5sum ((type (eql '<TargetVelocity-request>)))
  "Returns md5sum for a message object of type '<TargetVelocity-request>"
  "2adb301f21428b03215ce423fc357684")
(defmethod message-definition ((type (eql '<TargetVelocity-request>)))
  "Returns full string definition for message of type '<TargetVelocity-request>"
  (format nil "float64 velocity_pan             # set target pan acceleration~%float64 velocity_tilt            # set target tilt acceleration~%~%"))
(defmethod serialization-length ((msg <TargetVelocity-request>))
  (+ 0
     8
     8
))
(defmethod ros-message-to-list ((msg <TargetVelocity-request>))
  "Converts a ROS message object to a list"
  (list '<TargetVelocity-request>
    (cons ':velocity_pan (velocity_pan-val msg))
    (cons ':velocity_tilt (velocity_tilt-val msg))
))
;//! \htmlinclude TargetVelocity-response.msg.html

(defclass <TargetVelocity-response> (ros-message)
  ()
)
(defmethod serialize ((msg <TargetVelocity-response>) ostream)
  "Serializes a message object of type '<TargetVelocity-response>"
)
(defmethod deserialize ((msg <TargetVelocity-response>) istream)
  "Deserializes a message object of type '<TargetVelocity-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<TargetVelocity-response>)))
  "Returns string type for a service object of type '<TargetVelocity-response>"
  "amtec/TargetVelocityResponse")
(defmethod md5sum ((type (eql '<TargetVelocity-response>)))
  "Returns md5sum for a message object of type '<TargetVelocity-response>"
  "2adb301f21428b03215ce423fc357684")
(defmethod message-definition ((type (eql '<TargetVelocity-response>)))
  "Returns full string definition for message of type '<TargetVelocity-response>"
  (format nil "~%"))
(defmethod serialization-length ((msg <TargetVelocity-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <TargetVelocity-response>))
  "Converts a ROS message object to a list"
  (list '<TargetVelocity-response>
))
(defmethod service-request-type ((msg (eql 'TargetVelocity)))
  '<TargetVelocity-request>)
(defmethod service-response-type ((msg (eql 'TargetVelocity)))
  '<TargetVelocity-response>)
(defmethod ros-datatype ((msg (eql 'TargetVelocity)))
  "Returns string type for a service object of type '<TargetVelocity>"
  "amtec/TargetVelocity")
