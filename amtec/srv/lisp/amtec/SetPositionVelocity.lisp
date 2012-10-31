; Auto-generated. Do not edit!


(in-package amtec-srv)


;//! \htmlinclude SetPositionVelocity-request.msg.html

(defclass <SetPositionVelocity-request> (ros-message)
  ((position_pan
    :reader position_pan-val
    :initarg :position_pan
    :type float
    :initform 0.0)
   (position_tilt
    :reader position_tilt-val
    :initarg :position_tilt
    :type float
    :initform 0.0)
   (velocity_pan
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
(defmethod serialize ((msg <SetPositionVelocity-request>) ostream)
  "Serializes a message object of type '<SetPositionVelocity-request>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'position_pan))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'position_tilt))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
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
(defmethod deserialize ((msg <SetPositionVelocity-request>) istream)
  "Deserializes a message object of type '<SetPositionVelocity-request>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'position_pan) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'position_tilt) (roslisp-utils:decode-double-float-bits bits)))
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
(defmethod ros-datatype ((msg (eql '<SetPositionVelocity-request>)))
  "Returns string type for a service object of type '<SetPositionVelocity-request>"
  "amtec/SetPositionVelocityRequest")
(defmethod md5sum ((type (eql '<SetPositionVelocity-request>)))
  "Returns md5sum for a message object of type '<SetPositionVelocity-request>"
  "1faf4b4c83e1e50040afc67afdd423b0")
(defmethod message-definition ((type (eql '<SetPositionVelocity-request>)))
  "Returns full string definition for message of type '<SetPositionVelocity-request>"
  (format nil "float64 position_pan             # set pan position~%float64 position_tilt            # set tilt position~%float64 velocity_pan             # set pan velocity~%float64 velocity_tilt            # set tilt velocity~%~%"))
(defmethod serialization-length ((msg <SetPositionVelocity-request>))
  (+ 0
     8
     8
     8
     8
))
(defmethod ros-message-to-list ((msg <SetPositionVelocity-request>))
  "Converts a ROS message object to a list"
  (list '<SetPositionVelocity-request>
    (cons ':position_pan (position_pan-val msg))
    (cons ':position_tilt (position_tilt-val msg))
    (cons ':velocity_pan (velocity_pan-val msg))
    (cons ':velocity_tilt (velocity_tilt-val msg))
))
;//! \htmlinclude SetPositionVelocity-response.msg.html

(defclass <SetPositionVelocity-response> (ros-message)
  ()
)
(defmethod serialize ((msg <SetPositionVelocity-response>) ostream)
  "Serializes a message object of type '<SetPositionVelocity-response>"
)
(defmethod deserialize ((msg <SetPositionVelocity-response>) istream)
  "Deserializes a message object of type '<SetPositionVelocity-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<SetPositionVelocity-response>)))
  "Returns string type for a service object of type '<SetPositionVelocity-response>"
  "amtec/SetPositionVelocityResponse")
(defmethod md5sum ((type (eql '<SetPositionVelocity-response>)))
  "Returns md5sum for a message object of type '<SetPositionVelocity-response>"
  "1faf4b4c83e1e50040afc67afdd423b0")
(defmethod message-definition ((type (eql '<SetPositionVelocity-response>)))
  "Returns full string definition for message of type '<SetPositionVelocity-response>"
  (format nil "~%"))
(defmethod serialization-length ((msg <SetPositionVelocity-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <SetPositionVelocity-response>))
  "Converts a ROS message object to a list"
  (list '<SetPositionVelocity-response>
))
(defmethod service-request-type ((msg (eql 'SetPositionVelocity)))
  '<SetPositionVelocity-request>)
(defmethod service-response-type ((msg (eql 'SetPositionVelocity)))
  '<SetPositionVelocity-response>)
(defmethod ros-datatype ((msg (eql 'SetPositionVelocity)))
  "Returns string type for a service object of type '<SetPositionVelocity>"
  "amtec/SetPositionVelocity")
