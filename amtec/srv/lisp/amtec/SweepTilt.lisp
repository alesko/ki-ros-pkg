; Auto-generated. Do not edit!


(in-package amtec-srv)


;//! \htmlinclude SweepTilt-request.msg.html

(defclass <SweepTilt-request> (ros-message)
  ((sweep_amplitude
    :reader sweep_amplitude-val
    :initarg :sweep_amplitude
    :type float
    :initform 0.0)
   (sweep_period
    :reader sweep_period-val
    :initarg :sweep_period
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <SweepTilt-request>) ostream)
  "Serializes a message object of type '<SweepTilt-request>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'sweep_amplitude))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'sweep_period))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <SweepTilt-request>) istream)
  "Deserializes a message object of type '<SweepTilt-request>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'sweep_amplitude) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'sweep_period) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<SweepTilt-request>)))
  "Returns string type for a service object of type '<SweepTilt-request>"
  "amtec/SweepTiltRequest")
(defmethod md5sum ((type (eql '<SweepTilt-request>)))
  "Returns md5sum for a message object of type '<SweepTilt-request>"
  "5ecc0a29ab6ceff25a8c7df356aada72")
(defmethod message-definition ((type (eql '<SweepTilt-request>)))
  "Returns full string definition for message of type '<SweepTilt-request>"
  (format nil "float64 sweep_amplitude         # tilt sweep amplitude in radians ~%float64 sweep_period            # tilt sweep period in ms~%~%"))
(defmethod serialization-length ((msg <SweepTilt-request>))
  (+ 0
     8
     8
))
(defmethod ros-message-to-list ((msg <SweepTilt-request>))
  "Converts a ROS message object to a list"
  (list '<SweepTilt-request>
    (cons ':sweep_amplitude (sweep_amplitude-val msg))
    (cons ':sweep_period (sweep_period-val msg))
))
;//! \htmlinclude SweepTilt-response.msg.html

(defclass <SweepTilt-response> (ros-message)
  ()
)
(defmethod serialize ((msg <SweepTilt-response>) ostream)
  "Serializes a message object of type '<SweepTilt-response>"
)
(defmethod deserialize ((msg <SweepTilt-response>) istream)
  "Deserializes a message object of type '<SweepTilt-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<SweepTilt-response>)))
  "Returns string type for a service object of type '<SweepTilt-response>"
  "amtec/SweepTiltResponse")
(defmethod md5sum ((type (eql '<SweepTilt-response>)))
  "Returns md5sum for a message object of type '<SweepTilt-response>"
  "5ecc0a29ab6ceff25a8c7df356aada72")
(defmethod message-definition ((type (eql '<SweepTilt-response>)))
  "Returns full string definition for message of type '<SweepTilt-response>"
  (format nil "~%"))
(defmethod serialization-length ((msg <SweepTilt-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <SweepTilt-response>))
  "Converts a ROS message object to a list"
  (list '<SweepTilt-response>
))
(defmethod service-request-type ((msg (eql 'SweepTilt)))
  '<SweepTilt-request>)
(defmethod service-response-type ((msg (eql 'SweepTilt)))
  '<SweepTilt-response>)
(defmethod ros-datatype ((msg (eql 'SweepTilt)))
  "Returns string type for a service object of type '<SweepTilt>"
  "amtec/SweepTilt")
