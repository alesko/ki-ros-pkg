; Auto-generated. Do not edit!


(in-package amtec-srv)


;//! \htmlinclude SweepPan-request.msg.html

(defclass <SweepPan-request> (ros-message)
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
(defmethod serialize ((msg <SweepPan-request>) ostream)
  "Serializes a message object of type '<SweepPan-request>"
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
(defmethod deserialize ((msg <SweepPan-request>) istream)
  "Deserializes a message object of type '<SweepPan-request>"
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
(defmethod ros-datatype ((msg (eql '<SweepPan-request>)))
  "Returns string type for a service object of type '<SweepPan-request>"
  "amtec/SweepPanRequest")
(defmethod md5sum ((type (eql '<SweepPan-request>)))
  "Returns md5sum for a message object of type '<SweepPan-request>"
  "5ecc0a29ab6ceff25a8c7df356aada72")
(defmethod message-definition ((type (eql '<SweepPan-request>)))
  "Returns full string definition for message of type '<SweepPan-request>"
  (format nil "float64 sweep_amplitude         # pan sweep amplitude in radians ~%float64 sweep_period            # pan sweep period in ms~%~%"))
(defmethod serialization-length ((msg <SweepPan-request>))
  (+ 0
     8
     8
))
(defmethod ros-message-to-list ((msg <SweepPan-request>))
  "Converts a ROS message object to a list"
  (list '<SweepPan-request>
    (cons ':sweep_amplitude (sweep_amplitude-val msg))
    (cons ':sweep_period (sweep_period-val msg))
))
;//! \htmlinclude SweepPan-response.msg.html

(defclass <SweepPan-response> (ros-message)
  ()
)
(defmethod serialize ((msg <SweepPan-response>) ostream)
  "Serializes a message object of type '<SweepPan-response>"
)
(defmethod deserialize ((msg <SweepPan-response>) istream)
  "Deserializes a message object of type '<SweepPan-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<SweepPan-response>)))
  "Returns string type for a service object of type '<SweepPan-response>"
  "amtec/SweepPanResponse")
(defmethod md5sum ((type (eql '<SweepPan-response>)))
  "Returns md5sum for a message object of type '<SweepPan-response>"
  "5ecc0a29ab6ceff25a8c7df356aada72")
(defmethod message-definition ((type (eql '<SweepPan-response>)))
  "Returns full string definition for message of type '<SweepPan-response>"
  (format nil "~%"))
(defmethod serialization-length ((msg <SweepPan-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <SweepPan-response>))
  "Converts a ROS message object to a list"
  (list '<SweepPan-response>
))
(defmethod service-request-type ((msg (eql 'SweepPan)))
  '<SweepPan-request>)
(defmethod service-response-type ((msg (eql 'SweepPan)))
  '<SweepPan-response>)
(defmethod ros-datatype ((msg (eql 'SweepPan)))
  "Returns string type for a service object of type '<SweepPan>"
  "amtec/SweepPan")
