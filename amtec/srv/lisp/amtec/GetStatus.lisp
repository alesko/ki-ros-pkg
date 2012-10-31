; Auto-generated. Do not edit!


(in-package amtec-srv)


;//! \htmlinclude GetStatus-request.msg.html

(defclass <GetStatus-request> (ros-message)
  ()
)
(defmethod serialize ((msg <GetStatus-request>) ostream)
  "Serializes a message object of type '<GetStatus-request>"
)
(defmethod deserialize ((msg <GetStatus-request>) istream)
  "Deserializes a message object of type '<GetStatus-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<GetStatus-request>)))
  "Returns string type for a service object of type '<GetStatus-request>"
  "amtec/GetStatusRequest")
(defmethod md5sum ((type (eql '<GetStatus-request>)))
  "Returns md5sum for a message object of type '<GetStatus-request>"
  "1faf4b4c83e1e50040afc67afdd423b0")
(defmethod message-definition ((type (eql '<GetStatus-request>)))
  "Returns full string definition for message of type '<GetStatus-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <GetStatus-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <GetStatus-request>))
  "Converts a ROS message object to a list"
  (list '<GetStatus-request>
))
;//! \htmlinclude GetStatus-response.msg.html

(defclass <GetStatus-response> (ros-message)
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
(defmethod serialize ((msg <GetStatus-response>) ostream)
  "Serializes a message object of type '<GetStatus-response>"
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
(defmethod deserialize ((msg <GetStatus-response>) istream)
  "Deserializes a message object of type '<GetStatus-response>"
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
(defmethod ros-datatype ((msg (eql '<GetStatus-response>)))
  "Returns string type for a service object of type '<GetStatus-response>"
  "amtec/GetStatusResponse")
(defmethod md5sum ((type (eql '<GetStatus-response>)))
  "Returns md5sum for a message object of type '<GetStatus-response>"
  "1faf4b4c83e1e50040afc67afdd423b0")
(defmethod message-definition ((type (eql '<GetStatus-response>)))
  "Returns full string definition for message of type '<GetStatus-response>"
  (format nil "float64 position_pan             # pan position~%float64 position_tilt            # tilt position~%float64 velocity_pan             # pan velocity~%float64 velocity_tilt            # tilt velocity~%~%~%"))
(defmethod serialization-length ((msg <GetStatus-response>))
  (+ 0
     8
     8
     8
     8
))
(defmethod ros-message-to-list ((msg <GetStatus-response>))
  "Converts a ROS message object to a list"
  (list '<GetStatus-response>
    (cons ':position_pan (position_pan-val msg))
    (cons ':position_tilt (position_tilt-val msg))
    (cons ':velocity_pan (velocity_pan-val msg))
    (cons ':velocity_tilt (velocity_tilt-val msg))
))
(defmethod service-request-type ((msg (eql 'GetStatus)))
  '<GetStatus-request>)
(defmethod service-response-type ((msg (eql 'GetStatus)))
  '<GetStatus-response>)
(defmethod ros-datatype ((msg (eql 'GetStatus)))
  "Returns string type for a service object of type '<GetStatus>"
  "amtec/GetStatus")
