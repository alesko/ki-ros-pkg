; Auto-generated. Do not edit!


(in-package amtec-srv)


;//! \htmlinclude SetPosition-request.msg.html

(defclass <SetPosition-request> (ros-message)
  ((position_pan
    :reader position_pan-val
    :initarg :position_pan
    :type float
    :initform 0.0)
   (position_tilt
    :reader position_tilt-val
    :initarg :position_tilt
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <SetPosition-request>) ostream)
  "Serializes a message object of type '<SetPosition-request>"
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
)
(defmethod deserialize ((msg <SetPosition-request>) istream)
  "Deserializes a message object of type '<SetPosition-request>"
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
  msg
)
(defmethod ros-datatype ((msg (eql '<SetPosition-request>)))
  "Returns string type for a service object of type '<SetPosition-request>"
  "amtec/SetPositionRequest")
(defmethod md5sum ((type (eql '<SetPosition-request>)))
  "Returns md5sum for a message object of type '<SetPosition-request>"
  "cefac8c7f126ab41b3e46de351ef0cea")
(defmethod message-definition ((type (eql '<SetPosition-request>)))
  "Returns full string definition for message of type '<SetPosition-request>"
  (format nil "float64 position_pan             # set pan position~%float64 position_tilt            # set tilt position~%~%"))
(defmethod serialization-length ((msg <SetPosition-request>))
  (+ 0
     8
     8
))
(defmethod ros-message-to-list ((msg <SetPosition-request>))
  "Converts a ROS message object to a list"
  (list '<SetPosition-request>
    (cons ':position_pan (position_pan-val msg))
    (cons ':position_tilt (position_tilt-val msg))
))
;//! \htmlinclude SetPosition-response.msg.html

(defclass <SetPosition-response> (ros-message)
  ()
)
(defmethod serialize ((msg <SetPosition-response>) ostream)
  "Serializes a message object of type '<SetPosition-response>"
)
(defmethod deserialize ((msg <SetPosition-response>) istream)
  "Deserializes a message object of type '<SetPosition-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<SetPosition-response>)))
  "Returns string type for a service object of type '<SetPosition-response>"
  "amtec/SetPositionResponse")
(defmethod md5sum ((type (eql '<SetPosition-response>)))
  "Returns md5sum for a message object of type '<SetPosition-response>"
  "cefac8c7f126ab41b3e46de351ef0cea")
(defmethod message-definition ((type (eql '<SetPosition-response>)))
  "Returns full string definition for message of type '<SetPosition-response>"
  (format nil "~%"))
(defmethod serialization-length ((msg <SetPosition-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <SetPosition-response>))
  "Converts a ROS message object to a list"
  (list '<SetPosition-response>
))
(defmethod service-request-type ((msg (eql 'SetPosition)))
  '<SetPosition-request>)
(defmethod service-response-type ((msg (eql 'SetPosition)))
  '<SetPosition-response>)
(defmethod ros-datatype ((msg (eql 'SetPosition)))
  "Returns string type for a service object of type '<SetPosition>"
  "amtec/SetPosition")
