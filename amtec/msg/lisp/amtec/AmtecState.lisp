; Auto-generated. Do not edit!


(in-package amtec-msg)


;//! \htmlinclude AmtecState.msg.html

(defclass <AmtecState> (ros-message)
  ((header
    :reader header-val
    :initarg :header
    :type roslib-msg:<Header>
    :initform (make-instance 'roslib-msg:<Header>))
   (state
    :reader state-val
    :initarg :state
    :type integer
    :initform 0)
   (position
    :reader position-val
    :initarg :position
    :type float
    :initform 0.0)
   (velocity
    :reader velocity-val
    :initarg :velocity
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <AmtecState>) ostream)
  "Serializes a message object of type '<AmtecState>"
  (serialize (slot-value msg 'header) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'state)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'state)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'state)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'state)) ostream)
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'position))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'velocity))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <AmtecState>) istream)
  "Deserializes a message object of type '<AmtecState>"
  (deserialize (slot-value msg 'header) istream)
  (setf (ldb (byte 8 0) (slot-value msg 'state)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'state)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'state)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'state)) (read-byte istream))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'position) (roslisp-utils:decode-double-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'velocity) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<AmtecState>)))
  "Returns string type for a message object of type '<AmtecState>"
  "amtec/AmtecState")
(defmethod md5sum ((type (eql '<AmtecState>)))
  "Returns md5sum for a message object of type '<AmtecState>"
  "19ace19cde5aea661ada2f628e7afeab")
(defmethod message-definition ((type (eql '<AmtecState>)))
  "Returns full string definition for message of type '<AmtecState>"
  (format nil "#~%# Message for the state of the amtec powercube pan/tilt unit. ~%#~%Header  header         # header~%uint32  state          # module state~%float64 position       # module angle in radians~%float64 velocity       # module velocity in radians/s~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <AmtecState>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4
     8
     8
))
(defmethod ros-message-to-list ((msg <AmtecState>))
  "Converts a ROS message object to a list"
  (list '<AmtecState>
    (cons ':header (header-val msg))
    (cons ':state (state-val msg))
    (cons ':position (position-val msg))
    (cons ':velocity (velocity-val msg))
))
