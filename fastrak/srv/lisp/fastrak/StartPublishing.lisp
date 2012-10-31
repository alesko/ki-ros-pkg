; Auto-generated. Do not edit!


(in-package fastrak-srv)


;//! \htmlinclude StartPublishing-request.msg.html

(defclass <StartPublishing-request> (ros-message)
  ((state
    :reader state-val
    :initarg :state
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <StartPublishing-request>) ostream)
  "Serializes a message object of type '<StartPublishing-request>"
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'state) 1 0)) ostream)
)
(defmethod deserialize ((msg <StartPublishing-request>) istream)
  "Deserializes a message object of type '<StartPublishing-request>"
  (setf (slot-value msg 'state) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<StartPublishing-request>)))
  "Returns string type for a service object of type '<StartPublishing-request>"
  "fastrak/StartPublishingRequest")
(defmethod md5sum ((type (eql '<StartPublishing-request>)))
  "Returns md5sum for a message object of type '<StartPublishing-request>"
  "791bb60874e4f25ba44d61029a5c1beb")
(defmethod message-definition ((type (eql '<StartPublishing-request>)))
  "Returns full string definition for message of type '<StartPublishing-request>"
  (format nil "bool state~%~%"))
(defmethod serialization-length ((msg <StartPublishing-request>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <StartPublishing-request>))
  "Converts a ROS message object to a list"
  (list '<StartPublishing-request>
    (cons ':state (state-val msg))
))
;//! \htmlinclude StartPublishing-response.msg.html

(defclass <StartPublishing-response> (ros-message)
  ((state
    :reader state-val
    :initarg :state
    :type boolean
    :initform nil))
)
(defmethod serialize ((msg <StartPublishing-response>) ostream)
  "Serializes a message object of type '<StartPublishing-response>"
    (write-byte (ldb (byte 8 0) (if (slot-value msg 'state) 1 0)) ostream)
)
(defmethod deserialize ((msg <StartPublishing-response>) istream)
  "Deserializes a message object of type '<StartPublishing-response>"
  (setf (slot-value msg 'state) (not (zerop (read-byte istream))))
  msg
)
(defmethod ros-datatype ((msg (eql '<StartPublishing-response>)))
  "Returns string type for a service object of type '<StartPublishing-response>"
  "fastrak/StartPublishingResponse")
(defmethod md5sum ((type (eql '<StartPublishing-response>)))
  "Returns md5sum for a message object of type '<StartPublishing-response>"
  "791bb60874e4f25ba44d61029a5c1beb")
(defmethod message-definition ((type (eql '<StartPublishing-response>)))
  "Returns full string definition for message of type '<StartPublishing-response>"
  (format nil "bool state~%~%"))
(defmethod serialization-length ((msg <StartPublishing-response>))
  (+ 0
     1
))
(defmethod ros-message-to-list ((msg <StartPublishing-response>))
  "Converts a ROS message object to a list"
  (list '<StartPublishing-response>
    (cons ':state (state-val msg))
))
(defmethod service-request-type ((msg (eql 'StartPublishing)))
  '<StartPublishing-request>)
(defmethod service-response-type ((msg (eql 'StartPublishing)))
  '<StartPublishing-response>)
(defmethod ros-datatype ((msg (eql 'StartPublishing)))
  "Returns string type for a service object of type '<StartPublishing>"
  "fastrak/StartPublishing")
