; Auto-generated. Do not edit!


(in-package amtec-srv)


;//! \htmlinclude Reset-request.msg.html

(defclass <Reset-request> (ros-message)
  ()
)
(defmethod serialize ((msg <Reset-request>) ostream)
  "Serializes a message object of type '<Reset-request>"
)
(defmethod deserialize ((msg <Reset-request>) istream)
  "Deserializes a message object of type '<Reset-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<Reset-request>)))
  "Returns string type for a service object of type '<Reset-request>"
  "amtec/ResetRequest")
(defmethod md5sum ((type (eql '<Reset-request>)))
  "Returns md5sum for a message object of type '<Reset-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<Reset-request>)))
  "Returns full string definition for message of type '<Reset-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <Reset-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <Reset-request>))
  "Converts a ROS message object to a list"
  (list '<Reset-request>
))
;//! \htmlinclude Reset-response.msg.html

(defclass <Reset-response> (ros-message)
  ()
)
(defmethod serialize ((msg <Reset-response>) ostream)
  "Serializes a message object of type '<Reset-response>"
)
(defmethod deserialize ((msg <Reset-response>) istream)
  "Deserializes a message object of type '<Reset-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<Reset-response>)))
  "Returns string type for a service object of type '<Reset-response>"
  "amtec/ResetResponse")
(defmethod md5sum ((type (eql '<Reset-response>)))
  "Returns md5sum for a message object of type '<Reset-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<Reset-response>)))
  "Returns full string definition for message of type '<Reset-response>"
  (format nil "~%"))
(defmethod serialization-length ((msg <Reset-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <Reset-response>))
  "Converts a ROS message object to a list"
  (list '<Reset-response>
))
(defmethod service-request-type ((msg (eql 'Reset)))
  '<Reset-request>)
(defmethod service-response-type ((msg (eql 'Reset)))
  '<Reset-response>)
(defmethod ros-datatype ((msg (eql 'Reset)))
  "Returns string type for a service object of type '<Reset>"
  "amtec/Reset")
