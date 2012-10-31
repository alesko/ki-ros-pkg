; Auto-generated. Do not edit!


(in-package amtec-srv)


;//! \htmlinclude Halt-request.msg.html

(defclass <Halt-request> (ros-message)
  ()
)
(defmethod serialize ((msg <Halt-request>) ostream)
  "Serializes a message object of type '<Halt-request>"
)
(defmethod deserialize ((msg <Halt-request>) istream)
  "Deserializes a message object of type '<Halt-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<Halt-request>)))
  "Returns string type for a service object of type '<Halt-request>"
  "amtec/HaltRequest")
(defmethod md5sum ((type (eql '<Halt-request>)))
  "Returns md5sum for a message object of type '<Halt-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<Halt-request>)))
  "Returns full string definition for message of type '<Halt-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <Halt-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <Halt-request>))
  "Converts a ROS message object to a list"
  (list '<Halt-request>
))
;//! \htmlinclude Halt-response.msg.html

(defclass <Halt-response> (ros-message)
  ()
)
(defmethod serialize ((msg <Halt-response>) ostream)
  "Serializes a message object of type '<Halt-response>"
)
(defmethod deserialize ((msg <Halt-response>) istream)
  "Deserializes a message object of type '<Halt-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<Halt-response>)))
  "Returns string type for a service object of type '<Halt-response>"
  "amtec/HaltResponse")
(defmethod md5sum ((type (eql '<Halt-response>)))
  "Returns md5sum for a message object of type '<Halt-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<Halt-response>)))
  "Returns full string definition for message of type '<Halt-response>"
  (format nil "~%"))
(defmethod serialization-length ((msg <Halt-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <Halt-response>))
  "Converts a ROS message object to a list"
  (list '<Halt-response>
))
(defmethod service-request-type ((msg (eql 'Halt)))
  '<Halt-request>)
(defmethod service-response-type ((msg (eql 'Halt)))
  '<Halt-response>)
(defmethod ros-datatype ((msg (eql 'Halt)))
  "Returns string type for a service object of type '<Halt>"
  "amtec/Halt")
