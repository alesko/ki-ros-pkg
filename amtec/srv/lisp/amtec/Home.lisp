; Auto-generated. Do not edit!


(in-package amtec-srv)


;//! \htmlinclude Home-request.msg.html

(defclass <Home-request> (ros-message)
  ()
)
(defmethod serialize ((msg <Home-request>) ostream)
  "Serializes a message object of type '<Home-request>"
)
(defmethod deserialize ((msg <Home-request>) istream)
  "Deserializes a message object of type '<Home-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<Home-request>)))
  "Returns string type for a service object of type '<Home-request>"
  "amtec/HomeRequest")
(defmethod md5sum ((type (eql '<Home-request>)))
  "Returns md5sum for a message object of type '<Home-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<Home-request>)))
  "Returns full string definition for message of type '<Home-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <Home-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <Home-request>))
  "Converts a ROS message object to a list"
  (list '<Home-request>
))
;//! \htmlinclude Home-response.msg.html

(defclass <Home-response> (ros-message)
  ()
)
(defmethod serialize ((msg <Home-response>) ostream)
  "Serializes a message object of type '<Home-response>"
)
(defmethod deserialize ((msg <Home-response>) istream)
  "Deserializes a message object of type '<Home-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<Home-response>)))
  "Returns string type for a service object of type '<Home-response>"
  "amtec/HomeResponse")
(defmethod md5sum ((type (eql '<Home-response>)))
  "Returns md5sum for a message object of type '<Home-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<Home-response>)))
  "Returns full string definition for message of type '<Home-response>"
  (format nil "~%"))
(defmethod serialization-length ((msg <Home-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <Home-response>))
  "Converts a ROS message object to a list"
  (list '<Home-response>
))
(defmethod service-request-type ((msg (eql 'Home)))
  '<Home-request>)
(defmethod service-response-type ((msg (eql 'Home)))
  '<Home-response>)
(defmethod ros-datatype ((msg (eql 'Home)))
  "Returns string type for a service object of type '<Home>"
  "amtec/Home")
