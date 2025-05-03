;; Auto-generated. Do not edit!


(when (boundp 'fusion_estimator::FusionEstimatorTest)
  (if (not (find-package "FUSION_ESTIMATOR"))
    (make-package "FUSION_ESTIMATOR"))
  (shadow 'FusionEstimatorTest (find-package "FUSION_ESTIMATOR")))
(unless (find-package "FUSION_ESTIMATOR::FUSIONESTIMATORTEST")
  (make-package "FUSION_ESTIMATOR::FUSIONESTIMATORTEST"))

(in-package "ROS")
;;//! \htmlinclude FusionEstimatorTest.msg.html


(defclass fusion_estimator::FusionEstimatorTest
  :super ros::object
  :slots (_stamp _data_check_a _data_check_b _data_check_c _data_check_d _data_check_e _estimated_xyz _estimated_rpy _feet_based_position _feet_based_velocity _others ))

(defmethod fusion_estimator::FusionEstimatorTest
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:data_check_a __data_check_a) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:data_check_b __data_check_b) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:data_check_c __data_check_c) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:data_check_d __data_check_d) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:data_check_e __data_check_e) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:estimated_xyz __estimated_xyz) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:estimated_rpy __estimated_rpy) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:feet_based_position __feet_based_position) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:feet_based_velocity __feet_based_velocity) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:others __others) (make-array 4 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _data_check_a __data_check_a)
   (setq _data_check_b __data_check_b)
   (setq _data_check_c __data_check_c)
   (setq _data_check_d __data_check_d)
   (setq _data_check_e __data_check_e)
   (setq _estimated_xyz __estimated_xyz)
   (setq _estimated_rpy __estimated_rpy)
   (setq _feet_based_position __feet_based_position)
   (setq _feet_based_velocity __feet_based_velocity)
   (setq _others __others)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:data_check_a
   (&optional __data_check_a)
   (if __data_check_a (setq _data_check_a __data_check_a)) _data_check_a)
  (:data_check_b
   (&optional __data_check_b)
   (if __data_check_b (setq _data_check_b __data_check_b)) _data_check_b)
  (:data_check_c
   (&optional __data_check_c)
   (if __data_check_c (setq _data_check_c __data_check_c)) _data_check_c)
  (:data_check_d
   (&optional __data_check_d)
   (if __data_check_d (setq _data_check_d __data_check_d)) _data_check_d)
  (:data_check_e
   (&optional __data_check_e)
   (if __data_check_e (setq _data_check_e __data_check_e)) _data_check_e)
  (:estimated_xyz
   (&optional __estimated_xyz)
   (if __estimated_xyz (setq _estimated_xyz __estimated_xyz)) _estimated_xyz)
  (:estimated_rpy
   (&optional __estimated_rpy)
   (if __estimated_rpy (setq _estimated_rpy __estimated_rpy)) _estimated_rpy)
  (:feet_based_position
   (&optional __feet_based_position)
   (if __feet_based_position (setq _feet_based_position __feet_based_position)) _feet_based_position)
  (:feet_based_velocity
   (&optional __feet_based_velocity)
   (if __feet_based_velocity (setq _feet_based_velocity __feet_based_velocity)) _feet_based_velocity)
  (:others
   (&optional __others)
   (if __others (setq _others __others)) _others)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; float64[9] _data_check_a
    (* 8    9)
    ;; float64[12] _data_check_b
    (* 8    12)
    ;; float64[12] _data_check_c
    (* 8    12)
    ;; float64[12] _data_check_d
    (* 8    12)
    ;; float64[12] _data_check_e
    (* 8    12)
    ;; float64[9] _estimated_xyz
    (* 8    9)
    ;; float64[9] _estimated_rpy
    (* 8    9)
    ;; float64[12] _feet_based_position
    (* 8    12)
    ;; float64[12] _feet_based_velocity
    (* 8    12)
    ;; float64[4] _others
    (* 8    4)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float64[9] _data_check_a
     (dotimes (i 9)
       (sys::poke (elt _data_check_a i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[12] _data_check_b
     (dotimes (i 12)
       (sys::poke (elt _data_check_b i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[12] _data_check_c
     (dotimes (i 12)
       (sys::poke (elt _data_check_c i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[12] _data_check_d
     (dotimes (i 12)
       (sys::poke (elt _data_check_d i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[12] _data_check_e
     (dotimes (i 12)
       (sys::poke (elt _data_check_e i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[9] _estimated_xyz
     (dotimes (i 9)
       (sys::poke (elt _estimated_xyz i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[9] _estimated_rpy
     (dotimes (i 9)
       (sys::poke (elt _estimated_rpy i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[12] _feet_based_position
     (dotimes (i 12)
       (sys::poke (elt _feet_based_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[12] _feet_based_velocity
     (dotimes (i 12)
       (sys::poke (elt _feet_based_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[4] _others
     (dotimes (i 4)
       (sys::poke (elt _others i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[9] _data_check_a
   (dotimes (i (length _data_check_a))
     (setf (elt _data_check_a i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[12] _data_check_b
   (dotimes (i (length _data_check_b))
     (setf (elt _data_check_b i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[12] _data_check_c
   (dotimes (i (length _data_check_c))
     (setf (elt _data_check_c i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[12] _data_check_d
   (dotimes (i (length _data_check_d))
     (setf (elt _data_check_d i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[12] _data_check_e
   (dotimes (i (length _data_check_e))
     (setf (elt _data_check_e i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[9] _estimated_xyz
   (dotimes (i (length _estimated_xyz))
     (setf (elt _estimated_xyz i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[9] _estimated_rpy
   (dotimes (i (length _estimated_rpy))
     (setf (elt _estimated_rpy i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[12] _feet_based_position
   (dotimes (i (length _feet_based_position))
     (setf (elt _feet_based_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[12] _feet_based_velocity
   (dotimes (i (length _feet_based_velocity))
     (setf (elt _feet_based_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[4] _others
   (dotimes (i (length _others))
     (setf (elt _others i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get fusion_estimator::FusionEstimatorTest :md5sum-) "7e10b193fd0c51a4937b1aeebfa61ac4")
(setf (get fusion_estimator::FusionEstimatorTest :datatype-) "fusion_estimator/FusionEstimatorTest")
(setf (get fusion_estimator::FusionEstimatorTest :definition-)
      "time stamp
float64[9] data_check_a
float64[12] data_check_b
float64[12] data_check_c
float64[12] data_check_d
float64[12] data_check_e
float64[9] estimated_xyz
float64[9] estimated_rpy
float64[12] feet_based_position
float64[12] feet_based_velocity
float64[4] others
")



(provide :fusion_estimator/FusionEstimatorTest "7e10b193fd0c51a4937b1aeebfa61ac4")


