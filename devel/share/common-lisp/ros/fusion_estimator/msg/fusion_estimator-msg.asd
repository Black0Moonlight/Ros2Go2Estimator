
(cl:in-package :asdf)

(defsystem "fusion_estimator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FusionEstimatorTest" :depends-on ("_package_FusionEstimatorTest"))
    (:file "_package_FusionEstimatorTest" :depends-on ("_package"))
  ))