#!/usr/bin/env roseus

(require :unittest "lib/llib/unittest.l")
(init-unit-test)

(load "package://fetcheus/fetch-interface.l")

;; wait for gazebo
(ros::roseus "test_fetch_moveit")
(ros::advertise "/arm_controller/follow_joint_trajectory/goal" control_msgs::FollowJointTrajectoryActionGoal)
(while (= (ros::get-num-subscribers "/arm_controller/follow_joint_trajectory/goal") 0)
  (unix:sleep 1)
  (format *error-output* "waiting for controller ... arm_controller (~A)~%" (ros::time-now)))

(setq *ri* (instance fetch-interface :init))
(setq *fetch* (fetch))
(send *fetch* :reset-pose)

(deftest test-fetch-moveit ()
  (let (tm-0 tm-1)
    (setq tm-0 (ros::time-now))
    (while (null (send *ri* :angle-vector #f(0 0 0 0 0 0 0 0 0 0))))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (ros::ros-info "time for duration ~A" (send (ros::time- tm-1 tm-0) :to-sec))
    (assert (> (send (ros::time- tm-1 tm-0) :to-sec) 3) "collsion avoidance motion is too fast")

    (setq tm-0 (ros::time-now))
    (while (null (send *ri* :angle-vector (send *fetch* :reset-pose) 2000)))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (ros::ros-info "time for duration ~A" (send (ros::time- tm-1 tm-0) :to-sec))
    (assert (> (send (ros::time- tm-1 tm-0) :to-sec) 3) "collsion avoidance motion is too fast")

    (setq tm-0 (ros::time-now))
    (while (null (send *ri* :angle-vector #f(0 0 0 0 0 0 0 0 0 0))))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (ros::ros-info "time for duration ~A" (send (ros::time- tm-1 tm-0) :to-sec))
    (assert (> (send (ros::time- tm-1 tm-0) :to-sec) 3) "collsion avoidance motion is too fast")

    (setq tm-0 (ros::time-now))
    (while (null (send *ri* :angle-vector (send *fetch* :reset-pose) 2000 :use-torso t)))
    (send *ri* :wait-interpolation)
    (setq tm-1 (ros::time-now))
    (ros::ros-info "time for duration ~A" (send (ros::time- tm-1 tm-0) :to-sec))
    (assert (> (send (ros::time- tm-1 tm-0) :to-sec) 3) "collsion avoidance motion is too fast")

    ))

(run-all-tests)
(exit)
