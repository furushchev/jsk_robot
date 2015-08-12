#!/usr/bin/env roseus
;; pr2-tweet-log.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "pr2_tweet_log")
(ros::load-ros-manifest "jsk_demo_common")

(require :mongo-client "package://roseus_mongo/euslisp/mongo-client.l")
(defparameter *mongo-database* "jsk_robot_lifelog")
(defparameter *json-parse-key-function* #'string->keyword)

(defclass log-tweet-worker
  :super propertied-object
  :slots (name check-cb tweet-cb check-result last-check-date))

(defmethod log-tweet-worker
  (:init (&key ((:name n)) ((:check-cb ccb)) ((:tweet-cb tcb)))
   (setq name n)
   (setq check-cb ccb)
   (setq tweet-cb tcb))
  (:name (&optional n)
   (if n (setq name n) name))
  (:last-check (&optional d)
   (if d (setq last-check-date d) last-check-date))
  (:check (&key (timeout 300))
   (setq last-check-date (now))
   (setq check-result (funcall check-cb self)))
  (:tweet ()
   (let ((str (funcall tweet-cb self check-result)))
     (unless (stringp str)
       (ros::ros-error "[~A] expected ~A is not string." name str)
       (return-from :tweet nil))
     str))
  ) ;; end of defmethod


(defclass log-tweet-manager
  :super propertied-object
  :slots (workers))

(defmethod log-tweet-manager
  (:init ()
   (setq workers (make-hash-table)))
  (:add-worker (w)
   (let ((work-key (intern (send w :name) *keyword-package*)))
     (if (gethash work-key workers)
         (error "duplicate worker ~A" (gethash work-key workers))
         (setf (gethash work-key workers) w))))
  (:remove-worker (name)
   (let ((work-key (if (keywordp name) name (intern name *keyword-package*))))
     (remhash work-key workers)))
  (:start ()
    (do-until-key
        (dolist (k (send workers :list-keys))
          (let ((w (gethash k workers)) ret)
            (when (send w :check)
              (setq ret (send w :tweet))
              (when ret
                (ros::publish "tweet" (instance std_msgs::String :init :data ret)))))
          )
      (ros::sleep)
      ))
  ) ;; end of defmethod

;; utility functions
(defvar *latest-enough-sec* 10.0)
(defun date- (dl dr)
  (- (send dl :compute-seconds)
     (send dr :compute-seconds)))

(defun latest-enough-p (m)
  (when m
      (< (date- (now)
                (cdr (assoc :inserted_at m)))
         *latest-enough-sec*)))


(setq *manager* (instance log-tweet-manager :init))
(send *manager* :add-worker
      (instance log-tweet-worker :init
                :name "function_call_count"
                :check-cb #'(lambda (self)
                              (let* ((last-check-date (send self :last-check))
                                     (q (car (mongo::query jsk_demo_common::FunctionEvent
                                                           ;; :meta `((:inserted_at .
                                                           ;;           ((:$gt . ,last-check-date)))
                                                                   (:status . "start"))
                                                           :sort '((:$natural . -1))
                                                           :single t))))
                                  (ros::ros-info "last-check: ~A" last-check-date)
                                  (ros::ros-info "q: ~A" q)
                                  (latest-enough-p (cdr q))))
                :tweet-cb #'(lambda (self q)
                              (let ((same-funcs (mongo::query jsk_demo_common::FunctionEvent
                                                              :query '((:name . (send (car q) :name))))))
                                (format nil "この動作の成功率は~Aパーセントです。"
                                        (* 100 (/
                                                (count-if #'(lambda (x)
                                                              (string= "success" (cdr (assoc :status (cdr x)))))
                                                          same-funcs)
                                                (count-if #'(lambda (x)
                                                              (string= "start" (cdr (assoc :status (cdr x)))))
                                                          same-funcs)))
                                        )))
                ))

(ros::rate 0.2) ;; [Hz]
(send *manager* :start)


(provide :pr2-tweet-log) ;; end of pr2-tweet-log.l
