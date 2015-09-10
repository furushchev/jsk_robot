#!/usr/bin/env roseus
;; i18n.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


(defvar *language* :ja)

(setq *phrase-dict* (make-hash-table))

(defun defphrase (keystr phrases)
  (let* ((key (intern keystr *keyword-package*))
         (alst (gethash key *phrase-dict*)))
    (dolist (lp phrases)
      (if (assoc (car lp) alst)
          (setf (cdr (assoc (car lp) alst)) (cdr lp))
        (push lp alst)))
    (sethash key *phrase-dict* alst)))

(defun _ (keystr)
  (let* ((key (intern keystr *keyword-package*))
         (alst (gethash key *phrase-dict*)))
    (if (and alst (assoc *language* alst))
        (cdr (assoc *language* alst))
      keystr)))

(provide :i18n) ;; end of i18n.l