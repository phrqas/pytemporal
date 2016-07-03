;;;; Copyright (c) 2016 Massachusetts Institute of Technology

;;;; This software may not be redistributed, and can only be retained and used
;;;; with the explicit written consent of the author, subject to the following
;;;; conditions:

;;;; The above copyright notice and this permission notice shall be included in
;;;; all copies or substantial portions of the Software.

;;;; This software may only be used for non-commercial, non-profit, research
;;;; activities.

;;;; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESSED
;;;; OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
;;;; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
;;;; THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
;;;; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
;;;; FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
;;;; DEALINGS IN THE SOFTWARE.

;;;; Authors:
;;;;   Pedro Santana (psantana@mit.edu)
(in-package #:roslisp)

(defconstant +infinity+ MOST-POSITIVE-DOUBLE-FLOAT)
(defconstant +neg-infinity+ MOST-NEGATIVE-DOUBLE-FLOAT)

(defun print-paris-response (resp)
  "Parses a solution generated by PARIS and prints it to the screen."
  (with-fields (success schedule squeezes risk_upper_bound) resp
    (if success
      (progn
          (format t "~%~%This is the schedule:~%")
          (print-strong-schedule schedule)
          (format t "~%~%These are the squeezes:~%")
          (print-temporal-constraint-squeezes squeezes)
          (format t "~%~%Risk upper bound: ~5$%" (* 100.0 risk_upper_bound)))
      (format t "~%No solution found!"))))

(defun print-strong-schedule (schedule)
  "Prints a table of event names and their schedules."
  (with-fields (event_names event_schedules) schedule
    (loop for sched-pair in (map 'list #'list event_names event_schedules)
      do (format t "~%~a" sched-pair))))

(defun print-temporal-constraint-squeezes (squeezes)
  "Prints the squeezes of probabilistic temporal constraints in a schedule."
  (loop for sq across squeezes
    do (with-fields (name lb_model ub_model lb_imposed ub_imposed squeeze_risk) sq
        (format t "~%~a: [~$,~$]->[~$,~$], Risk: ~5$%" name lb_model ub_model lb_imposed ub_imposed (* 100.0 squeeze_risk)))))

(defun make-ros-msg (msg-type fields)
    "Creates a generic ROS message (could not figure out how to pass a list as a field to roslisp:make-msg)."
    (apply #'make-instance `(,msg-type ,@fields)))
