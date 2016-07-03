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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; These should be the most commonly used functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun make-controllable-temporal-constraint-msg (&key name start end (lb 0.0) (ub 0.0))
    "Creates a controllable temporal constraint message."
    (make-bounded-temporal-constraint-msg name start end "controllable" lb ub))

(defun make-uncontrollable-bounded-temporal-constraint-msg (&key name start end (lb 0.0) (ub 0.0))
    "Creates a set-bounded uncontrollable temporal constraint message."
    (make-bounded-temporal-constraint-msg name start end "uncontrollable_bounded" lb ub))

(defun make-probabilistic-uniform-temporal-constraint-msg (&key name start end (lb 0.0) (ub 0.0))
    "Creates a probabilistic temporal constraint message with uniform distribution."
    (let (prop)
      (setf prop (list "distribution_type=uniform"
                       (format nil "distribution_lb=~5$" lb)
                       (format nil "distribution_ub=~5$" ub)))
      (make-temporal-constraint-msg name "uncontrollable_probabilistic" start end prop)))

(defun make-probabilistic-gaussian-temporal-constraint-msg (&key name start end (mean 0.0) (var 1.0))
  "Creates a probabilistic temporal constraint message with Gaussian (normal) distribution."
  (let (prop)
    (setf prop (list "distribution_type=gaussian"
                     (format nil "distribution_mean=~5$" mean)
                     (format nil "distribution_variance=~5$" var)))
    (make-temporal-constraint-msg name "uncontrollable_probabilistic" start end prop)))

(defun make-pstnu-msg (&key name tcns)
    "Creates a PSTNU message from a p-list of fields."
    (make-ros-msg 'pytemporal_ros-msg:PSTNU `(:name ,name :tcns ,tcns)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Functions below this point are used to implement the useful syntatic sugars above
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun make-bounded-temporal-constraint-msg (name start end type lb ub)
  "Creates a temporal constraint message with upper and lower bounds."
  (let (prop)
    (setf prop (list (format nil "lb=~5$" lb) (format nil "ub=~5$" ub)))
    (make-temporal-constraint-msg name type start end prop)))

(defun make-temporal-constraint-msg (name type start end prop)
    "Creates a temporal constraint message."
    (make-ros-msg 'pytemporal_ros-msg:TemporalConstraint `(:name ,name
                                                           :start_event_name ,start
                                                           :end_event_name ,end
                                                           :type ,type
                                                           :properties ,prop)))
