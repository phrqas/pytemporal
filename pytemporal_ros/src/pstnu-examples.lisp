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

(defun stnu-stedl ()
  "Example in John Stedl's thesis. It has NO strongly controllable solution."
  (let ((tcs nil))
    (push (make-uncontrollable-bounded-temporal-constraint-msg :name "c1" :start "s1" :end "e1" :lb 5.0 :ub 10.0) tcs)
    (push (make-controllable-temporal-constraint-msg :name "c2" :start "s1" :end "s2" :lb 4.0 :ub +infinity+) tcs)
    (push (make-uncontrollable-bounded-temporal-constraint-msg :name "c3" :start "s2" :end "e2" :lb 1.0 :ub 2.0) tcs)
    (push (make-controllable-temporal-constraint-msg :name "c4" :start "e2" :end "e1" :lb 0.0 :ub +infinity+) tcs)
    (make-pstnu-msg :name "stnu_stedl" :tcns tcs)))

(defun stp-picard()
  "Pedagogical example in the Picard paper. It has a strongly controllable solution."
  (let ((tcs nil))
    (push (make-probabilistic-gaussian-temporal-constraint-msg :name "traversal" :start "dep" :end "arr" :mean 20.0 :var 4.0) tcs)
    (push (make-probabilistic-gaussian-temporal-constraint-msg :name "eruption" :start "SoD" :end "erupt" :mean 60.0 :var 25.0) tcs)
    (push (make-controllable-temporal-constraint-msg :name "arrival" :start "erupt" :end "arr" :lb 0.0 :ub 120.0) tcs)
    (make-pstnu-msg :name "stp_picard" :tcns tcs)))

(defun stp-rubato()
  "Pedagogical example in the Rubato paper. It has a strongly controllable solution."
  (let ((tcs nil))
    (push (make-probabilistic-gaussian-temporal-constraint-msg :name "tear-down-A" :start "start" :end "end-tear-down-A" :mean 15.0 :var 2.0) tcs)
    (push (make-probabilistic-gaussian-temporal-constraint-msg :name "tear-down-B" :start "start" :end "end-tear-down-B" :mean 30.0 :var 5.0) tcs)
    (push (make-probabilistic-gaussian-temporal-constraint-msg :name "vacuum-A" :start "end-tear-down-A" :end "end-vacuum-A" :mean 10.0 :var 3.0) tcs)
    (push (make-probabilistic-gaussian-temporal-constraint-msg :name "set-up-A" :start "end-vacuum-A" :end "end-set-up-A" :mean 10.0 :var 2.0) tcs)
    (push (make-probabilistic-gaussian-temporal-constraint-msg :name "vacuum-B" :start "start-vaccum-B" :end "end-vacuum-B" :mean 20.0 :var 5.0) tcs)
    (push (make-probabilistic-gaussian-temporal-constraint-msg :name "set-up-B" :start "end-vacuum-B" :end "end-set-up-B" :mean 10.0 :var 2.0) tcs)
    (push (make-controllable-temporal-constraint-msg :name "wait1" :start "end-tear-down-B" :end "start-vacuum-B" :lb 0.0 :ub +infinity+) tcs)
    (push (make-controllable-temporal-constraint-msg :name "wait2" :start "end-vacuum-A" :end "start-vacuum-B" :lb 0.0 :ub +infinity+) tcs)
    (push (make-controllable-temporal-constraint-msg :name "set-up-A-in-1-hour" :start "start" :end "end-set-up-A" :lb 0.0 :ub 60.0) tcs)
    (push (make-controllable-temporal-constraint-msg :name "set-up-B-in-1.5-hours" :start "start" :end "end-set-up-B" :lb 0.0 :ub 90.0) tcs)
    (make-pstnu-msg :name "stp_rubato" :tcns tcs)))
