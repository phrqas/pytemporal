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

(defun paris-stnu-stedl-test()
  (paris-tester (stnu-stedl)))

(defun paris-stp-picard-test()
  (paris-tester (stp-picard)))

(defun paris-stp-rubato-test()
  (paris-tester (stp-rubato)))

(defun paris-tester(pstnu)
  (let (paris-client resp)
    (setf paris-client (make-instance 'paris-cl-client))
    (setf resp (call-paris paris-client :pstnu pstnu))
    (print-paris-response resp)))
