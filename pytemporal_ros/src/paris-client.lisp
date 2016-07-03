;;;; Copyright (c) 2016 Massachusetts Institute of Technology
;;;;
;;;; This software may not be redistributed, and can only be retained and used
;;;; with the explicit written consent of the author, subject to the following
;;;; conditions:
;;;;
;;;; The above copyright notice and this permission notice shall be included in
;;;; all copies or substantial portions of the Software.
;;;;
;;;; This software may only be used for non-commercial, non-profit, research
;;;; activities.
;;;;
;;;; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESSED
;;;; OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
;;;; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
;;;; THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
;;;; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
;;;; FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
;;;; DEALINGS IN THE SOFTWARE.
;;;;
;;;; Authors:
;;;;   Pedro Santana
(in-package #:roslisp)

(defclass paris-cl-client ()
  ((node-name
    :initarg :node-name
    :initform "paris-cl-client"
    :documentation "Name of the ROS node associated with this PARIS client.")
    (paris-service-name
     :initarg :paris-service-name
     :initform "paris_service"
     :documentation "Name of the PARIS ROS service.")
    (paris-service-timeout
     :initarg :paris-service-timeout
     :initform 10
     :documentation "Number of seconds that the client waits to connect to the service before returning an error.")
    (node-status
     :documentation "Stores the ROS node initialization status.")
   ))

(defmethod initialize-instance :after ((client paris-cl-client) &key)
  (with-slots ((nn node-name) (ns node-status) (ps paris-service-name) (to paris-service-timeout)) client
    (setf ns (start-ros-node nn))
    (format t "~%Waiting to connect to the PARIS service...")
    (if (wait-for-service ps to)
      (format t "connected!")
      (error "Failed to connect to PARIS service"))))

(defmethod call-paris ((client paris-cl-client) &key(name "") pstnu (gaussian_div 5)
                                                    (gaussian_optimize_partition nil)
                                                    (gaussian_lr 0.05) (gaussian_tol 1e-5)
                                                    (gaussian_max_iter 10000) (cc -1.0)
                                                    (makespan nil))
  "Calls the PARIS ROS service."
  (let (resp)
    (with-slots ((ps paris-service-name)) client
       (setf resp (call-service "paris_service" 'pytemporal_ros-srv:PARISSrv
                     :name name
                     :pstnu pstnu
                     :gaussian_div gaussian_div
                     :gaussian_optimize_partition gaussian_optimize_partition
                     :gaussian_lr gaussian_lr
                     :gaussian_tol gaussian_tol
                     :gaussian_max_iter gaussian_max_iter
                     :cc cc
                     :makespan makespan)))
    resp))

(defmethod shutdown-client ((client paris-cl-client))
    (shutdown-ros-node))
