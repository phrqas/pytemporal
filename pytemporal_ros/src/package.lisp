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
;;;;   Pedro Santana

(in-package #:cl-user)

;; Import the roslisp functions
(eval-when (:compile-toplevel :load-toplevel)
           (use-package :roslisp)
           ;;Loads message and service types
           ;(roslisp:load-message-types "pytemporal_ros")
           ;(roslisp:load-service-types "pytemporal_ros")
           )
