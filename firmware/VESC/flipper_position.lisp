; =============================================================================
;  RoboCorea — Flipper position controller (runs ON the flipper VESC)
;  Target firmware: VESC 6.06 (LispBM)
; =============================================================================
;
;  WHY THIS EXISTS
;  ---------------
;  Each flipper is position-controlled, but the VESC's built-in position mode
;  only works over a single 0..360 turn and does not wrap. This script closes a
;  PD + stiction-feedforward loop with SHORTEST-PATH error on a WRAPPED [0,360)
;  angle, so the flipper can spin continuously past 360 in either direction.
;
;  HOW IT TALKS TO THE ESP32
;  -------------------------
;  The loop runs entirely on the VESC. The ESP32 only sends a target angle and
;  the VESC reports back its measured angle — both over custom CAN frames that
;  the firmware ignores (so they never trigger the built-in SET_RPM/SET_POS
;  controllers, which would fight our set-current):
;
;    ESP32 → VESC   ext-id = (0x7E << 8) | my-id     [int32 BE millideg][u8 enable]
;    VESC  → ESP32  ext-id = (0x7F << 8) | my-id     [int16 BE deci-deg measured]
;
;  Frame reception uses the `event-can-eid` event (frames with an undefined
;  command byte are handed to LispBM). See VESC LispBM reference for your build.
;
;  DEPLOY
;  ------
;  Flash this on the 4 FLIPPER VESCs only (CAN ids 20/10/40/30 per 1.ino).
;  Traction VESCs (60/50) keep stock firmware (SET_RPM velocity). `my-id` is read
;  from the VESC's own CAN id, so this same file works unmodified on all four.
;
;  TUNE ON BENCH (see firmware/ESP/include/config.h for the matching unknowns):
;    deg-per-dist  — get-dist → output degrees. THE critical constant.
;    kp / kd / fric / i-max — control gains and current limits.
; =============================================================================

; ─── Identity ────────────────────────────────────────────────────────────────
; Read this VESC's CAN id from its own config so one file fits all flippers.
; If `controller-id` is not the right symbol on your build, hard-code it instead.
(def my-id (conf-get 'controller-id))   ; e.g. 20 / 10 / 40 / 30

(def CMD-TARGET 0x7E)   ; ESP → VESC  (must match VESC_CMD_FLIPPER_TARGET)
(def CMD-REPORT 0x7F)   ; VESC → ESP  (must match VESC_CMD_FLIPPER_REPORT)

; ─── Tuning (UNKNOWNS — verify/tune on the bench) ────────────────────────────
(def deg-per-dist 1.14591559)  ; get-dist (m) → output degrees. CALIBRATE THIS.
(def kp 0.20)
(def kd 0.01)
(def fric 3.0)                 ; stiction feedforward current (A)
(def tol-deg 0.5)              ; deadband: below this |error| the ff is off
(def i-max 15.0)               ; current saturation (A)

(def loop-dt 0.005)            ; 200 Hz control period (s)
(def cmd-timeout 0.5)          ; s without a target frame → hold last setpoint
(def report-every 4)           ; emit a position report every Nth loop (~50 Hz)

; ─── State ───────────────────────────────────────────────────────────────────
(def target 0.0)               ; commanded angle, wrapped [0,360)
(def enabled 0)                ; 0 = coast, 1 = closed loop
(def last-cmd (systime))
(def prev-err 0.0)
(def loop-n 0)

; ─── Helpers ─────────────────────────────────────────────────────────────────
(defun wrap360 (x)
  (- x (* 360.0 (floor (/ x 360.0)))))          ; → [0,360)

(defun shortest (e)                              ; signed shortest path, [-180,180]
  (let ((w (wrap360 e)))
    (if (> w 180.0) (- w 360.0) w)))

(defun clampc (x)                                ; clamp to ±i-max
  (if (> x i-max) i-max (if (< x (- i-max)) (- i-max) x)))

; ─── Inbound CAN: decode a target frame addressed to this VESC ───────────────
(defun on-can (id data)
  (if (and (= (bitwise-and id 0xFF) my-id)
           (= (bitwise-and (shr id 8) 0xFF) CMD-TARGET)
           (>= (buflen data) 5))
      (progn
        (setq target  (wrap360 (/ (bufget-i32 data 0 'big-endian) 1000.0)))
        (setq enabled (bufget-u8 data 4))
        (setq last-cmd (systime)))))

(defun event-handler ()
  (loopwhile t
    (recv ((event-can-eid (? id) (? data)) (on-can id data))
          (_ nil))))

(event-register-handler (spawn event-handler))
(event-enable 'event-can-eid)

; ─── Outbound CAN: report measured angle back to the ESP ─────────────────────
(defun report (meas)
  (let ((b (bufcreate 2)))
    (bufset-i16 b 0 (to-i (* meas 10.0)) 'big-endian)
    (can-send-eid (bitwise-or my-id (shl CMD-REPORT 8)) b)))

; ─── Control loop ────────────────────────────────────────────────────────────
(loopwhile t
  (progn
    (let ((meas (wrap360 (* (get-dist) deg-per-dist))))

      (if (or (= enabled 0) (> (secs-since last-cmd) cmd-timeout))
          (progn (set-current 0.0) (setq prev-err 0.0))   ; coast / lost-comm
          (let* ((err (shortest (- target meas)))
                 (der (/ (- err prev-err) loop-dt))
                 (ff  (if (> err tol-deg) fric
                          (if (< err (- tol-deg)) (- fric) 0.0)))
                 (out (clampc (+ (* kp err) (* kd der) ff))))
            (set-current out)
            (setq prev-err err)))

      (setq loop-n (+ loop-n 1))
      (if (>= loop-n report-every) (progn (report meas) (setq loop-n 0)))
    )
    (sleep loop-dt)))
