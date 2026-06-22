;; =================================================================
;; Controlador PD Industrial (Derivada sobre Medición + Escala Real)
;; =================================================================

(def kp 0.22)
(def kd 0.1) 
(def i-max 15.0)

(def friccion-estatica 4.0)
(def tolerancia-grados 0.5)

;; Ahora guardamos la posición previa, no el error previo
(def pos-previa 0.0) 
(def contador-debug 0)

(def setpoint-grados 0.0)
(def dist-raw 0.0)
(def posicion-grados 0.0)
(def error-act 0.0)
(def vel-flipper 0.0)
(def offset-friccion 0.0)
(def salida-pd 0.0)
(def salida-segura 0.0)

(def factor-conversion 1.14591559)

(loopwhile t
  (progn
    ;; 1. LECTURAS
    (set 'setpoint-grados (/ (get-rpm-set) 1000.0))
    (set 'dist-raw (get-dist))
    (set 'posicion-grados (* dist-raw factor-conversion))
    
    (set 'error-act (- setpoint-grados posicion-grados))
    
    ;; ===============================================================
    ;; 2. DERIVADA REAL (Velocidad en Grados/Segundo)
    ;; ===============================================================
    ;; Restamos la posición y multiplicamos x100 (porque dt = 0.01s)
    (set 'vel-flipper (* (- posicion-grados pos-previa) 100.0))
    
    ;; 3. COMPENSACIÓN DE FRICCIÓN
    (if (> error-act tolerancia-grados)
        (set 'offset-friccion friccion-estatica)
        (if (< error-act (- 0.0 tolerancia-grados))
            (set 'offset-friccion (- 0.0 friccion-estatica))
            (set 'offset-friccion 0.0)
        )
    )
    
    ;; ===============================================================
    ;; 4. ECUACIÓN DE CONTROL 
    ;; ===============================================================
    (set 'salida-pd (+ (- (* kp error-act) (* kd vel-flipper)) offset-friccion))
    
    ;; 5. SATURACIÓN DE CORRIENTE
    (if (> salida-pd i-max)
        (set 'salida-segura i-max)
        (if (< salida-pd (- 0.0 i-max))
            (set 'salida-segura (- 0.0 i-max))
            (set 'salida-segura salida-pd)
        )
    )
    
    ;; 6. ACTUACIÓN Y ACTUALIZACIÓN
    (set-current salida-segura)
    (set 'pos-previa posicion-grados) ;; Guardamos para el siguiente loop
    
    ;; 7. TELEMETRÍA (10 Hz)
    (set 'contador-debug (+ contador-debug 1))
    (if (> contador-debug 10)
        (progn
          (print "SP:" setpoint-grados " | Pos:" posicion-grados " | Vel(deg/s):" vel-flipper " | Out:" salida-segura)
          (set 'contador-debug 0)
        )
        t
    )
    
    (sleep 0.01)
  )
)