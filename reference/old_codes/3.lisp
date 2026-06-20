;; =================================================================
;; Controlador PD + Compensación de Fricción (Stiction Feedforward)
;; =================================================================

(def kp 0.20);;0.08
(def kd 0.0);;0.05
(def i-max 15.0)

;; Nuevos parámetros para romper la inercia mecánica
(def friccion-estatica 3.0)  ;; Amperios mínimos para que el motor se mueva
(def tolerancia-grados 0.5)  ;; Banda muerta: Si el error es menor a esto, apagamos el offset

(def error-previo 0.0)
(def contador-debug 0)

(def setpoint-grados 0.0)
(def dist-raw 0.0)
(def posicion-grados 0.0)
(def error-act 0.0)
(def deriv 0.0)
(def offset-friccion 0.0)
(def salida-pd 0.0)
(def salida-segura 0.0)

(def factor-conversion 1.14591559)

(loopwhile t
  (progn
    ;; 1. Lectura de variables
    (set 'setpoint-grados (/ (get-rpm-set) 1000.0))
    (set 'dist-raw (get-dist))
    (set 'posicion-grados (* dist-raw factor-conversion))
    
    ;; 2. Cálculo del Error y Derivada
    (set 'error-act (- setpoint-grados posicion-grados))
    (set 'deriv (- error-act error-previo))
    
    ;; 3. LÓGICA DE COMPENSACIÓN DE FRICCIÓN (Offset Direccional)
    (if (> error-act tolerancia-grados)
        (set 'offset-friccion friccion-estatica)  ;; Si falta llegar, empuja positivo
        (if (< error-act (- 0.0 tolerancia-grados))
            (set 'offset-friccion (- 0.0 friccion-estatica)) ;; Si se pasó, empuja negativo
            (set 'offset-friccion 0.0) ;; Si ya está en la meta, apaga el offset
        )
    )
    
    ;; 4. Ecuación de Control Total: PD + Offset
    (set 'salida-pd (+ (+ (* kp error-act) (* kd deriv)) offset-friccion))
    
    ;; 5. Saturación Segura a i-max
    (if (> salida-pd i-max)
        (set 'salida-segura i-max)
        (if (< salida-pd (- 0.0 i-max))
            (set 'salida-segura (- 0.0 i-max))
            (set 'salida-segura salida-pd)
        )
    )
    
    ;; 6. Actuación FOC y guardado de historial
    (set-current salida-segura)
    (set 'error-previo error-act)
    
    ;; 7. Telemetría a 10 Hz
    (set 'contador-debug (+ contador-debug 1))
    (if (> contador-debug 10)
        (progn
          (print "SP:" setpoint-grados " | Pos:" posicion-grados " | Err:" error-act " | Out(A):" salida-segura)
          (set 'contador-debug 0)
        )
        t
    )
    
    (sleep 0.01)
  )
)