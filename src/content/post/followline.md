---
title: "Fórmula 1 Sigue Líneas"
description: "En esta práctica se pretende programar un monoplaza de Fórmula 1 virtual para que siga de forma autónoma la línea central de un circuito utilizando un controlador proporcional, integral y derivativo (PID). El reto combina visión artificial y control cinemático para mantener el coche en la trayectoria óptima y completar la vuelta en el menor tiempo posible, modificando tanto su velocidad como el ángulo de giro."
publishDate: "17 MAR 2025"
tags: [  "robótica",
  "visión por computadora",
  "sigue líneas",
  "control PID",
  "conducción autónoma",]
coverImage:
  src: "./images_post/FollowLine/portada_car.png"
  alt: "Sigue Líneas"
  width: 1200  # Sin unidades (se asume px)
  height: 600
draft: false
---

## Controlador PID 🔧
El controlador PID (Proporcional–Integral–Derivativo) es un algoritmo de control ampliamente utilizado en robótica para lograr la estabilidad y precisión en el movimiento de los robots. Utilizan un mecanismo de control de retroalimentación en bucle cerrado que ajusta continuamente las salidas en función de la diferencia entre un punto de ajuste deseado y el valor medido. 

Funciona ajustando la salida en función de tres términos: proporcional (P), integral (I) y derivativo (D). A continuación, en la Figura 1, se adjunta el diagrama estándar de un controlador PID donde se pueden observar los componentes mencionados, junto con las expresiones matemáticas que definen su comportamiento.

![Diagrama PID](./images_post/FollowLine/diagrama_pid.png)
**_Figura 1_**: Diagrama Controlador PID \
_[Waseem, U. (2023, 20 junio). PID Controller & Loops: A Comprehensive Guide to Understanding and Implementation. Wevolver. https://www.wevolver.com/article/pid-loops-a-comprehensive-guide-to-understanding-and-implementation.]_

- Control Proporcional (P): Responde al error presente, generando una salida proporcional a su magnitud. Al aplicar una acción correctiva inmediata, el término P minimiza rápidamente los errores. Esto ayuda a que el robot se dirija hacia el punto deseado rápidamente. 

- Control Derivativo (D): Prevé oscilaciones y movimientos bruscos al tener en cuenta la velocidad de cambio del error. Este enfoque amortigua las oscilaciones y estabiliza el sistema, especialmente durante las respuestas transitorias. Por tanto, ayuda a que el movimiento sea suave y preciso. 

- Control Integral (I): Aborda cualquier error persistente o desviaciones a largo plazo del punto de ajuste acumulando el error a lo largo del tiempo. Al integrar la señal de error, el término I asegura que el sistema se acerque y mantenga el punto de ajuste con precisión, eliminando los errores en estado estacionario. Esto es útil para eliminar errores de deriva o fluctuaciones en el movimiento. 


Un controlador PID calcula de forma continua una señal de error. En este contexto, el error representa cuánto se ha desviado el coche respecto a la trayectoria que debería seguir. A partir de esta información, el controlador ajusta la señal de control para corregir dicha desviación acorde a tres términos introducidos: Ganancia Proporcional (Kp), proporción de la señal de error que contribuye a la salida del controlador. Ganancia Derivativa (Kd), anticipa el comportamiento futuro del error basado en su tasa actual de cambio. Ganancia Integral (Ki), acumula errores pasados y corrige el desvío sostenido.

La configuración de estas tres ganancias influye directamente en el comportamiento del sistema, provocando oscilaciones ante valores demasiado altos o respuestas lentas e imprecisas cuando lo valores son demasiado bajos. La salida resultante se aplica a los actuadores del sistema, modificando la velocidad o el ángulo de giro para mantener una conducción estable y precisa. 


## Obtención de la región de interés 🎯
El primer paso de este reto consiste en definir la región de interés a partir de la cual se calculará el error de seguimiento. Dado que el objetivo es que el coche siga la línea roja del circuito, se aplica un filtro en el espacio de color HSV que permite segmentarla con precisión. 

Para ello, se construye una interfaz interactiva que permite ajustar dinámicamente los umbrales HSV sobre una captura del circuito, facilitando la selección precisa de los valores óptimos para detectar la línea roja de forma robusta. 

![Filtro HSV](./images_post/FollowLine/hsv1_hsv2_redimensionada.png)

Una vez se aplica el filtro de color, se detecta el área del contorno visualizado correspondiente a la línea roja sobre el asfalto. A partir de este contorno se calcula su centroide, que sirve como referencia visual para estimar la desviación del vehículo respecto al eje central deseado. No obstante, dado que la cámara no está alineada exactamente con el centro del vehículo, se aplica una corrección de dos píxeles para compensar este desplazamiento.

Además, en lugar de calcular el error sobre la zona más cercana al vehículo, se define una franja superior, cercana al horizonte visual, que permite anticipar cambios en la trayectoria y simular un comportamiento más similar al de un conductor humano.

![Mascara ROI](./images_post/FollowLine/mask_mix_redimensionada.png)

Por tanto, tras obtener la región de interés y el centroide de la línea roja, se calcula el error de seguimiento como la diferencia entre el centro de la imagen y la coordenada x del centroide. Este error indica cuánto se ha desviado el coche de la trayectoria ideal y será la entrada principal de los controladores, traduciéndose en correcciones sobre el ángulo de giro y la velocidad. 

Con el objetivo de facilitar la depuración y el análisis del comportamiento del sistema, se incorporan varios elementos visuales en la interfaz. Un punto amarillo, que representa el centroide de la línea roja detectada, una línea azul vertical, que indica el centro corregido de la imagen, sirviendo como referencia para el cálculo del error y valores numéricos del error de seguimiento, el ángulo de giro y velocidad lineal. 

![Depuracion Visual](./images_post/FollowLine/pantallafinal.png)


## Control cinemático 🚗
El sistema de control se compone de dos tipos de controladores, diseñados para regular el movimiento del vehículo a partir del error visual calculado.
1.  **Controlador PDI para el giro (w)**

Para corregir la desviación respecto a la línea, se implementa un controlador PDI. Este se aplica directamente sobre el error lateral obtenido. 

**Python: PDI angular.**
```python title="Follow_line.py"
def pdi_angular(error_w, error_pre_w, error_acc_w, Kp_w, Ki_w, Kd_w):
    d_w = error_w - error_pre_w
    error_acc_w += error_w
    w = -Kp_w * error_w - Ki_w * error_acc_w - Kd_w * d_w
    return w, error_w, error_acc_w
``` 

Además, el ángulo de giro calculado se restringe dentro de un umbral definido para evitar maniobras excesivas o irreales, garantizando así un comportamiento más coherente con las limitaciones físicas de un vehículo real.

```python title="Follow_line.py"
w = max(min(w, MAX_W), MIN_W)
HAL.setW(w)
```

2. **Controladores PD para la velocidad lineal (v)**

En el caso de la velocidad lineal, se opta por un controlador PD en lugar de PDI. El componente proporcional permite ajustar la velocidad en función de la magnitud del error, mientras que la derivativa ayuda a anticipar cambios bruscos y a suavizar la transición entre tramos rectos y curvas. En cambio, el uso de un término integral no resulta adecuado en este contexto, ya que su acumulación puede llevar a un sobreimpulso no deseado, especialmente cuando el vehículo pasa por curvas prolongadas.

Con el fin de optimizar la velocidad máxima sin comprometer la estabilidad, se definen dos pares de controladores de velocidad. Uno para tramos rectos, donde se permite una velocidad más alta, y otro para curvas, donde la velocidad se reduce de forma agresiva. Esta estrategia permite adaptar dinámicamente la velocidad al trazado del circuito, logrando un mayor rendimiento global. 

El cambio entre ambos controladores se determina mediante la suma de los errores más recientes junto a un umbral de decisión. Si esta suma es baja, se interpreta que el vehículo está siguiendo una recta y se puede aumentar la velocidad. En caso contrario, se asume una curva o una pérdida de precisión, y se reduce la velocidad.

```python title="Follow_line.py"
if abs(error_w) > threshold_c:
    # En curvas: velocidad baja
    v, error_pre_v = pd_velocidad_curvas(error_w, error_pre_v, Kp_v_c, Kd_v_c, v_min_c, v_max_c)
elif error_sum <= threshold_r:
    # En rectas: velocidad alta
    v, error_pre_v = pd_velocidad_rectas(error_sum, error_pre_v, Kp_v_r, Kd_v_r, v_min_r, v_max_r)
```

Tras programar la lógica de los controladores, se estiman las ganancias de cada uno siguiendo una versión simplificada del método de Ziegler–Nichols. Primero se incrementó el valor proporcional hasta alcanzar un punto de oscilación sostenida, y a partir de ahí se ajustan las constantes derivativa e integral de forma proporcional. Posteriormente, estas ganancias fueron refinadas observando el comportamiento en la simulación, hasta lograr una respuesta rápida, sin oscilaciones excesivas ni sobreimpulsos.

## Escenarios adversos 👀
Por último, se contempla un escenario adverso fundamental, la pérdida temporal de la línea roja en la imagen, ya sea por condiciones de iluminación, errores de segmentación o curvas extremadamente cerradas. En estos casos, el sistema no detecta contornos y, por tanto, no puede calcular el error de seguimiento ni aplicar los controladores descritos anteriormente.

Para evitar que el coche se detenga abruptamente o se descontrole, se implementa un comportamiento reactivo básico. El vehículo avanza lentamente girando en la dirección opuesta al último error registrado, con el objetivo de recuperar visualmente la línea. Aunque se trata de una solución simple, permite cierta tolerancia frente a fallos puntuales en la percepción.

```python title="Follow_line.py"
if not contours:
    if error_w < 0:
        HAL.setV(2)
        HAL.setW(1.5)
    else:
        HAL.setV(2)
        HAL.setW(-1.5)
```

En la sección de vídeos se adjuntan varias grabaciones que ilustran el comportamiento del vehículo simple en distintos circuitos. Estas simulaciones permiten observar la respuesta del controlador PID, la adaptación de la velocidad y la robustez del sistema ante curvas o pérdidas temporales de la línea.

## Transición a modelo Ackermann 🏎️

Una vez validado el comportamiento del controlador sobre el modelo de coche de dinámica simple, se aborda la transición al modelo Ackermann, más cercano a un vehículo real. A diferencia del modelo anterior, el coche Ackermann solo permite que las ruedas delanteras giren, mientras que las traseras siguen una trayectoria fija, lo que implica una respuesta más realista, pero también más exigente desde el punto de vista del control.

Como consecuencia, pequeñas desviaciones en el cálculo del giro pueden traducirse en trayectorias incorrectas, especialmente en curvas cerradas o cambios de dirección rápidos. Para compensar esta mayor sensibilidad, se han introducido varias mejoras clave en la lógica de control. 

- Se mantiene el controlador PDI para el giro, pero se le añade un filtro exponencial a la derivada para evitar oscilaciones indeseadas. 

```python title="Follow_line.py"
d_w_filt = 0.7 * d_w_filt + 0.3 * d_raw
```
- Se introduce una zona muerta para evitar correcciones pequeñas e innecesarias cuando el error angular es insignificante. 
```python title="Follow_line.py"
if abs(error_w) < 3:
    error_w = 0
```
- La señal de giro final (w) se suaviza mediante una rampa exponencial, lo que evita giros abruptos y mejora la estabilidad del coche. 
```python title="Follow_line.py"
w = 0.7 * w_prev + 0.3 * w_cmd
```
- En cuanto a la velocidad, se mantiene la lógica adaptativa con dos controladores PD, uno para curvas y otro para rectas. No obstante, la transición entre velocidades se modula usando un límite máximo de aceleración por ciclo para evitar picos bruscos.

```python title="Follow_line.py"
v = v_prev + np.clip(v_cmd - v_prev, -dv_max, dv_max)
```

Este conjunto de ajustes permite que el coche con dinámica Ackermann complete el circuito simple de forma estable, simulando un comportamiento realista y mostrando la capacidad del controlador para adaptarse a restricciones físicas más estrictas. No obstante, el tiempo por vuelta es considerablemente mayor, ya que es necesario reducir la velocidad para evitar que el coche se vuelva excesivamente reactivo. La idea futura es optimizar esta limitación mediante mejoras en los controladores y en el filtrado de la señal. 

El comportamiento actual puede observarse en la correspondiente grabación incluida en la sección de vídeos.

## Vídeos 🎥
1. [Coche de dinámica simple en el circuito simple.](https://youtu.be/JZIK89bfv90)
2. [Coche de dinámica simple en el circuito Montreal.](https://youtu.be/BtUnzcoujMU)
3. [Coche de dinámica simple en el circuito Montmelo.](https://youtu.be/I7RpnVOXL80)
4. [Coche de dinámica simple en el circuito Nürburgring.](https://youtu.be/_HtSosXdhNs)
5. [Coche de dinámica Ackermann en el circuito simple.](https://youtu.be/53Szezdb8bA)