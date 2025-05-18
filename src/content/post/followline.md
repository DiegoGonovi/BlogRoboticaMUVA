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
  alt: "Reconstrucción 3D"
  width: 1200  # Sin unidades (se asume px)
  height: 500
draft: false
---

## Controlador PID 🔧
El controlador PID (Proporcional–Integral–Derivativo) es un algoritmo de control ampliamente utilizado en robótica para lograr la estabilidad y precisión en el movimiento de los robots. Utilizan un mecanismo de control de retroalimentación en bucle cerrado que ajusta continuamente las salidas en función de la diferencia entre un punto de ajuste deseado y el valor medido. 

Funciona ajustando continuamente la salida del sistema en función de tres términos: proporcional (P), integral (I) y derivativo (D). A continuación, en la Figura 1, se adjunta el diagrama estándar de un controlador PID donde se pueden observar los componentes mencionados, junto con las expresiones matemáticas que definen su comportamiento.

![Diagrama PID](./images_post/FollowLine/diagrama_pid)
**_Figura 1_**: Diagrama Controlador PID \
_[Waseem, U. (2023, 20 junio). PID Controller & Loops: A Comprehensive Guide to Understanding and Implementation. Wevolver. https://www.wevolver.com/article/pid-loops-a-comprehensive-guide-to-understanding-and-implementation.]_

- Control Proporcional (P): Responde al error presente, generando una salida proporcional a su magnitud. Al aplicar una acción correctiva inmediata, el término P minimiza rápidamente los errores. Esto ayuda a que el robot se dirija hacia el punto deseado rápidamente. 
- Control Derivado (D): Prevé oscilaciones y movimientos bruscos al tener en cuenta la velocidad de cambio del error. Este enfoque amortigua las oscilaciones y estabiliza el sistema, especialmente durante las respuestas transitorias. Por tanto, ayuda a que el movimiento sea suave y preciso. 

- Control Integral (I): Aborda cualquier error persistente o desviaciones a largo plazo del punto de ajuste acumulando el error a lo largo del tiempo. Al integrar la señal de error, el término I asegura que el sistema se acerque y mantenga el punto de ajuste con precisión, eliminando los errores en estado estacionario. Esto es útil para eliminar errores de deriva o fluctuaciones en el movimiento. 


Un controlador PID calcula de forma continua una señal de error. En este contexto, el error representa cuánto se ha desviado el coche respecto a la trayectoria que debería seguir. A partir de esta información, el controlador ajusta la señal de control para corregir dicha desviación acorde a tres términos fundamentales: Ganancia Proporcional (Kp), proporción de la señal de error que contribuye a la salida del controlador. Ganancia Derivativa (Kd), anticipa el comportamiento futuro del error basado en su tasa actual de cambio. Ganancia Integral (Ki), acumula errores pasados y corrige el desvío sostenido.

La configuración de estas tres ganancias influye directamente en el comportamiento del sistema, provocando oscilaciones ante valores demasiado altos o respuestas lentas e imprecisas cuando lo valores son demasiado bajos. La salida resultante se aplica a los actuadores del sistema, modificando la velocidad o el ángulo de giro para mantener una conducción estable y precisa. 


## Obtención de la región de interés 🎯
(Recorte del horizonte + filtro HSV)

## Control cinemático 🚗
(Cálculo de error lateral/angulación y generación de comandos)

## Transición a modelo Ackermann 🏎️

## Visualización en tiempo real 👀
(HUD, mini‐mapa y métricas)

## Vídeos 🎥
1. [Coche de dinámica simple en el circuito simple.](https://youtu.be/JZIK89bfv90)
2. [Coche de dinámica simple en el circuito Montreal.](https://youtu.be/BtUnzcoujMU)
3. [Coche de dinámica simple en el circuito Montmelo.](https://youtu.be/Asdo_OwhfH4)
4. [Coche de dinámica simple en el circuito Nürburgring.](https://youtu.be/_HtSosXdhNs)
5. [Coche de dinámica Ackermann en el circuito simple.](https://youtu.be/53Szezdb8bA)