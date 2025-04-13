---
title: "Reconstrucción Tridimensional"
description: "En esta práctica se pretende programar la lógica necesaria para permitir que un robot genere una reconstrucción 3D de la escena que está recibiendo a través de sus cámaras izquierda y derecha. "
publishDate: "13 Marzo 2025"
tags: [  "robótica",
  "visión por computadora",
  "reconstrucción 3D",
  "cámaras estéreo",
  "mapa de profundidad",]
coverImage:
  src: "./images_post/3D/real.png"
  alt: "Reconstrucción 3D"
  width: 1200  # Sin unidades (se asume px)
  height: 600
draft: false
---


## Adquisición de Imágenes Estéreo 📸

La visión estéreo usa dos cámaras que capturan la escena desde ángulos ligeramente distintos. En el escenario de la simulación estas cámaras se encuentran perfectamente alineadas de manera que, sus ejes ópticos son paralelos entre sí, sus planos de imagen son coplanares y las líneas epipolares son horizontales. 

A esta configuración ideal se le llama canónica, y al par de imágenes que cumplen estas condiciones se les llama imágenes estéreo rectificadas. Esta alineación simplifica mucho la búsqueda de puntos correspondientes, ya que solo hay que buscar a lo largo de la misma fila en la otra imagen. En la Figura 1, se representa la configuración canónica de dos cámaras junto a sus centros ópticos, la línea epipolar resultantes y la localización tridimensional de un punto característico. 

![Epipolar geometric](./images_post/3D/Epipolar-geometry.png)

**Python: Get Images**
```python title="3D_reconstruction.py"
imageLeft = HAL.getImage('left') 
imageRight = HAL.getImage('right')
``` 



## Preprocesamiento y Detección de Píxeles Característicos 

## Establecimiento de la Geometría Epipolar

## Búsqueda de Correspondencias

## Triangulación y Generación de la Nube de Puntos Tridimensional
