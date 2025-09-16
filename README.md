# BIM2ROS — Integración BIM ↔ ROS para simulación, inspección y localización

> **Resumen**: Este repositorio conecta modelos BIM con el ecosistema de ROS/Gazebo para crear mundos simulados, generar mapas para la localización y realizar una estimación del estado del entorno. El objetivo es facilitar la evaluación de escenario construido durante el despliegue del robot.

---

## Índice
- [Introducción](#introducción)
- [Puesta en marcha y simulación](#puesta-en-marcha-y-simulación)
  - [Preparación mínima](#preparación-mínima)
  - [Ejecución rápida](#ejecución-rápida)
  - [Escenarios alternativos](#escenarios-alternativos)
  - [Comprobación visual](#comprobación-visual)
- [Vídeos ](#vídeos)

---

## Introducción

BIM2ROS actúa como puente entre **Building Information Modeling (BIM)** y **ROS**. A partir de un modelo IFC, el sistema:

- **Construye el mundo en Gazebo** (estructuras, salas, elementos móvivles...).  
- **Deriva un mapa 2D** utilizable por `map_server` y algoritmos de localización.  
- **Lanza el algoritmo de localización (AMCL)** para estimar la posición del robot en el escenario.  
- **Estimma el estado** de los elementos a través de la infomración del entorno y la actualización del mapa generado.

Este repositorio se centra en los módulos que automatizan la generación de esos artefactos y su puesta en escena para simulación.

---

## Puesta en marcha y simulación

> Asegúrate de usar el Dockerfile proporcionado y tener todas las dependencias correctamente instaladas.
> También es necesario instanciar al mismo nivel que la carpeta bim2ros el paquete laser_line_extraction, disponible en https://github.com/kam3k/laser_line_extraction
> También es necesario descargar la herramienta IfcConvert e instanciarla en el espacio de trabajo. Disponible en https://docs.ifcopenshell.org/ifcconvert/installation.html

### Preparación mínima
1. Coloca el **IFC** de tu edificio en la carpeta `Models/`.  
2. Selecciona el fichero **YAML** con parámetros BIM (p. ej., `config/atlas.yaml`).  
3. Preparación: revisa y ajusta los parámetros de tu **YAML** en función de las necesidades de tu modelo **IFC**.

### Ejecución rápida

1. **Arranca el master de ROS**

    ```bash
    roscore
    ```

2. **Carga parámetros BIM en el Parameter Server**

    ```bash
    rosparam load $(bim2ros)/config/atlas.yaml /bim
    ```

3. **(Opcional) Verifica que están disponibles**

    ```bash
    rosparam get /bim
    ```

4. **Genera el mundo desde IFC para Gazebo**

    ```bash
    roslaunch bim2ros ifc2world.launch
    ```

5. **Crea el mapa utilizable por navegación**

    ```bash
    roslaunch bim2ros bim2map.launch
    ```

6. **Lanza el entorno completo (robot + AMCL + RViz)**

    ```bash
    roslaunch bim2ros gazebo2map.launch
    ```

### Escenarios alternativos
Para cambiar de escenario, sustituye el YAML del paso 2 por el que corresponda a tu IFC y vuelve a ejecutar la secuencia.

### Comprobación visual
Gazebo: deberías ver la geometría del edificio y el robot en el escenario.

RViz: verifica la publicación de map, odom, base_link, las partículas de AMCL y la pose estimada.

Tópicos habituales a observar: /map, /amcl_pose, /tf, /scan... etc.

## Vídeos
El siguiente material audiovisual se publica para acompañar el uso del repositorio. En los siguientes vídeos se puede ver la ejecución de los comandos y la simulación final en funcionamiento para cada uno de los escenarios:

▶️ Escenario 1. Escenario sencillo con solo 2 elementos móviles: escenario_validacion.ifc. [(https://youtu.be/an2jK0vwwDA)]

▶️ Escenario 2. Escenario de una casa con 2 plantas. Prueba en la planta baja: casa.ifc. [(https://youtu.be/f6AINT04ZZI)]

▶️ Escenario 2. Escenario de una casa con 2 plantas. Prueba en la primera planta: casa.ifc. [(https://youtu.be/Tnkp7ZhiKhc)]

▶️ Escenario 3. Escenario amplio con gran cantidad de elementos: atlas.ifc. [(https://youtu.be/E99v2mVmk7w))]

▶️ Escenario 4. Escenario más realista: hospital.ifc. [(https://youtu.be/P42qYLm3Eq4)]


