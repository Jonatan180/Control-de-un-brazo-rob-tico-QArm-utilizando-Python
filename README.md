# Control de un brazo robótico QArm en Python

Este repositorio contiene el código fuente desarrollado en el marco de un proyecto académico correspondiente a la **Licenciatura en Automatización y Robótica (UNRaf)**. El trabajo evalúa la viabilidad del uso de **Python** como entorno de control para brazos robóticos, utilizando el **QArm de Quanser** como caso de estudio.

El sistema fue diseñado con una arquitectura modular, orientada a su uso en contextos educativos y experimentales, permitiendo ampliar las capacidades del QArm más allá de entornos propietarios tradicionales.

---

## Estructura del repositorio

El repositorio se organiza en tres carpetas principales, cada una correspondiente a un ejemplo o línea de desarrollo del proyecto:

### 📁 FINAL
Contiene la implementación principal del sistema de control del QArm, incluyendo:
- Comunicación con el hardware o entorno simulado  
- Control punto a punto mediante consignas articulares  
- Interfaz gráfica de usuario desarrollada en Python  
- Lógica de seguridad y validación de límites  

Este módulo representa el núcleo del proyecto y actúa como base para los demás desarrollos.

Videos:

Programación punto a punto del QArm en Python (timelapse): https://www.youtube.com/watch?v=REl6ruUV72A

Programación punto a punto del QArm en Python (tiempo real): https://www.youtube.com/watch?v=RGDOY5QbVYk

---

### 📁 INVERSE
Incluye los scripts correspondientes al **control por cinemática inversa** del QArm.  
Permite comandar el movimiento del efector final a partir de consignas cartesianas, funcionando como un prototipo experimental para el análisis del control cartesiano y sus limitaciones mecánicas.

Video:

Control por cinemática inversa del QArm en Python: https://www.youtube.com/watch?v=XAjjPOm35gA

---

### 📁 CAMERA
Contiene la implementación del **control asistido por visión artificial**.  
A partir de la información obtenida mediante una cámara, el sistema interpreta la posición y el estado de la mano del usuario para guiar el movimiento del efector final y accionar el gripper. Este desarrollo se presenta como un prototipo funcional orientado a la experimentación.

Video:

Control asistido por visión artificial del QArm en Python: https://www.youtube.com/watch?v=S1acEPbbCow

---

## Modo de operación

Los distintos módulos permiten seleccionar el modo de ejecución:
- **Simulado**, utilizando el entorno virtual provisto por Quanser  
- **Real**, mediante comunicación directa con el hardware del QArm  

Esta característica facilita el desarrollo, la depuración y el uso del sistema tanto en laboratorio como de forma remota.

---

## Contexto académico

Este repositorio acompaña la tesis de licenciatura titulada:

**“IMPLEMENTAR ENTORNO DE CONTROL BASADO EN PYTHON PARA SU APLICACIÓN EN BRAZOS ROBÓTICOS, UTILIZANDO EL QARM COMO CASO DE ESTUDIO”**

El código se presenta como material complementario para su análisis, replicación y reutilización en futuras prácticas académicas o proyectos de investigación.
