# Control de Robots Pololu 3Pi+ con Gripper

## Descripción General

Todos los archivos incluidos en este proyecto se utilizan para controlar los **robots Pololu 3Pi+** equipados con gripper.  
El sistema se encarga de **leer las posiciones de los marcadores (markers)** asociados a los cubos y a los robots, y **enviar comandos de velocidad y control del gripper** mediante **comunicación TCP** usando mensajes codificados en **CBOR**.

---

## Archivos Principales de Control

- **[`ControlCubo20_PID_NONLIN_PID_PID.m`](./ControlCubo20_PID_NONLIN_PID_PID.m)**  
  Controlador correspondiente al **robot 14**, encargado de manipular el **cubo 20**.

- **[`ControlCubo30.m`](./ControlCubo30.m)**  
  Controlador correspondiente al **robot 2**, encargado de manipular el **cubo 30**.

- **[`ControlCubo21.m`](./ControlCubo21.m)**  
  Controlador correspondiente al **robot 10**, encargado de manipular el **cubo 21**.

---

## Funciones Robotat

El proyecto también incluye varias funciones con el prefijo `robotat_`, las cuales permiten la **comunicación entre MATLAB, el sistema Robotat y los robots Pololu**.  
Entre las más relevantes se encuentran:

- `robotat_3pi_gripper_open` → Envía el comando para **abrir la garra**.  
- `robotat_3pi_gripper_close` → Envía el comando para **cerrar la garra**.  
- `robotat_3pi_set_wheel_velocities` → Envía las **velocidades de las ruedas** al robot.

---

## Versión de MATLAB Utilizada

El sistema fue desarrollado y probado en el siguiente entorno:


---

## Toolboxes Utilizados

| Toolbox | Versión |
|----------|----------|
| MATLAB | 23.2 (R2023b) |
| Simulink | 23.2 (R2023b) |
| Computer Vision Toolbox | 23.2 (R2023b) |
| Control System Toolbox | 23.2 (R2023b) |
| Curve Fitting Toolbox | 23.2 (R2023b) |
| DSP System Toolbox | 23.2 (R2023b) |
| Image Processing Toolbox | 23.2 (R2023b) |
| Instrument Control Toolbox | 23.2 (R2023b) |
| Optimization Toolbox | 23.2 (R2023b) |
| Parallel Computing Toolbox | 23.2 (R2023b) |
| Robotics System Toolbox | 23.2 (R2023b) |
| Robotics Toolbox for MATLAB | 10.4 |
| Signal Processing Toolbox | 23.2 (R2023b) |
| Simscape | 23.2 (R2023b) |
| Simscape Electrical | 23.2 (R2023b) |
| Simscape Multibody | 23.2 (R2023b) |
| Simulink Control Design | 23.2 (R2023b) |
| Statistics and Machine Learning Toolbox | 23.2 (R2023b) |
| Symbolic Math Toolbox | 23.2 (R2023b) |
| System Identification Toolbox | 23.2 (R2023b) |

---

## Uso del Proyecto

Todos los scripts pueden ejecutarse directamente en MATLAB.  
Para cada robot, se debe correr el script correspondiente a su número y cubo asignado.  
El sistema establecerá automáticamente la conexión TCP, enviará los comandos de movimiento y gestionará el control del gripper.


