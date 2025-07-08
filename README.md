
# 🚗 Proyecto F1Tenth: Controlador Reactivo - Follow the Gap

Este proyecto implementa un controlador reactivo para un vehículo autónomo del simulador F1Tenth usando el enfoque **Follow the Gap**. Fue desarrollado como parte del curso de Vehículos Autónomos y permite completar 10 vueltas sin colisiones, integrando mejoras para maximizar velocidad y estabilidad.

---

## 📌 Enfoque Utilizado: Follow the Gap

Follow the Gap es un enfoque reactivo que analiza los datos del LiDAR en tiempo real para encontrar la región libre más grande (gap) frente al vehículo. A partir de esa región, se elige el punto más adecuado para dirigir el vehículo y evitar obstáculos.

Pasos clave:
1. Preprocesamiento del LiDAR: limpieza, reemplazo de valores extremos.
2. Eliminación de una burbuja alrededor del obstáculo más cercano.
3. Búsqueda del mayor gap (espacio libre).
4. Selección del punto medio del gap como objetivo de navegación.
5. Publicación de comandos de giro y velocidad con mensajes Ackermann.

---

## 📂 Estructura del Código

El nodo `follow_the_gap_node.py` contiene:

- `preprocess_lidar`: Limpieza y recorte de los datos del LiDAR.
- `find_max_gap`: Identificación del mayor espacio libre entre obstáculos.
- `find_best_point`: Selección del mejor punto dentro del gap.
- `lidar_callback`: Función principal del algoritmo reactivo.
- `check_vuelta`: Contador de vueltas y cronómetro usando odometría.
- `publish_drive`: Publicación del ángulo y velocidad con AckermannDrive.

Se añaden funcionalidades para:
- Contar vueltas cruzando una zona virtual de meta (con forma de rombo).
- Medir el tiempo de cada vuelta en minutos con 3 decimales.
- Ajustar automáticamente la velocidad según el ángulo de giro para mejorar el tiempo sin comprometer la estabilidad.

---

## ▶️ Instrucciones de Ejecución

1. Asegúrate de tener el simulador F1Tenth en ROS2 Humble correctamente instalado.
2. Copia el archivo `follow_the_gap_node.py` en la carpeta `~/F1Tenth-Repository/src/follow_the_gap/`.
3. Compila el workspace si es necesario:

```bash
colcon build --packages-select follow_the_gap
source install/setup.bash
```

4. Ejecuta el nodo:

```bash
ros2 run follow_the_gap follow_the_gap_node
```

---

## 📈 Mejoras de Optimización

- **Aumento de campo visual** del LiDAR para anticipar curvas.
- **Ajuste dinámico de velocidad**: velocidad máxima en rectas (hasta 11.3 m/s) y reducción automática al acercarse a curvas.
- **Ángulo de giro ajustado** para evitar giros bruscos y reducir zigzag.
- **Aumento del umbral en `find_max_gap`** para una navegación más fluida.
- El controlador logra **hasta 0.616 min por vuelta**, uno de los mejores tiempos registrados.

---

## 🎯 Resultados

- ✅ 10 vueltas completas sin colisiones.
- ✅ Contador de vueltas funcionando en consola.
- ✅ Cronómetro por vuelta visible en terminal.
- ✅ Controlador estable y veloz.

---

## 👤 Autor

**Xavier Vilema**  
GitHub: [@XavierVilema](https://github.com/XavierVilema)

---
