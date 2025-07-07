
# 🏎️ Proyecto Follow the Gap – F1Tenth

Este repositorio contiene el nodo ROS2 implementado para el controlador reactivo **Follow the Gap**, desarrollado como parte del Primer Parcial del curso de Vehículos Autónomos (F1Tenth).

## 📌 Enfoque utilizado: Follow the Gap

El enfoque **Follow the Gap** se basa en analizar los datos del LiDAR para:

1. **Preprocesar** los datos (limpieza, recorte y suavizado).
2. **Eliminar una burbuja** alrededor del obstáculo más cercano.
3. **Encontrar el gap más largo** (región libre más amplia).
4. **Elegir el mejor punto** dentro del gap.
5. **Girar hacia ese punto** y avanzar con una velocidad constante.

Este método permite al vehículo tomar decisiones de manera reactiva, sin necesidad de un mapa o planificación global.

---

## 🧠 Estructura del Código

El nodo principal está en `follow_the_gap_node.py` e implementa:

- `preprocess_lidar`: limpia y recorta los datos del LiDAR.
- `find_max_gap`: identifica la región más amplia sin obstáculos.
- `find_best_point`: elige el punto objetivo dentro del gap.
- `lidar_callback`: función principal que procesa cada escaneo.
- `check_vuelta`: cuenta las vueltas al detectar paso por la zona virtual de meta.
- `publish_drive`: envía comandos de velocidad y dirección al vehículo.

También incluye un **contador de vueltas** y un **cronómetro por vuelta** utilizando los mensajes de `/ego_racecar/odom`.

---

## 🚀 Instrucciones de Ejecución

1. Clonar el repositorio:
   ```bash
   git clone https://github.com/XavierVilema/Project-follow-the-gap.git
   ```

2. Copiar el nodo al workspace de ROS2:
   ```bash
   cd ~/F1Tenth-Repository/src
   mv Project-follow-the-gap/follow_the_gap .
   ```

3. Compilar el workspace:
   ```bash
   cd ~/F1Tenth-Repository
   colcon build
   source install/setup.bash
   ```

4. Ejecutar el nodo:
   ```bash
   ros2 run follow_the_gap follow_the_gap_node
   ```

5. (Opcional) Para observar el odómetro:
   ```bash
   ros2 topic echo /ego_racecar/odom
   ```

---

## 📊 Evidencia

- El nodo completa **10 vueltas sin colisiones** en el simulador.
- Se imprime por consola el número de vuelta y el tiempo (en minutos, con 3 decimales).
- El paso por la línea de meta se detecta mediante una **zona virtual inclinada** con coordenadas relativas al punto inicial (0,0).

---

## 🔁 Notas

- Este proyecto está en fase de mejora: se pueden ajustar parámetros de velocidad y lógica para reducir el tiempo por vuelta.
- La zona de meta es configurable en el código (`check_vuelta`) y evita falsos positivos en otras partes de la pista.

---

✍️ Autor: Xavier Vilema  
📅 Última actualización: Julio 2025
