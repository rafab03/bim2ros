En esta carpeta se generarán las gráficas correspondientes al control de cada elemento. Cada gráfica contiene el valor real del enlace (azul), el valor estimado (naranja) y el error (verde).

Ejemplo de uso: rosrun bim2ros log_angles.py --link puerta1_link \
  --duration 60 \
  --csv src/bim2ros/graficas/puerta1.csv \
  --png src/bim2ros/graficas/puerta1.png