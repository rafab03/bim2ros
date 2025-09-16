import multiprocessing
import ifcopenshell
import ifcopenshell.geom
import numpy as np
from collections import defaultdict
import os
import rospy
import roslib
from PIL import Image
import yaml

PACKAGE_NAME = 'bim2ros'
RES = rospy.get_param('/bim/resolution', 0.05)
GRID_SIZEX = rospy.get_param('/bim/world_sizeX', 20)
GRID_SIZEY = rospy.get_param('/bim/world_sizeY', 20)
GRID_SIZEZ = rospy.get_param('/bim/world_sizeZ', 4)

def get_package_path(package_name):
    return roslib.packages.get_pkg_dir(package_name)

def setup_ifc_geometry(ifc_file_path):
    ifc_file = ifcopenshell.open(ifc_file_path)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.DISABLE_OPENING_SUBTRACTIONS, False)
    settings.set(settings.USE_WORLD_COORDS, True)
    return ifc_file, settings

def initialize_iterator(settings, ifc_file):
    iterator = ifcopenshell.geom.iterator(settings, ifc_file, multiprocessing.cpu_count())
    tree = ifcopenshell.geom.tree()
    if iterator.initialize():
        while True:
            shape = iterator.get_native()
            tree.add_element(shape)
            if not iterator.next():
                break
    return tree

def generar_grid_y_exportar(tree, ifc_file, settings, RES, tam_x, tam_y, output_folder, map_name, altura_fija):
    grid = np.full((tam_y, tam_x), -1)
    movable_mask = np.zeros((tam_y, tam_x), dtype=np.uint8)
    non_movable_mask = np.zeros((tam_y, tam_x), dtype=np.uint8)
    global_id_to_int = {}
    current_int = 1

    result_dict = defaultdict(int)
    movable_dict = defaultdict(int)
    non_movable_dict = defaultdict(int)

    non_movable_types = {'IfcWall', 'IfcSlab', 'IfcRoof', 'IfcColumn', 'IfcBeam', 'IfcStair', 'IfcRailing', 'IfcStairFlight', 'IfcCurtainWall', 'IfcFurnishingElement'}
    movable_types = {'IfcDoor', 'IfcWindow'}

    for y in range(tam_y):
        for x in range(tam_x):
            search_point = (x * RES, y * RES, altura_fija)
            elements = tree.select(search_point, extend=RES)

            if not elements:
                grid[y, x] = 0
                continue

            cell_occupied = False

            for item in elements:
                gid = item.GlobalId
                if gid not in global_id_to_int:
                    global_id_to_int[gid] = current_int
                    current_int += 1

                element = ifc_file.by_guid(gid)
                if not element:
                    continue

                if any(element.is_a(t) for t in movable_types):
                    movable_mask[y, x] = 1
                    movable_dict[gid] += 1
                elif any(element.is_a(t) for t in non_movable_types):
                    non_movable_mask[y, x] = 1
                    cell_occupied = True
                    non_movable_dict[gid] += 1

                result_dict[gid] += 1

            grid[y, x] = 100 if cell_occupied else 0

    print("=== DEBUG: Generación de mapa completada ===")
    print(f"Elementos incluidos: {len(global_id_to_int)}")
    print(f"Movibles: {len(movable_dict)}, No movibles: {len(non_movable_dict)}")

    pgm_path = os.path.join(output_folder, f"{map_name}.pgm")
    image_data = 255 - grid.astype(np.uint8)
    image_data = np.flipud(image_data)
    img = Image.fromarray(image_data)
    img.save(pgm_path)
    print(f"Mapa PGM exportado: {pgm_path}")

    yaml_path = os.path.join(output_folder, f"{map_name}.yaml")
    map_metadata = {
        'image': f"{map_name}.pgm",
        'resolution': RES,
        'origin': [0.0, 0.0, 0.0],
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'negate': 0,
        'movable_mask': movable_mask.tolist(),
        'non_movable_mask': non_movable_mask.tolist()
    }
    with open(yaml_path, 'w') as f:
        yaml.dump(map_metadata, f, default_flow_style=False)
    print(f"YAML exportado: {yaml_path}")

    return {
        "grid": grid,
        "movable_mask": movable_mask,
        "non_movable_mask": non_movable_mask,
        "pgm_path": pgm_path,
        "yaml_path": yaml_path,
        "global_id_to_int": global_id_to_int,

    }

def generate_gazebo_launch_from_yaml(yaml_params, launch_filename="gazebo2map.launch"):

    map_ifc = yaml_params["map"]  
    model_name = os.path.splitext(map_ifc)[0]

    spawn = yaml_params["robot_spawn"]
    x, y, z = spawn["x"], spawn["y"], spawn["z"]

    # Contenido del archivo launch
    launch_content = f"""<launch>

  <param name="use_sim_time" value="true"/>

  <!-- Gazebo con mundo -->
  <arg name="world_file" default="$(find bim2ros)/worlds/{model_name}.world"/>
  <param name="world_file" value="$(arg world_file)" />
  
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name"   value="$(arg world_file)"/>
    <arg name="paused"       value="false"/>
    <arg name="use_sim_time" value="true"/>
  </include>

  <!-- Spawner del robot -->
  <arg name="model" default="waffle" />
  <arg name="x_pos" default="{x}"/>
  <arg name="y_pos" default="{y}"/>
  <arg name="z_pos" default="{z}"/>

  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
  <node pkg="gazebo_ros" type="spawn_model" name="spawn_urdf"
        args="-urdf -model turtlebot3_$(arg model)
              -x $(arg x_pos)
              -y $(arg y_pos)
              -z $(arg z_pos)
              -param robot_description" />

  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />

  <!-- Nodo para mapeo dinámico -->
  <node name="dynamic_map_updater" pkg="bim2ros" type="dynamic_map_updater.py" output="screen">
    <param name="map_yaml" value="$(find bim2ros)/grids/{model_name}.yaml" />
    <param name="x_pos" value="$(arg x_pos)"/>
    <param name="y_pos" value="$(arg y_pos)"/>
    <param name="z_pos" value="$(arg z_pos)"/>
  </node>

  <!-- Localización AMCL -->
  <node pkg="amcl" type="amcl" name="amcl" output="screen"
        launch-prefix="bash -c 'sleep 5; exec $0 $@'">
    <param name="min_particles" value="1000"/>
    <param name="max_particles" value="5000"/>
    <param name="update_min_d" value="0.2"/>
    <param name="update_min_a" value="0.1"/>
    <param name="resample_interval" value="1"/>
    <param name="laser_z_hit" value="0.7"/>
    <param name="laser_z_rand" value="0.3"/>
    <param name="odom_model_type" value="diff-corrected"/>
    <param name="odom_alpha1" value="0.03"/>
    <param name="odom_alpha2" value="0.03"/>
    <param name="odom_alpha3" value="0.06"/>
    <param name="odom_alpha4" value="0.06"/>
    <param name="odom_alpha5" value="0.0"/>
    <param name="base_frame_id" value="base_link"/>
    <param name="odom_frame_id" value="odom"/>
    <param name="global_frame_id" value="map"/>

    <remap from="scan" to="/scan"/>
  </node>

  <!-- Pose inicial del robot -->
  <node pkg="bim2ros" type="publish_initial_pose.py" name="initial_pose_pub" output="screen" >
    <param name="x" value="{x}"/>
    <param name="y" value="{y}"/>
    <param name="z" value="{z}"/>

  </node>

  <!-- Visualización -->
  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find bim2ros)/config/RViz_map.rviz" />


</launch>
"""
    pkg_path = os.popen("rospack find bim2ros").read().strip()
    launch_dir = os.path.join(pkg_path, "launch")
    os.makedirs(launch_dir, exist_ok=True)
    launch_path = os.path.join(launch_dir, launch_filename)

    # Guardar contenido
    with open(launch_path, "w") as f:
        f.write(launch_content)

    rospy.loginfo(f" Archivo .launch generado: {launch_path}")
    return launch_path

if __name__ == "__main__":
    rospy.init_node('sGridGeneration')
    ifc_file_path = os.path.join(get_package_path(PACKAGE_NAME), 'models/', rospy.get_param('/bim/map'))
    ifc_file, settings = setup_ifc_geometry(ifc_file_path)
    tree = initialize_iterator(settings, ifc_file)

    tam_x = int(GRID_SIZEX / RES)
    tam_y = int(GRID_SIZEY / RES)

    output_folder = os.path.join(get_package_path(PACKAGE_NAME), "grids")
    launch_output_folder = os.path.join(get_package_path(PACKAGE_NAME), "launch")
    map_param = rospy.get_param('/bim/map')
    map_name = os.path.splitext(os.path.basename(map_param))[0]
    altura_fija = rospy.get_param('/bim/altura_fija', 0.4)
    grid_data = generar_grid_y_exportar(tree, ifc_file, settings, RES, tam_x, tam_y, output_folder, map_name, altura_fija)
    generate_gazebo_launch_from_yaml(rospy.get_param("/bim"))
    print("=== DEBUG: Exportación de mapa completada ===")
    print(f"Ruta PGM: {grid_data['pgm_path']}")
    print(f"Ruta YAML: {grid_data['yaml_path']}")
    print(f"Launch file: {os.path.join(launch_output_folder, 'gazebo2map.launch')}")
