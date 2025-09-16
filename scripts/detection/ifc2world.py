#!/usr/bin/python3

import os
import subprocess
import math
import json
import glob
import shutil
import numpy as np
import xml.etree.ElementTree as ET
import xml.dom.minidom
import multiprocessing
import ifcopenshell
import ifcopenshell.geom
import ifcopenshell.util.placement
import rospy
import roslib
import stat
from collections import defaultdict

PACKAGE_NAME = 'bim2ros'

def get_package_path(package_name):
    return roslib.packages.get_pkg_dir(package_name)

def setup_ifc_geometry(ifc_file_path):
    ifc_file = ifcopenshell.open(ifc_file_path)
    settings = ifcopenshell.geom.settings()
    settings.set(settings.DISABLE_OPENING_SUBTRACTIONS, False)  # Ignorar huecos
    settings.set(settings.USE_WORLD_COORDS, True)               # Usar coordenadas globales
    return ifc_file, settings

def process_ifc_file(ifc_file, settings):

    result_dict       = defaultdict(int)
    movable_dict      = defaultdict(int)
    non_movable_dict  = defaultdict(int)
    link_poses        = {}
    global_id_to_int  = {}
    current_int       = 1

    non_movable_types = {'IfcWall', 'IfcSlab', 'IfcRoof', 'IfcColumn', 'IfcBeam', 'IfcStair', 'IfcRailing', 'IfcStairFlight', 'IfcCurtainWall', 'IfcFurnishingElement'}
    movable_types={'IfcDoor', 'IfcWindow'}

    iterator = ifcopenshell.geom.iterator(settings, ifc_file, multiprocessing.cpu_count())
    if not iterator.initialize():
        return global_id_to_int, movable_dict, non_movable_dict, link_poses

    while True:
        shape   = iterator.get()
        id     = shape.id               #Numero de entidad IFC
        element = ifc_file.by_id(id)
        gid=element.GlobalId            #ID global de la entidad 

        result_dict[id] += 1

        if gid not in global_id_to_int:
            global_id_to_int[gid] = current_int
            current_int += 1
            # Pose real
            link_poses[gid] = get_element_pose(element)


        if any(element.is_a(t) for t in non_movable_types):
            non_movable_dict[gid] += 1
        elif any(element.is_a(t) for t in movable_types):
            movable_dict[gid] += 1

        print(f"Elemento registrado: {gid} {element.is_a()}")

        if not iterator.next():
            break

    print("=== DEBUG: Resumen de process_ifc_file ===")
    print(f"Total de elementos únicos: {len(global_id_to_int)}")
    print(f"Movibles: {len(movable_dict)}, No movibles: {len(non_movable_dict)}")
    for gid, pose in link_poses.items():
        print(f"  {gid}: {pose}")

    return global_id_to_int, dict(movable_dict), dict(non_movable_dict), link_poses

def matrix_to_pose(matrix):

    # Extraer traslación 
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]

    # Cambio de escala si es necesario
    if max(abs(x), abs(y), abs(z)) > 1000:
        x /= 1000.0
        y /= 1000.0
        z /= 1000.0
    
    # Extraer la matriz de rotación (3x3)
    R = matrix[0:3, 0:3]
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = np.arctan2(R[2,1], R[2,2])
        pitch = np.arctan2(-R[2,0], sy)
        yaw   = np.arctan2(R[1,0], R[0,0])
    else:
        roll  = np.arctan2(-R[1,2], R[1,1])
        pitch = np.arctan2(-R[2,0], sy)
        yaw   = 0
    return f"{x} {y} {z} {roll} {pitch} {yaw}"

def get_element_pose(element):

    try:
        matrix = ifcopenshell.util.placement.get_local_placement(element.ObjectPlacement)
        pose_str = matrix_to_pose(matrix)
        return pose_str
    except Exception as ex:
        return "0 0 0 0 0 0"

def save_results(global_id_to_int):
    package_path = get_package_path(PACKAGE_NAME)
    grids_folder_path = os.path.join(package_path, "grids")
    os.makedirs(grids_folder_path, exist_ok=True)
    with open(os.path.join(grids_folder_path, 'global_id_mapping.json'), 'w') as file:
        json.dump(global_id_to_int, file, indent=4)

def export_elements_and_dae(ifc_path, export_folder, movable_ids):

    seen = set()

    # 1) Muros y losas agrupados
    static_file = os.path.join(export_folder, 'walls_slabs.dae')
    if os.path.exists(static_file):
        try: os.remove(static_file)
        except PermissionError:
            os.chmod(static_file, stat.S_IWUSR); os.remove(static_file)

    cmd_static = [
        "IfcConvert",
        "--element-hierarchy",
        "--include", "entities", "IfcWall", "IfcSlab", "IfcRoof", "IfcColumn", "IfcBeam", "IfcStair", "IfcRailing", "IfcStairFlight", "IfcCurtainWall", "IfcFurnishingElement",
        "-v", ifc_path, static_file
    ]
    print(f"[export] Muros/Losas a {static_file}")
    try:
        subprocess.run(cmd_static, check=True, capture_output=True, text=True)
    except subprocess.CalledProcessError as e:
        print(f"ERROR al exportar walls_slabs.dae:\n  {e.stderr or e}")

    # mover .tmp 
    if not os.path.exists(static_file):
        tmps = glob.glob(os.path.join(os.getcwd(), '.ifcopenshell.*.tmp'))
        if tmps:
            tmp = tmps[0]
            print(f"  Encontrado temporal para muros/losas: {tmp}")
            shutil.move(tmp, static_file)
    if os.path.exists(static_file):
        os.chmod(static_file, 0o777)
        seen.add('walls_slabs')
    else:
        print("  ¡Fallo total al generar walls_slabs.dae!")

    # 2) .dae por elemento móvil
    for gid in movable_ids:
        out = os.path.join(export_folder, f"{gid}.dae")

        if os.path.exists(out):
            try: os.remove(out)
            except PermissionError:
                os.chmod(out, stat.S_IWUSR)
                os.remove(out)

        # bloque de export 
        cmd = [
            "IfcConvert",
            "--center-model",
            "--element-hierarchy",
            "--include", "attribute", "GlobalId", gid, 
            "-v", ifc_path, out
        ]
        print(f"[export] Movible {gid} a {out}")
        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            print(f"ERROR al exportar {gid}:\n  {e.stderr or e}")

        # mover .tmp 
        if not os.path.exists(out):
            tmps = glob.glob(os.path.join(os.getcwd(), '.ifcopenshell.*.tmp'))
            if tmps:
                tmp = tmps[0]
                print(f" Encontrado temporal para {gid}: {tmp}")
                try: shutil.move(tmp, out)
                except Exception as mv:
                    print(f"  ERROR moviendo {tmp} a {out}: {mv}")
            else:
                print(f" No se encontró .tmp para {gid}")

        if os.path.exists(out):
            os.chmod(out, 0o777)
            seen.add(gid)
        else:
            print(f"Fallo al generar {gid}.dae")

    return seen

def compute_box_inertia(verts, density):

    mins = verts.min(axis=0)
    maxs = verts.max(axis=0)
    dims = maxs - mins                      
    volume = dims[0] * dims[1] * dims[2]
    mass = density * volume

    ixx = mass * (dims[1]**2 + dims[2]**2) / 12.0
    iyy = mass * (dims[0]**2 + dims[2]**2) / 12.0
    izz = mass * (dims[0]**2 + dims[1]**2) / 12.0
    return mass, ixx, iyy, izz

def create_combined_sdf(model_name, mesh_ids, output_folder, verts_path, movable_ids=None, link_poses=None):

    package_path   = get_package_path(PACKAGE_NAME)
    ifc_file_path  = os.path.join(package_path, 'models', f'{model_name}.ifc')
    ifc_file, settings = setup_ifc_geometry(ifc_file_path)

    movable_ids  = movable_ids or set()
    link_poses   = link_poses  or {}

    vertices_dict = {}
    poses_dict    = {}
    params_dict = {}

    sdf = ET.Element('sdf', version='1.6')
    model = ET.SubElement(sdf, 'model', name=model_name)
    ET.SubElement(model, 'static').text = 'false'

    # 1) link estático
    if 'walls_slabs' in mesh_ids:

        non_movables = []
        for t in ('IfcWall', 'IfcSlab', 'IfcRoof', 'IfcColumn', 'IfcBeam', 'IfcStair', 'IfcRailing', 'IfcStairFlight', 'IfcCurtainWall', 'IfcFurnishingElement'):
            non_movables += ifc_file.by_type(t)

        all_verts = []
        
        for element in non_movables:
            if not getattr(element, "Representation", None):
                continue           
            shape = ifcopenshell.geom.create_shape(settings, element)
            v = np.array(shape.geometry.verts).reshape(-1,3)
            all_verts.append(v)

        if all_verts:
            verts_all = np.vstack(all_verts)
            mass_w, ixx_w, iyy_w, izz_w = compute_box_inertia(verts_all, density=2400.0)             # densidad hormigón: 2400 kg/m3
        else:
            mass_w, ixx_w, iyy_w, izz_w = 100.0, 10.0, 10.0, 10.0

        link = ET.SubElement(model, 'link', name='pared_link')
        ET.SubElement(link, 'pose').text = '0 0 0 0 0 0'
        ET.SubElement(link, 'gravity').text = 'false'
        ET.SubElement(link, 'self_collide').text = 'false'
        for tag, nm in (('visual','pared_visual'), ('collision','pared_collision')):
            node = ET.SubElement(link, tag, name=nm)
            geom = ET.SubElement(node, 'geometry')
            mesh = ET.SubElement(geom, 'mesh')
            ET.SubElement(mesh, 'uri').text = f'model://{model_name}/meshes/walls_slabs.dae'

        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = f'{mass_w:.4f}'
        inertia = ET.SubElement(inertial, 'inertia')
        ET.SubElement(inertia, 'ixx').text = f'{ixx_w:.4f}'
        ET.SubElement(inertia, 'iyy').text = f'{iyy_w:.4f}'
        ET.SubElement(inertia, 'izz').text = f'{izz_w:.4f}'
        for comp in ('ixy','ixz','iyz'):
            ET.SubElement(inertia, comp).text = '0.0'


    # 2) puertas/ventanas
    door_count = 1
    window_count = 1

    for gid in movable_ids:

        raw_pose = link_poses.get(gid, '0 0 0 0 0 0')
        print(f"[DEBUG] {gid} pose original: {raw_pose}")
        parts = raw_pose.split()
        pose_floats = [float(x) for x in parts] 
        parts[3] = parts[4] = parts[5] = '0'
        pose_str = ' '.join(parts)                          # "x0 y0 z0 0 0 0"

        element = ifc_file.by_guid(gid)
        if element and getattr(element, "Representation", None):
            shape = ifcopenshell.geom.create_shape(settings, element)
            verts = np.array(shape.geometry.verts).reshape(-1,3)
        else:
            verts = np.zeros((0,3))

        if element and element.is_a('IfcWindow'):
            name_base = f'ventana{window_count}'
            window_count += 1
            density = 2000.0   # vidrio + marco
        else:
            name_base = f'puerta{door_count}'
            door_count += 1
            density =  600.0   # madera de puerta


        mass_m, ixx_m, iyy_m, izz_m = compute_box_inertia(verts, density=density)

        #Almacenar la posición en una lista
        link_name_full = f"{model_name}::{name_base}_link"
        poses_dict[link_name_full] = pose_floats   

        if verts.size > 0:
            minv = verts.min(axis=0)
            maxv = verts.max(axis=0)

            #Dimensiones locales
            dx, dy, dz = maxv - minv   

            local_corners = [
                (0.0, 0.0, 0.0),     
                (dx,  0.0, 0.0),
                (dx,  dy,  0.0),
                (0.0, dy,  0.0),
                (0.0, 0.0, dz),      
                (dx,  0.0, dz),
                (dx,  dy,  dz),
                (0.0, dy,  dz),
            ]
        else:
            local_corners = []
        vertices_dict[link_name_full] = local_corners

        link = ET.SubElement(model, 'link', name=f'{name_base}_link')
        ET.SubElement(link, 'pose').text = pose_str
        visual = ET.SubElement(link, 'visual', name=f'{name_base}_visual')
        geom_v = ET.SubElement(visual, 'geometry')
        mesh_v = ET.SubElement(geom_v, 'mesh')
        ET.SubElement(mesh_v, 'uri').text = f'model://{model_name}/meshes/{gid}.dae'

        collision = ET.SubElement(link, 'collision', name=f'{name_base}_collision')
        geom_c = ET.SubElement(collision, 'geometry')
        mesh_c = ET.SubElement(geom_c, 'mesh')
        ET.SubElement(mesh_c, 'uri').text = f'model://{model_name}/meshes/{gid}.dae'

        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = f'{mass_m:.4f}'
        inertia = ET.SubElement(inertial, 'inertia')
        ET.SubElement(inertia, 'ixx').text = f'{ixx_m:.4f}'
        ET.SubElement(inertia, 'iyy').text = f'{iyy_m:.4f}'
        ET.SubElement(inertia, 'izz').text = f'{izz_m:.4f}'
        for comp in ('ixy','ixz','iyz'):
            ET.SubElement(inertia, comp).text = '0.0'

        # Articulación
        if element and element.is_a('IfcWindow'):

            _, _, _, _, _, yaw_s = link_poses.get(gid, '0 0 0 0 0 0').split()
            yaw = float(yaw_s)
            yaw_deg = math.degrees(yaw)
            yaw_deg = (yaw_deg + 180) % 360 - 180                   # Normalizar a [-180,180]

            if abs(yaw_deg) <= 45 or abs(yaw_deg) >= 135:           # Alineada con X
                axis_xyz = '1 0 0'
            else:
                axis_xyz = '0 1 0'                                  # Alineada con Y

            joint = ET.SubElement(model, 'joint', name=f'prism_joint_{name_base}', type='prismatic')
            ET.SubElement(joint, 'parent').text = 'pared_link'
            ET.SubElement(joint, 'child').text  = f'{name_base}_link'
            ET.SubElement(joint, 'pose').text   = '0 0 0 0 0 0'
            axis = ET.SubElement(joint, 'axis')
            ET.SubElement(axis, 'xyz').text     = axis_xyz       
            limit = ET.SubElement(axis, 'limit')
            ET.SubElement(limit, 'lower').text  = '0.0'          
            ET.SubElement(limit, 'upper').text  = '1.5'          
            ET.SubElement(limit, 'effort').text = '500'
            ET.SubElement(limit, 'velocity').text = '0.5'

            dyn = ET.SubElement(axis, 'dynamics')
            damping_val = 0.2 * mass_m
            friction_val = 0.1* mass_m
            ET.SubElement(dyn, 'damping').text  = f'{damping_val:.4f}'
            ET.SubElement(dyn, 'friction').text = f'{friction_val:.4f}'
        else:
            joint = ET.SubElement(model, 'joint', name=f'rev_joint_{name_base}', type='revolute')
            ET.SubElement(joint, 'parent').text = 'pared_link'
            ET.SubElement(joint, 'child').text  = f'{name_base}_link'
            ET.SubElement(joint, 'pose').text   = '0 0 0 0 0 0'
            axis = ET.SubElement(joint, 'axis')
            ET.SubElement(axis, 'xyz').text     = '0 0 1'
            limit = ET.SubElement(axis, 'limit')
            ET.SubElement(limit, 'lower').text  = '-1.57'
            ET.SubElement(limit, 'upper').text  = '1.57'
            ET.SubElement(limit, 'effort').text = '500'
            ET.SubElement(limit, 'velocity').text = '1.0'

            dyn = ET.SubElement(axis, 'dynamics')
            damping_val = 0.1 * izz_m
            friction_val = 0.05 * izz_m
            ET.SubElement(dyn, 'damping').text   = f'{damping_val:.4f}'
            ET.SubElement(dyn, 'friction').text  = f'{friction_val:.4f}'
        
        # Almacenar parámetros físicos
        params_dict[link_name_full] = {
            "mass": mass_m,
            "damping": damping_val,
            "friction": friction_val
        }

    # Guardar JSON con vértices
    json_path = os.path.join(verts_path, f'{model_name}_movable_vertices.json')
    with open(json_path, 'w') as jf:
        json.dump(vertices_dict, jf, indent=4)

    #  Guardar JSON con poses
    json_poses = os.path.join(verts_path, f'{model_name}_movable_poses.json')
    with open(json_poses, 'w') as pf:
        json.dump(poses_dict, pf, indent=4)
    print(f'Poses saved to {json_poses}')

    # Guardar JSON con parámetros físicos
    json_params_path = os.path.join(verts_path, f'{model_name}_movable_physics.json')
    with open(json_params_path, 'w') as pf:
        json.dump(params_dict, pf, indent=4)


    raw = ET.tostring(sdf, encoding='unicode')
    dom = xml.dom.minidom.parseString(raw)
    pretty = dom.toprettyxml(indent='    ')
    sdf_path = os.path.join(output_folder, f'{model_name}.sdf')
    with open(sdf_path, 'w') as f:
        f.write(pretty)

    print(f'Total de elementos movibles: {len(movable_ids)}')
    return sdf_path


def create_model_config(model_name, output_folder):
    config = ET.Element("model")
    ET.SubElement(config, "name").text = model_name
    ET.SubElement(config, "version").text = "1.0"
    ET.SubElement(config, "sdf", version="1.6").text = f"{model_name}.sdf"
    ET.SubElement(config, "description").text = "Combined model autogenerated from IFC"
    raw_string = ET.tostring(config, encoding="unicode")
    parsed_dom = xml.dom.minidom.parseString(raw_string)
    pretty_string = parsed_dom.toprettyxml(indent="    ")
    config_path = os.path.join(output_folder, "model.config")
    with open(config_path, "w") as f:
        f.write(pretty_string)
    return config_path

def create_world_file(world_name, model_name):
    package_path = get_package_path(PACKAGE_NAME)
    worlds_folder_path = os.path.join(package_path, "worlds")
    os.makedirs(worlds_folder_path, exist_ok=True)
    sdf = ET.Element("sdf", version="1.6")
    world = ET.SubElement(sdf, "world", name=world_name)
    include_ground = ET.SubElement(world, "include")
    ET.SubElement(include_ground, "uri").text = "model://ground_plane"
    include_sun = ET.SubElement(world, "include")
    ET.SubElement(include_sun, "uri").text = "model://sun"
    include_model = ET.SubElement(world, "include")
    ET.SubElement(include_model, "uri").text = f"model://{model_name}"
    pose = ET.SubElement(include_model, "pose")
    pose.text = "0 0 0 0 0 0"
    raw_string = ET.tostring(sdf, encoding="unicode")
    parsed_dom = xml.dom.minidom.parseString(raw_string)
    pretty_string = parsed_dom.toprettyxml(indent="    ")
    world_path = os.path.join(worlds_folder_path, f"{world_name}.world")
    with open(world_path, "w") as f:
        f.write(pretty_string)
    return world_path


if __name__ == "__main__":
    rospy.init_node('ifc2world')

    # Obtener ruta del IFC a partir del parámetro 'map'
    ifc_file_path = os.path.join(get_package_path(PACKAGE_NAME), 'models/', rospy.get_param('/bim/map'))
    ifc_file, settings = setup_ifc_geometry(ifc_file_path)

    model_name = rospy.get_param('/bim/map').replace('.ifc', '')
    base_path = os.path.join(get_package_path(PACKAGE_NAME), 'models/', model_name)
    meshes_path = os.path.join(base_path, "meshes")
    os.makedirs(meshes_path, exist_ok=True)

    (global_id_to_int, movable_dict, non_movable_dict, link_poses) = process_ifc_file(ifc_file, settings)
    save_results(global_id_to_int)

    print("\nElementos movibles:")
    for global_id in movable_dict.keys():
        print(f"- {global_id}  (ID asignado: {global_id_to_int.get(global_id)})")
    print("\nElementos no movibles:")
    for global_id in non_movable_dict.keys():
        print(f"- {global_id}  (ID asignado: {global_id_to_int.get(global_id)})")


    verts_path = os.path.join(get_package_path(PACKAGE_NAME), 'grids')
    movables = set(movable_dict.keys())
    
    seen_ids=export_elements_and_dae(ifc_file_path, meshes_path,  movables)
    create_combined_sdf(model_name, seen_ids, base_path, verts_path, movable_ids=movables, link_poses=link_poses)
    create_model_config(model_name, base_path)
    world_file = create_world_file(model_name, model_name)
    print(f"\033[92mWorld file created at: {world_file}\033[0m")
    print("\033[92mSingle combined model export complete.\033[0m")
