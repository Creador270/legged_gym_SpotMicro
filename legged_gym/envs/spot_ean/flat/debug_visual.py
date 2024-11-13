import numpy as np
from isaacgym import gymapi
import xml.etree.ElementTree as ET
from legged_gym import LEGGED_GYM_ROOT_DIR

# Inicializar Gym
gym = gymapi.acquire_gym()

# Crear la simulación
sim_params = gymapi.SimParams()
sim_params.dt = 0.01

# set PhysX-specific parameters
sim_params.physx.use_gpu = True

#sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)

#Crear la simulación
sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

# configure the ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 1, 0) # z-up!
plane_params.distance = 0
plane_params.static_friction = 1
plane_params.dynamic_friction = 1
plane_params.restitution = 0

# create the ground plane
gym.add_ground(sim, plane_params)

# Cargar el URDF del robot
asset_options = gymapi.AssetOptions()
asset_options.fix_base_link = False
#asset_options.armature = 0.01
robot_asset = gym.load_asset(sim, "{LEGGED_GYM_ROOT_DIR}/resources/robots/spot_ean/urdf", "spotmicroaiean.urdf", asset_options)

# Crear un entorno y agregar el robot
# spacing = 2.0
# lower = gymapi.Vec3(-spacing, 0.0, -spacing)
# upper = gymapi.Vec3(spacing, spacing, spacing)
# env = gym.create_env(sim, lower, upper, 1)

env = gym.create_env(sim, gymapi.Vec3(-1, -1, 0), gymapi.Vec3(1, 1, 2), 1)
pose = gymapi.Transform()
pose.p = gymapi.Vec3(12, 0.5, 2)
angle = np.radians(270)
pose.r = gymapi.Quat.from_euler_zyx(angle, 0, 0)
robot_handle = gym.create_actor(env, robot_asset, pose, "SpotMicroAI", 0, 1)

# Habilitar la visualización de colisiones
# rigid_shape_props = gym.get_actor_rigid_shape_properties(env, robot_handle)
# for prop in rigid_shape_props:
#     prop.enable_collision_visuals = True
# gym.set_actor_rigid_shape_properties(env, robot_handle, rigid_shape_props)

# Crear un visor para la simulación
viewer = gym.create_viewer(sim, gymapi.CameraProperties())

# Función para calcular el promedio de una lista de tuplas
def average_position(positions):
    avg_pos = np.mean(positions, axis=0)
    return avg_pos

# Simulación y visualización
gym.prepare_sim(sim)
for _ in range(10000):
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # Visualizar la simulación
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)

# Obtener transformaciones de visualización y colisión para debugging
def get_link_names(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    link_names = []
    for link in root.findall('link'):
        name = link.get('name')
        if name:
            link_names.append(name)
    
    return link_names
link_names = get_link_names("{LEGGED_GYM_ROOT_DIR}/resources/robots/spot_ean/urdf/spotmicroaiean.urdf")
print("number of links: ", len(link_names), link_names)
# for link_name in link_names:
#     link_index = gym.find_actor_rigid_body_index(env, robot_handle, link_name, gymapi.DOMAIN_ENV)
#     rigid_body_states = gym.get_actor_rigid_body_states(env, robot_handle, gymapi.STATE_ALL)
#     pos = rigid_body_states[link_index].pose.p
#     rot = rigid_body_states[link_index].pose.r
#     print(f"Link: {link_name}")
#     print(f"Rigid body position: {pos.x}, {pos.y}, {pos.z}")
#     print(f"Rigid body orientation: {rot.x}, {rot.y}, {rot.z}, {rot.w}")

# Cerrar Gym
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)