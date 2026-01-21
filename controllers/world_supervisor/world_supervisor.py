#!/usr/bin/env python3
"""
World Supervisor
- Fire / Target / Base 등 world object를 관리
- 동적으로 object spawn/remove 지원
- pose publish
"""

from controller import Supervisor
import math
import random  # [추가] 랜덤 생성을 위해 import
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from std_msgs.msg import Float32


# -----------------------------
# Object Category 정의
# -----------------------------
class ObjectCategory:
    FIRE = "Fire"
    TARGET = "Target"
    WATER = "Water"
    BASE = "Base"

# PROTO 템플릿 (spawn 시 사용)
PROTO_TEMPLATES = {
  ObjectCategory.FIRE: """
DEF {def_name} Pose {{
  translation {x} {y} {z}
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor 0.878431 0.105882 0.141176
          emissiveColor 0.647059 0.113725 0.176471
          shininess 0.5
          specularColor 0.878431 0.105882 0.141176
          transparency 0.7
        }}
      }}
      geometry Sphere {{
        radius {radius}  # [수정] 고정값 2 -> {radius} 변수로 변경
        subdivision 3
      }}
    }}
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor 0.878431 0.105882 0.141176
          emissiveColor 0.752941 0.109804 0.156863
        }}
      }}
      geometry Sphere {{
        radius 0.1
        subdivision 3
      }}
    }}
  ]
}}
""",
  ObjectCategory.TARGET: """
DEF {def_name} Pose {{
  translation {x} {y} {z}
  rotation 0 0 1 -1.57
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor 1 1 1
          emissiveColor 1 1 1
        }}
        texture ImageTexture {{
          url [ "icons/target.png" ]
        }}
        textureTransform TextureTransform {{
        }}
      }}
      geometry Plane {{
        size 1.5 1.5
      }}
    }}
  ]
}}
"""
}

# -----------------------------
# util
# -----------------------------
def axis_angle_to_quaternion(axis_x, axis_y, axis_z, angle):
    norm = math.sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z)
    if norm < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    axis_x, axis_y, axis_z = axis_x / norm, axis_y / norm, axis_z / norm
    sin_half = math.sin(angle * 0.5)
    return (axis_x * sin_half, axis_y * sin_half, axis_z * sin_half, math.cos(angle * 0.5))


# -----------------------------
# 동적 Object 관리 클래스
# -----------------------------
class DynamicObjectManager:
    def __init__(self, supervisor: Supervisor, category: str, def_prefix: str):
        self.supervisor = supervisor
        self.category = category
        self.def_prefix = def_prefix
        
        self.active_objects = {}
        self.next_index = 1
        
    def scan_existing_objects(self):
        root = self.supervisor.getRoot()
        children_field = root.getField("children")
        children_count = children_field.getCount()
        
        max_found_index = 0
        
        for i in range(children_count):
            child_node = children_field.getMFNode(i)
            if child_node is None:
                continue
            
            def_name = child_node.getDef()
            if def_name and def_name.startswith(self.def_prefix):
                try:
                    index_str = def_name[len(self.def_prefix):]
                    index = int(index_str)
                    self.active_objects[def_name] = child_node
                    max_found_index = max(max_found_index, index)
                except ValueError:
                    pass
        
        # [설명] 여기서 max_found_index + 1을 하므로,
        # Fire_2가 삭제되고 Fire_1, Fire_3만 있어도 max는 3이 되어 next_index는 4가 됩니다.
        self.next_index = max_found_index + 1
        return list(self.active_objects.keys())
    
    # [수정] **kwargs 추가: radius 등 추가 파라미터를 받기 위함
    def spawn_object(self, position_x: float, position_y: float, position_z: float, **kwargs) -> str:
        def_name = f"{self.def_prefix}{self.next_index}"
        self.next_index += 1
        
        template = PROTO_TEMPLATES.get(self.category)
        if template is None:
            return None
        
        # [수정] format에 **kwargs 전달
        # Fire인 경우 kwargs에 'radius'가 포함되어야 함
        try:
            proto_string = template.format(
                def_name=def_name,
                x=position_x,
                y=position_y,
                z=position_z,
                **kwargs 
            )
        except KeyError as e:
            # 템플릿에 필요한 인자가 안 넘어온 경우 에러 처리 (예: radius 누락)
            print(f"Error formatting template: Missing key {e}")
            return None
        
        root = self.supervisor.getRoot()
        children_field = root.getField("children")
        children_field.importMFNodeFromString(-1, proto_string)
        
        new_node = self.supervisor.getFromDef(def_name)
        if new_node:
            self.active_objects[def_name] = new_node
            return def_name
        
        return None
    
    def remove_object(self, def_name: str) -> bool:
        if def_name not in self.active_objects:
            return False
        
        webots_node = self.active_objects[def_name]
        if webots_node:
            webots_node.remove()
        
        del self.active_objects[def_name]
        return True
    
    def get_active_object_names(self):
        return list(self.active_objects.keys())
    
    def get_webots_node(self, def_name: str):
        return self.active_objects.get(def_name)


# -----------------------------
# ROS Node
# -----------------------------
class WorldSupervisor(Node):
    def __init__(self, supervisor: Supervisor):
        super().__init__("world_supervisor")

        self.supervisor = supervisor
        self.timestep = int(self.supervisor.getBasicTimeStep())

        self.frame_id = "webots_world"
        self.publish_rate = 20.0

        self.fire_manager = DynamicObjectManager(
            supervisor, ObjectCategory.FIRE, "Fire_"
        )
        self.target_manager = DynamicObjectManager(
            supervisor, ObjectCategory.TARGET, "Target_"
        )
        self.water_manager = DynamicObjectManager(
            supervisor, ObjectCategory.WATER, "Water_"
        )
        
        self.base_node = None
        self.base_def_name = "Base"
        
        self._scan_initial_objects()
        
        self.fire_publishers = {}
        self.fire_radius_publishers = {}
        self.target_publishers = {}
        self.water_publishers = {}
        self.base_publisher = None
        
        self._create_initial_publishers()
        
        self.remove_services = {}
        self.spawn_services = {}
        
        self._create_spawn_services()
        self._create_initial_remove_services()

        # [옵션] 주기적으로 자동 생성하고 싶다면 Timer 사용
        # self.create_timer(5.0, self._auto_spawn_fire_timer_callback)

        self.last_publish_time = self.get_clock().now()
        self.get_logger().info("WorldSupervisor ready")

    # ... (기존 _scan_initial_objects, _create_initial_publishers 등은 동일) ...
    def _scan_initial_objects(self):
        self.fire_manager.scan_existing_objects()
        self.target_manager.scan_existing_objects()
        self.water_manager.scan_existing_objects()
        self.base_node = self.supervisor.getFromDef(self.base_def_name)

    def _create_initial_publishers(self):
        for def_name in self.fire_manager.get_active_object_names():
            self._create_fire_publisher(def_name)
        for def_name in self.target_manager.get_active_object_names():
            self._create_target_publisher(def_name)
        for def_name in self.water_manager.get_active_object_names():
            self._create_water_publisher(def_name)
        if self.base_node:
            self.base_publisher = self.create_publisher(PoseStamped, "/world/base/pose", 1)

    def _create_fire_publisher(self, def_name: str):
        topic_name = f"/world/fire/{def_name}/pose"
        self.fire_publishers[def_name] = self.create_publisher(PoseStamped, topic_name, 1)
        
        radius_topic_name = f"/world/fire/{def_name}/radius"
        self.fire_radius_publishers[def_name] = self.create_publisher(Float32, radius_topic_name, 1)

    def _create_target_publisher(self, def_name: str):
        topic_name = f"/world/target/{def_name}/pose"
        self.target_publishers[def_name] = self.create_publisher(PoseStamped, topic_name, 1)

    def _create_water_publisher(self, def_name: str):
        topic_name = f"/world/water/{def_name}/pose"
        self.water_publishers[def_name] = self.create_publisher(PoseStamped, topic_name, 1)

    def _create_spawn_services(self):
        self.create_service(Empty, "/world/fire/spawn", self._handle_spawn_fire)
        self.create_service(Empty, "/world/target/spawn", self._handle_spawn_target)

    # -----------------------------------------------------
    # [핵심 로직] Fire 랜덤 생성 핸들러
    # -----------------------------------------------------
    def _handle_spawn_fire(self, request, response):
        """
        서비스 호출 시 랜덤 위치/크기의 Fire 생성
        - 위치: 15x15 영역 (x, y: -7.5 ~ 7.5)
        - 크기: radius 0.5 ~ 2.0
        """
        # [수정] 랜덤 좌표 생성 (15x15 영역)
        range_limit = 7.5  # -7.5 ~ 7.5
        rand_x = random.uniform(-range_limit, range_limit)
        rand_y = random.uniform(-range_limit, range_limit)
        rand_z = 0.2  # 지면보다 살짝 위

        # [수정] 랜덤 반지름 생성
        rand_radius = random.uniform(0.5, 2.0)
        
        # radius 키워드 인자 전달
        def_name = self.fire_manager.spawn_object(rand_x, rand_y, rand_z, radius=rand_radius)
        
        if def_name:
            self._create_fire_publisher(def_name)
            self._create_suppress_service_for_fire(def_name)
            self.get_logger().info(
                f"Spawned {def_name} at ({rand_x:.2f}, {rand_y:.2f}) with radius {rand_radius:.2f}"
            )
        else:
            self.get_logger().error("Failed to spawn Fire")
        
        return response

    def _handle_spawn_target(self, request, response):
        default_x, default_y, default_z = 2.0, 2.0, 0.01
        def_name = self.target_manager.spawn_object(default_x, default_y, default_z)
        if def_name:
            self._create_target_publisher(def_name)
            self._create_complete_service_for_target(def_name)
            self.get_logger().info(f"Spawned {def_name}")
        return response

    # ... (나머지 서비스 관련 코드, suppress 등 기존과 동일) ...
    def _create_initial_remove_services(self):
        for def_name in self.fire_manager.get_active_object_names():
            self._create_suppress_service_for_fire(def_name)
        for def_name in self.target_manager.get_active_object_names():
            self._create_complete_service_for_target(def_name)
    
    def _create_suppress_service_for_fire(self, def_name: str):
        service_name = f"/world/fire/{def_name}/suppress"
        service = self.create_service(Empty, service_name, self._make_fire_suppress_callback(def_name))
        self.remove_services[def_name] = service
    
    def _create_complete_service_for_target(self, def_name: str):
        service_name = f"/world/target/{def_name}/complete"
        service = self.create_service(Empty, service_name, self._make_target_complete_callback(def_name))
        self.remove_services[def_name] = service

    def _make_fire_suppress_callback(self, def_name: str):
        def callback(request, response):
            success = self.fire_manager.remove_object(def_name)
            if success:
                # 관련 publisher/service 정리
                if def_name in self.fire_publishers:
                    self.destroy_publisher(self.fire_publishers[def_name])
                    del self.fire_publishers[def_name]
                if def_name in self.fire_radius_publishers:
                    self.destroy_publisher(self.fire_radius_publishers[def_name])
                    del self.fire_radius_publishers[def_name]
                if def_name in self.remove_services:
                    self.destroy_service(self.remove_services[def_name])
                    del self.remove_services[def_name]
                self.get_logger().info(f"{def_name} suppressed")
            return response
        return callback

    def _make_target_complete_callback(self, def_name: str):
        def callback(request, response):
            success = self.target_manager.remove_object(def_name)
            if success:
                if def_name in self.target_publishers:
                    self.destroy_publisher(self.target_publishers[def_name])
                    del self.target_publishers[def_name]
                if def_name in self.remove_services:
                    self.destroy_service(self.remove_services[def_name])
                    del self.remove_services[def_name]
            return response
        return callback

    def _read_fire_radius(self, webots_node):
        try:
            children = webots_node.getField("children")
            if children.getCount() > 0:
                shape = children.getMFNode(0)
                geom = shape.getField("geometry").getSFNode()
                if geom.getTypeName() == "Sphere":
                    return geom.getField("radius").getSFFloat()
        except Exception:
            pass
        return 2.0 # fallback

    def _read_pose(self, webots_node):
        trans = webots_node.getField("translation").getSFVec3f()
        rot = webots_node.getField("rotation").getSFRotation()
        quat = axis_angle_to_quaternion(rot[0], rot[1], rot[2], rot[3])
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = trans[0]
        msg.pose.position.y = trans[1]
        msg.pose.position.z = trans[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        return msg

    def publish_if_needed(self):
        now = self.get_clock().now()
        if (now - self.last_publish_time).nanoseconds < 1e9 / self.publish_rate:
            return
        self.last_publish_time = now

        for def_name, pub in list(self.fire_publishers.items()):
            node = self.fire_manager.get_webots_node(def_name)
            if node:
                pub.publish(self._read_pose(node))
                r_pub = self.fire_radius_publishers.get(def_name)
                if r_pub:
                    msg = Float32()
                    msg.data = self._read_fire_radius(node)
                    r_pub.publish(msg)

        for def_name, pub in list(self.target_publishers.items()):
            node = self.target_manager.get_webots_node(def_name)
            if node:
                pub.publish(self._read_pose(node))

        for def_name, pub in list(self.water_publishers.items()):
            node = self.water_manager.get_webots_node(def_name)
            if node:
                pub.publish(self._read_pose(node))
        
        if self.base_node and self.base_publisher:
            self.base_publisher.publish(self._read_pose(self.base_node))

# ... (main 함수는 동일) ...
def main():
    supervisor = Supervisor()
    rclpy.init()
    node = WorldSupervisor(supervisor)
    try:
        while supervisor.step(node.timestep) != -1:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.publish_if_needed()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()