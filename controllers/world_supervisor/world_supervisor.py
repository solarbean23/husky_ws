#!/usr/bin/env python3
"""
World Supervisor
- Fire / RescueZone / Base 등 world object를 관리
- 동적으로 object spawn/remove 지원
- pose publish
"""

from controller import Supervisor
import math
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
    RESCUE_ZONE = "Rescue_Zone"
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
        radius 2
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
  ObjectCategory.RESCUE_ZONE: """
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
          url [ "icons/rescue_red.png" ]
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
    """axis-angle을 quaternion으로 변환"""
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
    """
    Fire, RescueZone 등 동적으로 생성/삭제 가능한 object를 관리
    - Fire_1, Fire_2 삭제 후 새로 생성하면 Fire_3이 됨
    """
    def __init__(self, supervisor: Supervisor, category: str, def_prefix: str):
        self.supervisor = supervisor
        self.category = category
        self.def_prefix = def_prefix  # e.g., "Fire_", "Rescue_Zone_"
        
        self.active_objects = {}  # def_name -> webots_node
        self.next_index = 1  # 다음 생성 시 사용할 인덱스
        
    def scan_existing_objects(self):
        """
        world에 이미 존재하는 object들을 스캔하여 등록
        Fire_1, Fire_2 등이 있으면 next_index를 그에 맞게 설정
        """
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
                # e.g., "Fire_1" -> index = 1
                try:
                    index_str = def_name[len(self.def_prefix):]
                    index = int(index_str)
                    self.active_objects[def_name] = child_node
                    max_found_index = max(max_found_index, index)
                except ValueError:
                    # 숫자가 아닌 경우 무시
                    pass
        
        self.next_index = max_found_index + 1
        return list(self.active_objects.keys())
    
    def spawn_object(self, position_x: float, position_y: float, position_z: float) -> str:
        """
        새 object를 world에 생성하고 def_name 반환
        """
        def_name = f"{self.def_prefix}{self.next_index}"
        self.next_index += 1
        
        template = PROTO_TEMPLATES.get(self.category)
        if template is None:
            return None
        
        proto_string = template.format(
            def_name=def_name,
            x=position_x,
            y=position_y,
            z=position_z
        )
        
        root = self.supervisor.getRoot()
        children_field = root.getField("children")
        children_field.importMFNodeFromString(-1, proto_string)
        
        # 새로 생성된 노드 찾기
        new_node = self.supervisor.getFromDef(def_name)
        if new_node:
            self.active_objects[def_name] = new_node
            return def_name
        
        return None
    
    def remove_object(self, def_name: str) -> bool:
        """
        object를 world에서 삭제
        """
        if def_name not in self.active_objects:
            return False
        
        webots_node = self.active_objects[def_name]
        if webots_node:
            webots_node.remove()
        
        del self.active_objects[def_name]
        return True
    
    def get_active_object_names(self):
        """현재 활성화된 object def_name 목록 반환"""
        return list(self.active_objects.keys())
    
    def get_webots_node(self, def_name: str):
        """def_name에 해당하는 webots node 반환"""
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
        self.publish_rate = 20.0  # Hz

        # ====== Object Managers ======
        self.fire_manager = DynamicObjectManager(
            supervisor, ObjectCategory.FIRE, "Fire_"
        )
        self.rescue_zone_manager = DynamicObjectManager(
            supervisor, ObjectCategory.RESCUE_ZONE, "Rescue_Zone_"
        )
        self.water_manager = DynamicObjectManager(
            supervisor, ObjectCategory.WATER, "Water_"
        )
        
        # Base는 정적 object (위치 정보만 publish)
        self.base_node = None
        self.base_def_name = "Base"
        
        # ====== 초기 object 스캔 ======
        self._scan_initial_objects()
        
        # ====== publishers ======
        self.fire_publishers = {}
        self.fire_radius_publishers = {}
        self.rescue_zone_publishers = {}
        self.water_publishers = {}
        self.base_publisher = None
        
        self._create_initial_publishers()
        
        # ====== services ======
        self.remove_services = {}
        self.spawn_services = {}
        
        self._create_spawn_services()
        self._create_initial_remove_services()

        self.last_publish_time = self.get_clock().now()

        self.get_logger().info("WorldSupervisor ready")
        self.get_logger().info(f"  Fires: {self.fire_manager.get_active_object_names()}")
        self.get_logger().info(f"  RescueZones: {self.rescue_zone_manager.get_active_object_names()}")
        self.get_logger().info(f"  Waters: {self.water_manager.get_active_object_names()}")
        self.get_logger().info(f"  Base: {self.base_def_name if self.base_node else 'Not found'}")

    # -----------------------------
    def _scan_initial_objects(self):
        """world에 존재하는 object들을 스캔"""
        self.fire_manager.scan_existing_objects()
        self.rescue_zone_manager.scan_existing_objects()
        self.water_manager.scan_existing_objects()
        
        # Base 스캔
        self.base_node = self.supervisor.getFromDef(self.base_def_name)
        if self.base_node is None:
            self.get_logger().warn(f"Base DEF not found: {self.base_def_name}")

    # -----------------------------
    def _create_initial_publishers(self):
        """초기 object들에 대한 publisher 생성"""
        # Fire publishers
        for def_name in self.fire_manager.get_active_object_names():
            self._create_fire_publisher(def_name)
        
        # RescueZone publishers
        for def_name in self.rescue_zone_manager.get_active_object_names():
            self._create_rescue_zone_publisher(def_name)
        
        # Water publishers
        for def_name in self.water_manager.get_active_object_names():
            self._create_water_publisher(def_name)
        
        # Base publisher
        if self.base_node:
            self.base_publisher = self.create_publisher(
                PoseStamped, "/world/base/pose", 1
            )
    
    def _create_fire_publisher(self, def_name: str):
        """Fire object에 대한 publisher 생성"""
        topic_name = f"/world/fire/{def_name}/pose"
        self.fire_publishers[def_name] = self.create_publisher(
            PoseStamped, topic_name, 1
        )
        self.get_logger().info(f"Created publisher: {topic_name}")
        
        # radius publisher도 생성
        radius_topic_name = f"/world/fire/{def_name}/radius"
        self.fire_radius_publishers[def_name] = self.create_publisher(
            Float32, radius_topic_name, 1
        )
        self.get_logger().info(f"Created radius publisher: {radius_topic_name}")
    
    def _create_rescue_zone_publisher(self, def_name: str):
        """RescueZone object에 대한 publisher 생성"""
        topic_name = f"/world/rescue_zone/{def_name}/pose"
        self.rescue_zone_publishers[def_name] = self.create_publisher(
            PoseStamped, topic_name, 1
        )
        self.get_logger().info(f"Created publisher: {topic_name}")
    
    def _create_water_publisher(self, def_name: str):
        """Water object에 대한 publisher 생성"""
        topic_name = f"/world/water/{def_name}/pose"
        self.water_publishers[def_name] = self.create_publisher(
            PoseStamped, topic_name, 1
        )
        self.get_logger().info(f"Created publisher: {topic_name}")

    # -----------------------------
    def _create_spawn_services(self):
        """Spawn 서비스 생성 (Fire, RescueZone)"""
        # Fire spawn service
        self.create_service(
            Empty,
            "/world/fire/spawn",
            self._handle_spawn_fire
        )
        
        # RescueZone spawn service
        self.create_service(
            Empty,
            "/world/rescue_zone/spawn",
            self._handle_spawn_rescue_zone
        )
    
    def _handle_spawn_fire(self, request, response):
        """Fire spawn 서비스 핸들러 (기본 위치에 생성)"""
        # TODO: 추후 위치 파라미터 받을 수 있도록 커스텀 서비스로 변경 가능
        default_x, default_y, default_z = 0.0, 0.0, 0.0
        
        def_name = self.fire_manager.spawn_object(default_x, default_y, default_z)
        if def_name:
            self._create_fire_publisher(def_name)
            self._create_suppress_service_for_fire(def_name)
            self.get_logger().info(f"Spawned {def_name} at ({default_x}, {default_y}, {default_z})")
        else:
            self.get_logger().error("Failed to spawn Fire")
        
        return response
    
    def _handle_spawn_rescue_zone(self, request, response):
        """RescueZone spawn 서비스 핸들러 (기본 위치에 생성)"""
        default_x, default_y, default_z = 0.0, 0.0, 0.0
        
        def_name = self.rescue_zone_manager.spawn_object(default_x, default_y, default_z)
        if def_name:
            self._create_rescue_zone_publisher(def_name)
            self._create_complete_service_for_rescue_zone(def_name)
            self.get_logger().info(f"Spawned {def_name} at ({default_x}, {default_y}, {default_z})")
        else:
            self.get_logger().error("Failed to spawn RescueZone")
        
        return response

    # -----------------------------
    def _create_initial_remove_services(self):
        """초기 object들에 대한 remove 서비스 생성"""
        for def_name in self.fire_manager.get_active_object_names():
            self._create_suppress_service_for_fire(def_name)
        
        for def_name in self.rescue_zone_manager.get_active_object_names():
            self._create_complete_service_for_rescue_zone(def_name)
    
    def _create_suppress_service_for_fire(self, def_name: str):
        """Fire object에 대한 suppress 서비스 생성"""
        service_name = f"/world/fire/{def_name}/suppress"
        service = self.create_service(
            Empty,
            service_name,
            self._make_fire_suppress_callback(def_name)
        )
        self.remove_services[def_name] = service
        self.get_logger().info(f"Created suppress service: {service_name}")
    
    def _create_complete_service_for_rescue_zone(self, def_name: str):
        """RescueZone object에 대한 complete 서비스 생성"""
        service_name = f"/world/rescue_zone/{def_name}/complete"
        service = self.create_service(
            Empty,
            service_name,
            self._make_rescue_zone_complete_callback(def_name)
        )
        self.remove_services[def_name] = service
        self.get_logger().info(f"Created complete service: {service_name}")
    
    def _make_fire_suppress_callback(self, def_name: str):
        """Fire suppress 서비스 콜백 생성"""
        def callback(request, response):
            success = self.fire_manager.remove_object(def_name)
            if success:
                publisher = self.fire_publishers.pop(def_name, None)
                if publisher is not None:
                    self.destroy_publisher(publisher)

                # radius publisher도 제거
                radius_publisher = self.fire_radius_publishers.pop(def_name, None)
                if radius_publisher is not None:
                    self.destroy_publisher(radius_publisher)

                service = self.remove_services.pop(def_name, None)
                if service is not None:
                    self.destroy_service(service)

                self.get_logger().info(f"{def_name} suppressed from world")
            else:
                self.get_logger().info(f"{def_name} already suppressed or not found")
            return response
        return callback
    
    def _make_rescue_zone_complete_callback(self, def_name: str):
        """RescueZone complete 서비스 콜백 생성"""
        def callback(request, response):
            success = self.rescue_zone_manager.remove_object(def_name)
            if success:
                if def_name in self.rescue_zone_publishers:
                    self.destroy_publisher(self.rescue_zone_publishers[def_name])
                    del self.rescue_zone_publishers[def_name]
                self.get_logger().info(f"{def_name} complete from world")
            else:
                self.get_logger().info(f"{def_name} already completed or not found")
            return response
        return callback

    # -----------------------------
    def _read_fire_radius(self, webots_node):
        """Fire webots node에서 radius 읽기"""
        try:
            # Fire 구조: Pose -> children[0] Shape -> geometry Sphere -> radius
            children_field = webots_node.getField("children")
            if children_field.getCount() > 0:
                shape_node = children_field.getMFNode(0)  # 첫 번째 Shape
                if shape_node.getTypeName() == "Shape":
                    geometry_field = shape_node.getField("geometry")
                    geometry_node = geometry_field.getSFNode()
                    if geometry_node.getTypeName() == "Sphere":
                        radius_field = geometry_node.getField("radius")
                        return radius_field.getSFFloat()
        except Exception as e:
            self.get_logger().warn(f"Failed to read radius from {webots_node.getDef()}: {e}")
        
        # 기본값 반환
        return 2.0

    def _read_pose(self, webots_node):
        """webots node에서 pose 읽어서 PoseStamped 메시지로 변환"""
        translation = webots_node.getField("translation").getSFVec3f()
        rotation = webots_node.getField("rotation").getSFRotation()
        quaternion = axis_angle_to_quaternion(rotation[0], rotation[1], rotation[2], rotation[3])

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.pose.position.x = translation[0]
        message.pose.position.y = translation[1]
        message.pose.position.z = translation[2]
        message.pose.orientation.x = quaternion[0]
        message.pose.orientation.y = quaternion[1]
        message.pose.orientation.z = quaternion[2]
        message.pose.orientation.w = quaternion[3]
        return message

    # -----------------------------
    def publish_if_needed(self):
        """주기에 맞춰 pose publish"""
        now = self.get_clock().now()
        elapsed_nanoseconds = (now - self.last_publish_time).nanoseconds
        if elapsed_nanoseconds < 1e9 / self.publish_rate:
            return
        self.last_publish_time = now

        # Fire poses and radius
        for def_name, publisher in list(self.fire_publishers.items()):
            webots_node = self.fire_manager.get_webots_node(def_name)
            if webots_node:
                publisher.publish(self._read_pose(webots_node))
                
                # 실제 radius 값 읽어서 publish
                radius_publisher = self.fire_radius_publishers.get(def_name)
                if radius_publisher:
                    radius_value = self._read_fire_radius(webots_node)
                    radius_msg = Float32()
                    radius_msg.data = radius_value
                    radius_publisher.publish(radius_msg)

        # RescueZone poses
        for def_name, publisher in list(self.rescue_zone_publishers.items()):
            webots_node = self.rescue_zone_manager.get_webots_node(def_name)
            if webots_node:
                publisher.publish(self._read_pose(webots_node))

        # Water poses
        for def_name, publisher in list(self.water_publishers.items()):
            webots_node = self.water_manager.get_webots_node(def_name)
            if webots_node:
                publisher.publish(self._read_pose(webots_node))

        # Base pose
        if self.base_node and self.base_publisher:
            self.base_publisher.publish(self._read_pose(self.base_node))


# -----------------------------
# main loop
# -----------------------------
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

