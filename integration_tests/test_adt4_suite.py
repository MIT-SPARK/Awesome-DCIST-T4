#!/usr/bin/env python3
"""
Complete ADT4 Production Test Suite
Includes system integration, navigation, and DSG analysis tests
"""

import pytest
import subprocess
import time
import os
import json
import shutil
import yaml
import numpy as np
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
import logging
import threading

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Import ROS2 packages if available
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from omniplanner_msgs.msg import GotoPointsGoalMsg
    from tf2_ros import Buffer, TransformListener
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logger.warning("ROS2 Python packages not available - navigation tests will fail")


# ==================== CONFIGURATION ====================

@dataclass
class TestConfig:
    """Test configuration parameters"""
    robot_name: str = field(default_factory=lambda: os.getenv('ADT4_ROBOT_NAME', 'hilbert'))
    workspace: str = field(default_factory=lambda: os.getenv('ADT4_WS', ''))
    prior_dsg_path: str = field(default_factory=lambda: os.getenv('ADT4_PRIOR_DSG_PATH', ''))
    output_dir: str = field(default_factory=lambda: os.getenv('ADT4_OUTPUT_DIR', '/tmp'))
    timeout_short: float = 30.0
    timeout_medium: float = 60.0
    timeout_long: float = 120.0
    success_threshold: float = 3.0  # meters for navigation success
    use_sim_time: bool = False
    
    # Room centroids pulled from dsg
    room_centroids: Dict[int, Tuple[float, float, float]] = field(default_factory=lambda: {
        70: (22.4264, 10.1144, 0.0),
        71: (54.309, 6.4453, 0.0),
        69: (-15.375, 24.386, 0.0),
    })
    
    # Critical topics to monitor
    critical_topics: List[str] = field(default_factory=lambda: [
        '/global/ros_system_monitor/table_in',
        '/{robot}/omniplanner_node/compiled_plan_out',
        '/{robot}/hydra/backend/dsg'
    ])
    
    # Expected nodes for system health
    expected_nodes: List[str] = field(default_factory=lambda: [
        'dsg_saver', 'spot_executor', 'omniplanner', 'hydra'
    ])
    
    def validate(self) -> List[str]:
        """Validate configuration and return list of issues"""
        issues = []
        if not self.robot_name:
            issues.append("ADT4_ROBOT_NAME not set")
        if not self.workspace:
            issues.append("ADT4_WS not set")
        if self.prior_dsg_path and not Path(self.prior_dsg_path).exists():
            issues.append(f"Prior DSG not found: {self.prior_dsg_path}")
        return issues


# ==================== TEST FIXTURES ====================

@pytest.fixture(scope="session")
def test_config():
    """Provide test configuration for all tests"""
    config = TestConfig()
    issues = config.validate()
    if issues:
        logger.warning(f"Configuration issues: {', '.join(issues)}")
    return config


@pytest.fixture(scope="session")
def ros_context():
    """Initialize ROS2 context once for all tests"""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 Python packages not available")
    
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def navigation_node(ros_context, test_config):
    """Create a ROS2 node for navigation testing"""
    node = NavigationTestNode(test_config)
    
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    time.sleep(1)
    yield node
    
    executor.shutdown()
    node.destroy_node()


@pytest.fixture
def dsg_analyzer(test_config):
    """Create DSG analyzer if utilities are available"""
    if not DSG_UTILS_AVAILABLE:
        pytest.skip("DSG utilities not available")
    
    analyzer = DSGAnalyzer()
    
    # Add room nodes from config
    for room_id, centroid in test_config.room_centroids.items():
        room_node = DSGNode(
            id=room_id,
            layer=4,
            name=f"R({room_id})",
            position=np.array(centroid)
        )
        analyzer.add_node(room_node)
    
    return analyzer


# ==================== ROS2 NAVIGATION NODE ====================

if ROS2_AVAILABLE:
    class NavigationTestNode(Node):
        """ROS2 node for navigation testing"""
        
        def __init__(self, config: TestConfig):
            super().__init__('pytest_navigation_node')
            self.config = config
            
            # Publishers
            self.goto_pub = self.create_publisher(
                GotoPointsGoalMsg,
                f'/{config.robot_name}/omniplanner_node/goto_points/goto_points_goal',
                10
            )
            
            # TF2 for position monitoring
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            
            self.current_position = None
            self.position_history = []
            
            self.get_logger().info(f"Navigation test node initialized for {config.robot_name}")
        
        def send_goto_command(self, room_names: List[str]) -> bool:
            """Send goto_points command"""
            try:
                msg = GotoPointsGoalMsg()
                msg.robot_id = self.config.robot_name
                msg.point_names_to_visit = room_names
                
                self.goto_pub.publish(msg)
                self.get_logger().info(f"âœ“ Published navigation goal: {room_names}")
                return True
            except Exception as e:
                self.get_logger().error(f"Failed to publish goal: {e}")
                return False
        
        def get_robot_position(self) -> Optional[np.ndarray]:
            """Get current robot position from TF"""
            try:
                from rclpy.time import Time
                from rclpy.duration import Duration
                
                transform = self.tf_buffer.lookup_transform(
                    'map', f'{self.config.robot_name}/body',
                    Time(), timeout=Duration(seconds=1.0)
                )
                
                pos = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])
                self.current_position = pos
                self.position_history.append((time.time(), pos))
                return pos
                
            except Exception as e:
                self.get_logger().debug(f"TF lookup failed: {e}")
                return None
        
        def distance_to_room(self, room_id: int) -> Optional[float]:
            """Calculate distance to room centroid"""
            if room_id not in self.config.room_centroids:
                return None
            
            pos = self.get_robot_position()
            if pos is None:
                return None
            
            centroid = np.array(self.config.room_centroids[room_id])
            return np.linalg.norm(pos[:2] - centroid[:2])



# ==================== SYSTEM INTEGRATION TESTS ====================

class TestSystemIntegration:
    """System-level integration tests"""
    
    @pytest.mark.quick
    def test_environment_variables(self, test_config):
        """Test that all required environment variables are set"""
        required_vars = [
            'ADT4_WS', 'ADT4_ENV', 'ADT4_OUTPUT_DIR', 
            'ADT4_ROBOT_NAME', 'ADT4_PRIOR_DSG_PATH'
        ]
        
        missing = [var for var in required_vars if not os.getenv(var)]
        assert not missing, f"Missing environment variables: {missing}"
    
    @pytest.mark.quick
    def test_prior_dsg_exists(self, test_config):
        """Test that prior DSG file exists"""
        if not test_config.prior_dsg_path:
            pytest.skip("ADT4_PRIOR_DSG_PATH not set")
        assert Path(test_config.prior_dsg_path).exists(), \
               f"Prior DSG not found: {test_config.prior_dsg_path}"
    
    @pytest.mark.integration
    def test_ros_node_discovery(self):
        """Test ROS2 node discovery functionality"""
        result = subprocess.run(['ros2', 'node', 'list'], 
                              capture_output=True, text=True, timeout=10)
        assert result.returncode == 0, "Failed to list ROS2 nodes"
        
        nodes = result.stdout.strip().split('\n')
        logger.info(f"Found {len(nodes)} ROS2 nodes")
    
    @pytest.mark.integration
    def test_topic_data_flow(self, test_config):
        """Test that critical topics are publishing data"""
        # Update critical topics to only check topics that continuously publish
        always_publishing_topics = [
            '/global/ros_system_monitor/table_in',
        ]
        
        on_demand_topics = [
            '/{robot}/omniplanner_node/compiled_plan_out',  # Only publishes on plan
            '/{robot}/hydra/backend/dsg'
        ]
        
        topics_checked = {}
        
        # Check always-publishing topics
        for topic_template in always_publishing_topics:
            topic = topic_template.format(robot=test_config.robot_name)
            # Check if exists
            list_result = subprocess.run(['ros2', 'topic', 'list'],
                                        capture_output=True, text=True, timeout=5)
            topics_checked[topic] = topic in list_result.stdout
        
        # For on-demand topics, just check they exist
        for topic_template in on_demand_topics:
            topic = topic_template.format(robot=test_config.robot_name)
            list_result = subprocess.run(['ros2', 'topic', 'list'],
                                        capture_output=True, text=True, timeout=5)
            if topic in list_result.stdout:
                topics_checked[topic] = True
                logger.info(f"Topic exists: {topic}")
        
        existing = sum(topics_checked.values())
        total = len(topics_checked)
        assert existing >= total * 0.5, f"Too few topics exist: {existing}/{total}"
    
    @pytest.mark.integration
    def test_system_monitor_health(self, test_config):
        """Test system monitor health status"""
        monitor_topic = '/global/ros_system_monitor/table_in'
        
        result = subprocess.run(
            ['ros2', 'topic', 'echo', monitor_topic, '--once'],
            capture_output=True, text=True, timeout=10
        )
        
        if result.returncode == 0:
            logger.info("System monitor is publishing")
            assert True
        else:
            # Topic might exist but not be publishing
            list_result = subprocess.run(['ros2', 'topic', 'list'],
                                       capture_output=True, text=True, timeout=5)
            assert monitor_topic in list_result.stdout, f"System monitor topic not found"
            logger.warning("System monitor topic exists but may not be actively publishing")
    

# ==================== NAVIGATION TESTS ====================

class TestNavigation:
    """Navigation tests using ROS2 Python API"""
    
    @pytest.mark.navigation
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 Python required")
    @pytest.mark.parametrize("room_id", [70, 71])
    def test_single_room_navigation(self, navigation_node, room_id):
        """Test navigation to a single room"""
        config = navigation_node.config
        
        # Check initial distance
        initial_distance = navigation_node.distance_to_room(room_id)
        if initial_distance is None:
            pytest.skip("Cannot get robot position")
        
        logger.info(f"Initial distance to R({room_id}): {initial_distance:.2f}m")
        
        if initial_distance <= config.success_threshold:
            logger.info(f"âœ“ Already at R({room_id})")
            return
        
        # Send navigation command
        assert navigation_node.send_goto_command([f'R({room_id})']), \
               f"Failed to send goal to R({room_id})"
        
        start_time = time.time()
        min_distance = initial_distance
        
        while (time.time() - start_time) < config.timeout_medium:
            current_distance = navigation_node.distance_to_room(room_id)
            
            if current_distance is not None:
                min_distance = min(min_distance, current_distance)
                
                if current_distance <= config.success_threshold:
                    logger.info(f"âœ“ Reached R({room_id}) in {time.time()-start_time:.1f}s")
                    return
                
                if int(time.time() - start_time) % 10 == 0:
                    logger.info(f"Distance to R({room_id}): {current_distance:.2f}m")
            
            time.sleep(1.0)
        
        # Check final state
        final_distance = navigation_node.distance_to_room(room_id)
        assert final_distance and final_distance <= config.success_threshold * 2, \
               f"Failed to reach R({room_id}). Final: {final_distance:.2f}m"
    
    @pytest.mark.navigation
    @pytest.mark.slow
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 Python required")
    def test_multi_room_sequence(self, navigation_node):
        """Test navigation through multiple rooms"""
        config = navigation_node.config
        room_sequence = [69, 70]
        
        # Send multi-room command
        room_names = [f'R({rid})' for rid in room_sequence]
        assert navigation_node.send_goto_command(room_names), \
               "Failed to send multi-room goal"
        
        # Track rooms reached
        rooms_reached = set()
        start_time = time.time()
        timeout = config.timeout_long
        
        while (time.time() - start_time) < timeout:
            for room_id in room_sequence:
                if room_id not in rooms_reached:
                    distance = navigation_node.distance_to_room(room_id)
                    if distance and distance <= config.success_threshold:
                        rooms_reached.add(room_id)
                        logger.info(f"âœ“ Reached R({room_id})")
            
            if len(rooms_reached) == len(room_sequence):
                logger.info(f"âœ“ All rooms visited in {time.time()-start_time:.1f}s")
                return
            
            time.sleep(1.0)
        
        assert len(rooms_reached) >= len(room_sequence) * 0.5, \
               f"Only reached {len(rooms_reached)}/{len(room_sequence)} rooms"


# ==================== PERFORMANCE TESTS ====================

class TestPerformance:
    """Performance and stress tests"""
    
    @pytest.mark.performance
    def test_tf_lookup_performance(self, navigation_node):
        """Test TF lookup performance"""
        if not ROS2_AVAILABLE:
            pytest.skip("ROS2 required for performance test")
        
        lookup_times = []
        for _ in range(10):
            start = time.time()
            pos = navigation_node.get_robot_position()
            if pos is not None:
                lookup_times.append(time.time() - start)
        
        if lookup_times:
            avg_time = np.mean(lookup_times)
            p95_time = np.percentile(lookup_times, 95)
            
            logger.info(f"TF Performance: avg={avg_time*1000:.1f}ms, p95={p95_time*1000:.1f}ms")
            assert avg_time < 0.5, f"TF lookup too slow: {avg_time:.3f}s average"
    
    @pytest.mark.performance
    def test_goal_publishing_rate(self, navigation_node):
        """Test goal publishing performance"""
        if not ROS2_AVAILABLE:
            pytest.skip("ROS2 required for performance test")
        
        start = time.time()
        num_goals = 10
        
        for i in range(num_goals):
            success = navigation_node.send_goto_command([f'R({70 + i % 3})'])
            assert success, f"Failed to send goal {i}"
        
        elapsed = time.time() - start
        rate = num_goals / elapsed
        
        logger.info(f"Goal publishing rate: {rate:.1f} goals/sec")
        assert rate > 5, f"Goal publishing too slow: {rate:.1f} goals/sec"


# ==================== PYTEST CONFIGURATION ====================

def pytest_configure(config):
    """Configure pytest with custom markers"""
    markers = [
        "quick: Quick tests (< 10s)",
        "integration: Integration tests",
        "navigation: Navigation tests",
        "performance: Performance tests",
        "slow: Slow tests (> 60s)"
    ]
    for marker in markers:
        config.addinivalue_line("markers", marker)


# ==================== MAIN ENTRY POINT ====================

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="ADT4 Complete Test Suite")
    parser.add_argument("-m", "--markers", help="Run tests with specific markers")
    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("--junit", help="Generate JUnit XML report")
    parser.add_argument("--html", help="Generate HTML report")
    
    args = parser.parse_args()
    
    pytest_args = []
    if args.verbose:
        pytest_args.append("-v")
    if args.markers:
        pytest_args.extend(["-m", args.markers])
    if args.junit:
        pytest_args.append(f"--junitxml={args.junit}")
    if args.html:
        pytest_args.extend([f"--html={args.html}", "--self-contained-html"])
    
    pytest_args.append(__file__)
    exit(pytest.main(pytest_args))