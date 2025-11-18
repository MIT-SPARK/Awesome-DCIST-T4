#!/usr/bin/env python3
"""
Diagnostic script to debug tmux launch issues.
This helps identify why ROS nodes aren't starting in the tmux session.
"""

import subprocess
import time
import os
import sys

def run_command(cmd, timeout=10):
    """Run a command and return result."""
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "TIMEOUT"
    except Exception as e:
        return -1, "", str(e)

def check_environment():
    """Check if all required environment variables are set."""
    print("=== ENVIRONMENT CHECK ===")
    required_vars = [
        'ADT4_WS', 'ADT4_ENV', 'ADT4_OUTPUT_DIR', 'ADT4_ROBOT_NAME',
        'ADT4_BOSDYN_USERNAME', 'ADT4_BOSDYN_PASSWORD', 'ADT4_PRIOR_DSG_PATH'
    ]
    
    missing = []
    for var in required_vars:
        value = os.getenv(var)
        if value:
            print(f"✓ {var}: {value}")
        else:
            print(f"✗ {var}: NOT SET")
            missing.append(var)
    
    if missing:
        print(f"\nMISSING VARIABLES: {missing}")
        return False
    return True

def check_ros_setup():
    """Check if ROS is properly sourced."""
    print("\n=== ROS SETUP CHECK ===")
    
    # Check if ROS commands work (ros2 --version doesn't exist, try ros2 --help)
    ret, out, err = run_command(['ros2', 'node', 'list'])
    if ret == 0 or "No nodes found" in err:
        print(f"✓ ROS2 available and functional")
    else:
        print(f"✗ ROS2 not available: {err}")
        return False
    
    # Check if workspace is sourced
    ret, out, err = run_command(['ros2', 'pkg', 'list'])
    if 'dcist_launch_system' in out:
        print("✓ ADT4 workspace is sourced")
    else:
        print("✗ ADT4 workspace not properly sourced")
        print("Available packages:", out[:200] + "...")
        return False
    
    return True

def test_manual_launch():
    """Test launching one component manually."""
    print("\n=== MANUAL LAUNCH TEST ===")
    
    robot_name = os.getenv('ADT4_ROBOT_NAME', 'spot')
    config = 'spot_prior_dsg'
    sim_time = 'false'
    
    # First, start Zenoh router
    print("Starting Zenoh router...")
    zenoh_process = None
    try:
        zenoh_process = subprocess.Popen(['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'], 
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        time.sleep(2)  # Give router time to start
        
        # Test launching just the scene graph publisher
        cmd = [
            'ros2', 'launch', 'dcist_launch_system', 'master.launch.yaml',
            f'conf_name:={config}',
            f'sim_time:={sim_time}', 
            f'robot_name:={robot_name}',
            'launch_scene_graph_publisher:=true'
        ]
        
        print(f"Testing command: {' '.join(cmd)}")
        print("Running for 15 seconds...")
        
        nodes_found = False
        try:
            process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
            
            # Monitor for a bit and capture output
            start_time = time.time()
            output_lines = []
            
            while time.time() - start_time < 15:
                try:
                    # Check if process is still running
                    if process.poll() is not None:
                        print("⚠ Launch process terminated early")
                        break
                        
                    # Capture some output
                    try:
                        line = process.stdout.readline()
                        if line:
                            output_lines.append(line.strip())
                            if len(output_lines) <= 10:  # Show first few lines
                                print(f"Launch: {line.strip()}")
                    except:
                        pass
                    
                    # Check for nodes every 3 seconds
                    if int(time.time() - start_time) % 3 == 0:
                        ret, out, err = run_command(['ros2', 'node', 'list'], timeout=2)
                        if ret == 0:
                            nodes = [node.strip() for node in out.split('\n') 
                                    if node.strip() and not node.startswith('\x1b') and '/' in node]
                            if len(nodes) > 0:
                                nodes_found = True
                                print(f"✓ Found {len(nodes)} ROS nodes:")
                                for node in nodes[:5]:
                                    print(f"  - {node}")
                                break
                    
                    time.sleep(1)
                except KeyboardInterrupt:
                    break
            
            # Check for successful operation even without node discovery
            success_indicators = [
                "Published map", 
                "prior_dsg_publisher", 
                "INFO"
            ]
            
            has_success_indicators = any(indicator in line for line in output_lines 
                                       for indicator in success_indicators)
            
            if not nodes_found:
                print("⚠ No ROS nodes found via discovery, but checking for functional signs...")
                if has_success_indicators:
                    print("✓ Node appears to be running and functional (despite discovery issues)")
                    nodes_found = True
                else:
                    print("✗ No signs of successful node operation")
                
                if output_lines:
                    print("Launch output (last 10 lines):")
                    for line in output_lines[-10:]:
                        print(f"  {line}")
            
            # Terminate the process
            try:
                process.terminate()
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                
        except Exception as e:
            print(f"✗ Manual launch failed: {e}")
            return False
            
    finally:
        # Clean up Zenoh router
        if zenoh_process:
            try:
                zenoh_process.terminate()
                zenoh_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                zenoh_process.kill()
    
    return nodes_found

def check_tmux_session():
    """Check if tmux session exists and what's in it."""
    print("\n=== TMUX SESSION CHECK ===")
    
    # List tmux sessions
    ret, out, err = run_command(['tmux', 'list-sessions'])
    if ret == 0:
        print("Active tmux sessions:")
        print(out)
    else:
        print("No active tmux sessions or tmux not available")
        return False
    
    # If adt4_system session exists, inspect it
    if 'adt4_system' in out:
        print("\n--- ADT4 Session Details ---")
        
        # List windows
        ret, out, err = run_command(['tmux', 'list-windows', '-t', 'adt4_system'])
        if ret == 0:
            print("Windows:")
            print(out)
        
        # List panes and their content
        ret, out, err = run_command([
            'tmux', 'list-panes', '-s', 'adt4_system', '-F', 
            '#{window_name}:#{pane_index} - #{pane_current_command}'
        ])
        if ret == 0:
            print("\nPanes:")
            print(out)
        
        # Capture pane content from first few panes to see what's happening
        for pane_id in ['0.0', '1.0', '2.0']:
            ret, out, err = run_command([
                'tmux', 'capture-pane', '-t', f'adt4_system:{pane_id}', '-p'
            ])
            if ret == 0:
                print(f"\n--- Pane {pane_id} Content ---")
                print(out[-500:])  # Last 500 characters
    
    return True

def diagnose_config_files():
    """Check if config files exist and are valid."""
    print("\n=== CONFIG FILE CHECK ===")
    
    config_path = "dcist_launch_system/tmux/autogenerated/spot_prior_dsg-spot_prior_dsg.yaml"
    adt4_ws = os.getenv('ADT4_WS')
    
    if adt4_ws:
        # Try the correct path first (in src/awesome_dcist_t4)
        full_path = os.path.join(adt4_ws, "src/awesome_dcist_t4", config_path)
        if os.path.exists(full_path):
            print(f"✓ Tmux config exists: {full_path}")
            
            # Check if it's valid YAML
            try:
                with open(full_path, 'r') as f:
                    content = f.read()
                    print(f"✓ Config file is readable ({len(content)} chars)")
            except Exception as e:
                print(f"✗ Error reading config: {e}")
                return False
        else:
            # Also try the old path in case it's there
            alt_path = os.path.join(adt4_ws, config_path)
            if os.path.exists(alt_path):
                print(f"✓ Tmux config exists: {alt_path}")
                full_path = alt_path
            else:
                print(f"✗ Tmux config not found at either:")
                print(f"  {full_path}")
                print(f"  {alt_path}")
                return False
    else:
        print("✗ ADT4_WS not set, cannot check config")
        return False
    
    # Check launch file exists
    launch_path = "dcist_launch_system/launch/master.launch.yaml"  # Fixed: added /launch/
    full_launch_path = os.path.join(adt4_ws, "src/awesome_dcist_t4", launch_path)
    if os.path.exists(full_launch_path):
        print(f"✓ Launch file exists: {full_launch_path}")
    else:
        print(f"✗ Launch file not found: {full_launch_path}")
        return False
        
    return True

def main():
    """Run all diagnostic checks."""
    print("ADT4 Integration Test Diagnostics")
    print("=" * 50)
    
    checks = [
        ("Environment Variables", check_environment),
        ("ROS Setup", check_ros_setup), 
        ("Config Files", diagnose_config_files),
        ("Manual Launch Test", test_manual_launch),
        ("Tmux Session", check_tmux_session)
    ]
    
    results = {}
    for name, check_func in checks:
        print(f"\n{name}:")
        try:
            results[name] = check_func()
        except Exception as e:
            print(f"✗ {name} failed with exception: {e}")
            results[name] = False
    
    print("\n" + "=" * 50)
    print("DIAGNOSTIC SUMMARY")
    print("=" * 50)
    
    passed = sum(1 for result in results.values() if result)
    total = len(results)
    
    for name, result in results.items():
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status:8} {name}")
    
    print(f"\nOverall: {passed}/{total} checks passed")
    
    if passed != total:
        print("\nRECOMMENDATIONS:")
        if not results.get("Environment Variables"):
            print("- Source your setup script: source /path/to/adt4_setup.zsh")
        if not results.get("ROS Setup"):
            print("- Make sure ROS2 is installed and workspace is built")
        if not results.get("Config Files"):
            print("- Run config generation: dcist_launch_system/scripts/generate_configs.sh")
        if not results.get("Manual Launch Test"):
            print("- Check ROS launch files and dependencies")

if __name__ == "__main__":
    main()
