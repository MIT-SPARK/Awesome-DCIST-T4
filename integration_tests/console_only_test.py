#!/usr/bin/env python3
"""
Simple version that removes tmux logging and keeps everything in the console.
"""

import subprocess
import time
import os
import sys
import logging

# Configure cleaner logging
logging.basicConfig(
    level=logging.INFO, 
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Import the main test class
from prior_dsg_integration_test import PriorDSGIntegrationTest

class ConsoleOnlyIntegrationTest(PriorDSGIntegrationTest):
    """Integration test that only uses console output."""
    
    def _create_test_logging_window(self):
        """Skip tmux window creation."""
        logger.info("Running in console-only mode (no tmux logging window)")
        pass
    
    def _log_to_tmux_window(self, message: str):
        """Skip tmux logging."""
        pass
    
    def _monitor_startup_progress(self):
        """Simplified startup monitoring with less verbose output."""
        max_wait_time = 90
        check_interval = 15  # Check less frequently
        elapsed = 0
        
        print("⏳ Waiting for system startup...", end="", flush=True)
        
        while elapsed < max_wait_time:
            time.sleep(check_interval)
            elapsed += check_interval
            
            print(".", end="", flush=True)  # Progress dots
            
            # Check nodes less frequently
            try:
                result = subprocess.run(
                    ['ros2', 'node', 'list'], 
                    capture_output=True, 
                    text=True, 
                    timeout=5
                )
                
                if result.returncode == 0:
                    running_nodes = [node.strip() for node in result.stdout.split('\n') 
                                   if node.strip() and '/' in node]
                    
                    if len(running_nodes) >= 5:  # Good number of nodes
                        print(f" ✓ ({len(running_nodes)} nodes)")
                        logger.info(f"System startup complete - {len(running_nodes)} nodes running")
                        return
                        
            except Exception:
                pass
        
        print(f" ⏰ (timeout after {elapsed}s)")
        logger.info(f"Startup monitoring complete after {elapsed}s")

def main():
    """Main entry point for console-only integration test."""
    print("🧪 ADT4 Integration Test (Console Mode)")
    print("=" * 50)
    
    test_runner = ConsoleOnlyIntegrationTest()
    
    try:
        success = test_runner.run_all_tests()
        
        print("\n" + "=" * 50)
        print("🎯 TMUX SESSION INFO:")
        print("  🤖 View robotics system: tmux attach-session -t adt4_system")
        print("  🚪 Kill system:          tmux kill-session -t adt4_system")
        print("=" * 50)
        
        return 0 if success else 1
        
    except KeyboardInterrupt:
        print("\n🛑 Tests interrupted by user")
        return 1
    except Exception as e:
        print(f"\n💥 Test execution failed: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
