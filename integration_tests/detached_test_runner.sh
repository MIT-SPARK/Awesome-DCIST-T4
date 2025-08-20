#!/bin/bash
# Run integration tests in a completely separate process/terminal

set -e

echo "🧪 ADT4 Detached Integration Test Runner"
echo "========================================"

# Configuration
WORKSPACE_DIR="$ADT4_WS"
LOG_DIR="/tmp/adt4_integration"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
TEST_LOG="$LOG_DIR/test_$TIMESTAMP.log"
PID_FILE="$LOG_DIR/test_runner.pid"

# Create log directory
mkdir -p "$LOG_DIR"

echo "📁 Workspace: $WORKSPACE_DIR"
echo "📝 Test log: $TEST_LOG" 
echo "🔄 Process ID file: $PID_FILE"
echo ""

# Function to launch test in new terminal
launch_in_new_terminal() {
    local test_script="$LOG_DIR/run_test_$TIMESTAMP.sh"
    
    # Create a script to run in the new terminal
    cat > "$test_script" << EOF
#!/bin/bash
cd "$WORKSPACE_DIR"

echo "🧪 ADT4 Integration Test Runner"
echo "Started at: \$(date)"
echo "Workspace: $WORKSPACE_DIR"
echo "Log file: $TEST_LOG"
echo "=" * 50

# Set environment
source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null || source "$WORKSPACE_DIR/install/setup.zsh" 2>/dev/null || true

# Run the test
echo "🚀 Starting integration tests..."
./src/awesome_dcist_t4/integration_tests/prior_dsg_integration_test.py 2>&1 | tee "$TEST_LOG"

echo ""
echo "✅ Integration test completed at: \$(date)"
echo "📝 Full log saved to: $TEST_LOG"
echo ""
echo "Press Enter to close this window..."
read
EOF

    chmod +x "$test_script"
    
    # Try different terminal emulators
    if command -v gnome-terminal >/dev/null 2>&1; then
        echo "🖥️  Launching in gnome-terminal..."
        gnome-terminal --title="ADT4 Integration Tests" --working-directory="$WORKSPACE_DIR" -- bash "$test_script" &
        echo $! > "$PID_FILE"
    elif command -v xterm >/dev/null 2>&1; then
        echo "🖥️  Launching in xterm..."
        xterm -title "ADT4 Integration Tests" -e "bash '$test_script'" &
        echo $! > "$PID_FILE"
    elif command -v konsole >/dev/null 2>&1; then
        echo "🖥️  Launching in konsole..."
        konsole --title "ADT4 Integration Tests" --workdir "$WORKSPACE_DIR" -e bash "$test_script" &
        echo $! > "$PID_FILE"
    else
        echo "❌ No suitable terminal emulator found!"
        echo "💡 Available options: gnome-terminal, xterm, konsole"
        echo "📝 You can run manually: bash $test_script"
        return 1
    fi
    
    echo "✅ Integration test launched in separate terminal"
    return 0
}

# Function to run test in background with monitoring
run_in_background() {
    echo "🔄 Running integration test in background..."
    
    # Run in background with nohup
    cd "$WORKSPACE_DIR"
    nohup ./src/awesome_dcist_t4/integration_tests/prior_dsg_integration_test.py > "$TEST_LOG" 2>&1 &
    local test_pid=$!
    echo $test_pid > "$PID_FILE"
    
    echo "✅ Test started with PID: $test_pid"
    echo "📝 Monitor progress: tail -f $TEST_LOG"
    
    return 0
}

# Main execution
echo "Choose execution method:"
echo "1) 🖥️  New terminal window (recommended)"
echo "2) 🔄 Background process"
echo "3) 📝 View existing log"
echo ""
read -p "Select option (1-3): " choice

case $choice in
    1)
        if launch_in_new_terminal; then
            echo ""
            echo "🎯 INTEGRATION TEST LAUNCHED"
            echo "================================"
            echo "📺 Test running in separate terminal window"
            echo "📝 Monitor: tail -f $TEST_LOG"
            echo "🤖 View robotics system: tmux attach-session -t adt4_system"
            echo "🛑 Stop test: kill \$(cat $PID_FILE) 2>/dev/null"
            echo "================================"
        fi
        ;;
    2)
        run_in_background
        echo ""
        echo "🎯 INTEGRATION TEST RUNNING"
        echo "============================"
        echo "📝 Monitor: tail -f $TEST_LOG"
        echo "🤖 View robotics system: tmux attach-session -t adt4_system"
        echo "🛑 Stop test: kill \$(cat $PID_FILE) 2>/dev/null"
        echo "============================"
        ;;
    3)
        echo "📋 Recent log files:"
        ls -la "$LOG_DIR"/test_*.log 2>/dev/null | tail -5 || echo "No log files found"
        echo ""
        read -p "Enter log file to view: " log_file
        if [[ -f "$log_file" ]]; then
            tail -f "$log_file"
        else
            echo "❌ File not found: $log_file"
        fi
        ;;
    *)
        echo "❌ Invalid option"
        exit 1
        ;;
esac
