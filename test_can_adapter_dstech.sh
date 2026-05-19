#!/bin/bash

###############################################################################
# DSDTech SH-C30A USB-CAN Adapter Test Script
# Tests device recognition and CAN communication capabilities
###############################################################################

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# CAN interface to test (default: can1 = DSDTech USB adapter)
CAN_IF="${1:-can1}"

# Test results counter
TESTS_PASSED=0
TESTS_FAILED=0

print_header() {
    echo -e "\n${BLUE}=====================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}=====================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
    ((TESTS_PASSED++))
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
    ((TESTS_FAILED++))
}

print_info() {
    echo -e "${YELLOW}ℹ $1${NC}"
}

###############################################################################
# Test 1: Check USB Device Recognition
###############################################################################
test_usb_recognition() {
    print_header "Test 1: USB Device Recognition"
    
    if lsusb | grep -q "1d50:606f"; then
        print_success "USB device detected (Geschwister Schneider CAN adapter)"
        lsusb | grep "1d50:606f"
    else
        print_error "USB device NOT detected"
        print_info "Expected: Device ID 1d50:606f"
        echo "Available USB devices:"
        lsusb
        return 1
    fi
}

###############################################################################
# Test 2: Check CAN Interface Availability
###############################################################################
test_can_interface() {
    print_header "Test 2: CAN Interface Availability"
    
    if ip link show $CAN_IF &>/dev/null; then
        print_success "CAN interface '$CAN_IF' exists"
        ip -details link show $CAN_IF
    else
        print_error "CAN interface '$CAN_IF' NOT found"
        print_info "Available network interfaces:"
        ip link show
        return 1
    fi
}

###############################################################################
# Test 3: Check CAN Interface State
###############################################################################
test_can_state() {
    print_header "Test 3: CAN Interface State"
    
    STATE=$(ip -details link show $CAN_IF | grep -oP 'state \K\S+' | head -1)
    if [[ "$STATE" == "UP" ]]; then
        print_success "CAN interface is UP"
    else
        print_error "CAN interface is $STATE (expected UP)"
        print_info "Attempting to bring interface up..."
        sudo ip link set $CAN_IF up
        if [[ $? -eq 0 ]]; then
            print_success "Interface brought up successfully"
        else
            print_error "Failed to bring interface up"
            return 1
        fi
    fi
    
    # Check CAN state
    CAN_STATE=$(ip -details link show $CAN_IF | grep -oP 'can state \K\S+')
    if [[ "$CAN_STATE" == "ERROR-ACTIVE" ]]; then
        print_success "CAN state is ERROR-ACTIVE (ready for communication)"
    else
        print_info "CAN state: $CAN_STATE"
    fi
    
    # Display bitrate
    BITRATE=$(ip -details link show $CAN_IF | grep -oP 'bitrate \K\d+')
    print_info "Current bitrate: $BITRATE bps"
}

###############################################################################
# Test 4: Check CAN Utilities
###############################################################################
test_can_utilities() {
    print_header "Test 4: CAN Utilities Availability"
    
    if command -v candump &>/dev/null; then
        print_success "candump utility found: $(which candump)"
    else
        print_error "candump utility NOT found"
        print_info "Install with: sudo apt-get install can-utils"
    fi
    
    if command -v cansend &>/dev/null; then
        print_success "cansend utility found: $(which cansend)"
    else
        print_error "cansend utility NOT found"
        print_info "Install with: sudo apt-get install can-utils"
    fi
    
    if command -v cangen &>/dev/null; then
        print_success "cangen utility found: $(which cangen)"
    else
        print_info "cangen utility not found (optional)"
    fi
}

###############################################################################
# Test 5: CAN Loopback Test
###############################################################################
test_can_loopback() {
    print_header "Test 5: CAN Communication (Loopback Test)"
    
    if ! command -v candump &>/dev/null || ! command -v cansend &>/dev/null; then
        print_error "CAN utilities not available, skipping communication test"
        return 1
    fi
    
    print_info "Starting candump in background..."
    DUMP_FILE=$(mktemp)
    timeout 2 candump $CAN_IF > "$DUMP_FILE" 2>&1 &
    CANDUMP_PID=$!
    sleep 0.5
    
    print_info "Sending test CAN message (ID: 123, Data: DEADBEEF)..."
    if cansend $CAN_IF 123#DEADBEEF 2>&1; then
        print_success "Message sent successfully"
    else
        print_error "Failed to send message"
        kill $CANDUMP_PID 2>/dev/null
        rm -f "$DUMP_FILE"
        return 1
    fi
    
    sleep 0.5
    kill $CANDUMP_PID 2>/dev/null
    wait $CANDUMP_PID 2>/dev/null
    
    if grep -q "123.*DE AD BE EF" "$DUMP_FILE"; then
        print_success "Message received successfully (loopback working)"
        echo "Captured message:"
        cat "$DUMP_FILE"
    else
        print_error "Message NOT received"
        print_info "Candump output:"
        cat "$DUMP_FILE"
    fi
    
    rm -f "$DUMP_FILE"
}

###############################################################################
# Test 6: Send Multiple Messages
###############################################################################
test_multiple_messages() {
    print_header "Test 6: Multiple Message Test"
    
    if ! command -v cansend &>/dev/null; then
        print_error "cansend not available, skipping test"
        return 1
    fi
    
    print_info "Sending 5 test messages..."
    
    MESSAGES=(
        "100#11223344"
        "200#AABBCCDD"
        "300#12345678"
        "400#FEDCBA98"
        "7FF#FFFFFFFF"
    )
    
    for msg in "${MESSAGES[@]}"; do
        if cansend $CAN_IF "$msg" 2>&1; then
            print_success "Sent: $msg"
        else
            print_error "Failed to send: $msg"
        fi
        sleep 0.1
    done
}

###############################################################################
# Test 7: Driver Information
###############################################################################
test_driver_info() {
    print_header "Test 7: Driver Information"
    
    DRIVER=$(ip -details link show $CAN_IF | grep -oP 'gs_usb')
    if [[ "$DRIVER" == "gs_usb" ]]; then
        print_success "Using gs_usb driver (correct for DSDTech SH-C30A)"
    else
        print_info "Driver information not found in interface details"
    fi
    
    # Show full interface details
    print_info "Full interface details:"
    ip -details link show $CAN_IF | sed 's/^/    /'
}

###############################################################################
# Main Test Execution
###############################################################################
main() {
    echo -e "${BLUE}"
    echo "###############################################################################"
    echo "#  DSDTech SH-C30A USB-CAN Adapter Test Suite"
    echo "#  Date: $(date)"
    echo "###############################################################################"
    echo -e "${NC}"
    
    test_usb_recognition
    test_can_interface
    test_can_state
    test_can_utilities
    test_can_loopback
    test_multiple_messages
    test_driver_info
    
    # Summary
    print_header "Test Summary"
    echo -e "Tests Passed: ${GREEN}$TESTS_PASSED${NC}"
    echo -e "Tests Failed: ${RED}$TESTS_FAILED${NC}"
    
    if [[ $TESTS_FAILED -eq 0 ]]; then
        echo -e "\n${GREEN}✓ All tests passed! CAN adapter is fully functional.${NC}\n"
        return 0
    else
        echo -e "\n${RED}✗ Some tests failed. Please check the errors above.${NC}\n"
        return 1
    fi
}

# Run tests
main
exit $?