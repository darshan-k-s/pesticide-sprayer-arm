#!/usr/bin/env python3
"""
Arduino detection script
Check whether Arduino devices can be discovered and accessed
"""

import os
import sys
import stat

def check_arduino_ports():
    """Check possible Arduino serial ports"""
    possible_ports = [
        "/dev/ttyUSB0",
        "/dev/ttyUSB1", 
        "/dev/ttyUSB2",
        "/dev/ttyACM0",
        "/dev/ttyACM1",
        "/dev/ttyACM2"
    ]
    
    found_ports = []
    
    print("=" * 60)
    print("Arduino device detection")
    print("=" * 60)
    print()
    
    for port in possible_ports:
        if os.path.exists(port):
            try:
                # Check file type
                file_stat = os.stat(port)
                is_char_device = stat.S_ISCHR(file_stat.st_mode)
                
                if is_char_device:
                    # Check permissions
                    readable = os.access(port, os.R_OK)
                    writable = os.access(port, os.W_OK)
                    
                    found_ports.append({
                        'port': port,
                        'readable': readable,
                        'writable': writable,
                        'mode': oct(file_stat.st_mode)[-3:]
                    })
                    
                    print(f" Found device: {port}")
                    print(f"  Type: character device")
                    print(f"  Mode: {oct(file_stat.st_mode)[-3:]}")
                    print(f"  Readable: {'yes' if readable else 'no'}")
                    print(f"  Writable: {'yes' if writable else 'no'}")
                    print()
            except Exception as e:
                print(f" Error while checking {port}: {e}")
                print()
    
    if not found_ports:
        print(" No Arduino serial devices found")
        print()
        return False
    
    # Check user permissions
    print("Permission check:")
    print("-" * 60)
    
    # Check whether the user is in the dialout group
    import grp
    try:
        user_groups = [g.gr_name for g in grp.getgrall() if os.getlogin() in g.gr_mem]
        user_groups.append(grp.getgrgid(os.getgid()).gr_name)
        
        in_dialout = 'dialout' in user_groups
        print(f"User in 'dialout' group: {'yes' if in_dialout else 'no'}")
        
        if not in_dialout:
            print("  Warning: user is not in 'dialout' group, serial ports may not be accessible")
            print("   Fix: sudo usermod -a -G dialout $USER")
            print("   Then re-login or run: newgrp dialout")
            print()
    except Exception as e:
        print(f"Error while checking user groups: {e}")
        print()
    
    # Try to open detected devices
    print("Device access test:")
    print("-" * 60)
    for port_info in found_ports:
        port = port_info['port']
        if port_info['writable']:
            try:
                # Try to open in read/write mode (non-blocking, but without actual writes)
                fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                os.close(fd)
                print(f" {port}: successfully opened (accessible)")
            except PermissionError:
                print(f" {port}: insufficient permissions, cannot open")
                print(f"  You need to add the user to the 'dialout' group or use sudo")
            except Exception as e:
                print(f" {port}: failed to open - {e}")
        else:
            print(f" {port}: no write permission")
    
    print()
    print("=" * 60)
    
    # Summary
    accessible_ports = [p for p in found_ports if p['writable']]
    if accessible_ports:
        print(f" Detected {len(accessible_ports)} accessible Arduino device(s)")
        for p in accessible_ports:
            print(f"  - {p['port']}")
        return True
    else:
        print(" Arduino devices detected but not accessible (permission issue)")
        return False

if __name__ == "__main__":
    success = check_arduino_ports()
    sys.exit(0 if success else 1)




