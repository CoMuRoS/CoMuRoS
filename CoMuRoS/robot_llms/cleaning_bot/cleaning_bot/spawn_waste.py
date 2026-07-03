#!/usr/bin/env python3
import subprocess
import time

def spawn_ground_plane():
    sdf = '''<?xml version="1.0" ?>
    <sdf version="1.6">
        <model name="small_cube">
            <static>true</static>
            <pose>3.5 -3.0 0.0005 0 0 0</pose>
            <link name="link">
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>0.5 0.5 0.001</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>1 0 0 1</ambient>
                        <diffuse>1 0 0 1</diffuse>
                        <specular>0.5 0.5 0.5 1</specular>
                        <emissive>0.2 0 0 1</emissive>
                    </material>
                </visual>
            </link>
        </model>
    </sdf>'''
    
    sdf_escaped = sdf.replace('\n', ' ').replace('"', '\\"')
    
    cmd = [
        'ign', 'service', '-s', '/world/food_court/create',
        '--reqtype', 'ignition.msgs.EntityFactory',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '1000',
        '--req', f'sdf: "{sdf_escaped}"'
    ]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    print(f"Spawn result: {result.stdout}")
    if result.stderr:
        print(f"Spawn error: {result.stderr}")
    return result.returncode == 0

def remove_ground_plane():
    cmd = [
        'ign', 'service', '-s', '/world/food_court/remove',
        '--reqtype', 'ignition.msgs.Entity',
        '--reptype', 'ignition.msgs.Boolean',
        '--timeout', '1000',
        '--req', 'name: "small_ground_plane", type: 2'
    ]
    
    result = subprocess.run(cmd, capture_output=True, text=True)
    print(f"Remove result: {result.stdout}")
    return result.returncode == 0

def main():
    print("Spawning red ground plane (visual only, no collision)...")
    spawn_ground_plane()
    time.sleep(20)

if __name__ == "__main__":
    main()