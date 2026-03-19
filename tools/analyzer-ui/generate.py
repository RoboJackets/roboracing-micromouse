import math
import random
import csv

with open('log_001.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    headers = ['timestamp_us', 'w/x', 'w/y', 'w/theta', 'IR[0]', 'IR[1]', 'IR[2]', 'IR[3]', 'state']
    writer.writerow(headers)
    
    x = 0.09 # Center of cell (0,0) in meters
    y = 0.09
    theta = math.pi / 2
    
    for i in range(2000):
        t_us = i * 100000 # 100ms steps
        
        # Simple differential drive kinematics for a testing curve
        velocity = 0.5
        omega = math.sin(i * 0.02) * 2.0
        
        theta += omega * 0.1
        x += velocity * math.cos(theta) * 0.1
        y += velocity * math.sin(theta) * 0.1
        
        # Simulated IR sensors (noise + proximity to walls based on sine waves simulating passing cells)
        ir0 = 100 + math.sin(x * 20) * 50 + random.random() * 10
        ir1 = 100 + math.sin(y * 20) * 50 + random.random() * 10
        ir2 = 50 + random.random() * 5
        ir3 = 50 + random.random() * 5
        
        state = (i // 100) % 4
        
        writer.writerow([t_us, f"{x:.4f}", f"{y:.4f}", f"{theta:.4f}", f"{ir0:.2f}", f"{ir1:.2f}", f"{ir2:.2f}", f"{ir3:.2f}", state])
        
print("Successfully generated log_001.csv")
