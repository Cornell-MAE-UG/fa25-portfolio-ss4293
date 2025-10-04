---
layout: project
title: Linear Actuator Optimization Portfolio
description: optimizaition of height and weight for a linear-actuator lever system
technologies: [Python]
image: /assets/images/actuator-design-sketch.jpg
---


In Homework 4 of ENGRD 2020 (Statics and Mechanics of Solids), I completed an assignment focusing on designing a lever-based lifting mechanism within a constrained 2D design space measuring 150 cm in length and 50 cm in height. Within this space, we had to design the system using only 3 pins (2 of which are ground supports), a linear actuator from the given [Tolomatic catalog](https://www.tolomatic.com/wp-content/uploads/2022/05/2700-4000_29_IMA_cat.pdf), and a rigid bar of any fixed length. 

This project demonstrates how principles of statics and mechanics can be combined with computational optimization to design a constrained lever-actuator system. By systematically analyzing forces, geometry, and actuator specifications, I optimized pin placements and bar length to achieve maximum lifting height and payload within the design space.

All elements are assumed to be rigid. The objective  of this project is  to create a system capable of lifting the maximum possible weight to the greatest possible height.

When designing the system the trade-off between lift height and payload posed a significant challenge. The placement of the actuator attachment point on the bar (P3) relative to the pivot (P1) determines the performance of the system. Positioning P3 closer to P1 (smaller moment arm for the linear actuator's force) increases the bar’s angular sweep for a fixed actuator stroke, which raises the maximum tip height. However, the reduced moment arm lowers the actuator’s moment about P1, decreasing payload capacity. Placing P3 farther from P1 does the opposite; it results in a greater payload capacity but reduced angular sweep and thus lower maximum height.

Balancing these competing effects required the consideration of the moment about P1 from both the actuator and the applied load, along with the kinematics and geometry of the bar’s motion. The optimization determined feasible pin placements that produced the best compromise between payload and height.

The actuator was selected from the catalog by considering the listed stroke lengths and peak thrust values. The IMA55 RN05 linear actuator was chosen due to its maximum stroke length of 457.2 mm and a maximum force of 35.81 kN.

The length of the rigid bar was chosen to be 1580 mm because this is the maximum possible length the bar can be and still fit within the 150 cm x 50 cm design space. The bar will be along the diagonal of the design space at a starting angle of 18.3 degrees.

<img src="{{ "/assets/images/actuator-design-sketch.jpg" | relative_url }}" alt="Design Sketch" width="600">

The image shows the sketch of the design. P1 and P2 are the 2 ground support pins. P1 is attached to the bar and is the point about which the bar rotates when pushed by the linear actuator. P2 is the pin that attaches the linear actuator to the ground. P3 is the pin that connects the tip of the linear actuator to some point on the rigid bar. P1 is located at one end of the bar. 

To determine the distance between P1 and P2 and between P1 and P3, the work done by the actuator (W = F x d) will be maximized. The force done by the actuator will be determined by taking the moment about P1. This will include the force exerted by the payload and the force exerted by the actuator about P1. The force exerted by the actuator is the same as the peak thrust value listed in the catalog. d, the vertical distance the tip of the rigid bar moves, is determined by using the Law of Cosines to find the angle, theta, the bar moves and its final height. 


The python code used to calculate and optimize these values is shown below:

```python
import math

# --- INPUT PARAMETERS (Physical and Geometric Constraints) ---
# Actuator: IMA55 RN05 (Max force 35.81 kN)
MAX_ACTUATOR_FORCE_N = 35810.0      # N (35.81 kN)
STROKE_S = 457.2                    # mm (Max stroke for IMA55)

# Design Constraints
L_BAR = 1580.0                      # mm (Max feasible length based on 1500x500 box hypotenuse)
THETA_MIN_DEG = 18.3                # Degrees (Starting angle to fit initial Y-height < 500mm)
L_BODY_MIN = 330.0                  # mm (Physical fully retracted length of IMA55 housing, estimated)

# Derived Fixed Constants
THETA_MIN_RAD = math.radians(THETA_MIN_DEG)
L_MAX_PHYSICAL = L_BODY_MIN + STROKE_S  # 330.0 + 457.2 = 787.2 mm (Absolute longest actuator can be)

# --- GEOMETRIC & PERFORMANCE FUNCTIONS ---

def calculate_kinematics(a, x2):
    """
    Calculates the resulting L_min and L_max, and checks feasibility.
    L_MAX_PHYSICAL is used to find the theoretical max angle.
    """
    
    # 1. Calculate Required Minimum Length (L_min_req)
    # L_min_req is the length required by the mechanism to sit at THETA_MIN
    try:
        L_min_req = math.sqrt(a**2 + x2**2 - 2 * a * x2 * math.cos(THETA_MIN_RAD))
    except ValueError:
        return None, "Invalid Geometry (a, x2 distance too small)"

    # 2. SAFETY CHECK: Check against the physical actuator limit (L_body)
    if L_min_req < L_BODY_MIN:
        return None, f"L_min {L_min_req:.1f} < L_body {L_BODY_MIN} (Actuator crashes)"

    # 3. Calculate Max Angle (Theta_max) using the Fixed Physical L_MAX_PHYSICAL
    # Cos(theta_max) = (a^2 + x2^2 - L_max^2) / (2 * a * x2)
    
    L_max_sq = L_MAX_PHYSICAL**2
    cos_theta_max = (a**2 + x2**2 - L_max_sq) / (2 * a * x2)
    
    # Check for range validity (acos argument must be between -1 and 1)
    if not (-1.0 <= cos_theta_max <= 1.0):
        return None, "Invalid Angle (Actuator cannot reach or pass L_MAX)"
    
    theta_max_rad = math.acos(cos_theta_max)
    theta_max_deg = math.degrees(theta_max_rad)

    # 4. Calculate Performance Metrics
    
    # Y_tip_max: Max height reached (H)
    Y_tip_max = L_BAR * math.sin(theta_max_rad)
    
    # Payload W (Max Force * Moment Arm Ratio)
    # W = F_actuator * (a / L_bar)
    W_payload_N = MAX_ACTUATOR_FORCE_N * (a / L_BAR)
    W_payload_kN = W_payload_N / 1000
    
    # Calculate Total Lift (H) from Y_tip_min to Y_tip_max
    Y_tip_min = L_BAR * math.sin(THETA_MIN_RAD)
    H_lift = Y_tip_max - Y_tip_min
    
    # Work Product (W * H) is the optimization metric
    work_product_kJ = (W_payload_N / 1000) * (H_lift / 1000)

    # Package results
    results = {
        'a': a, 'x2': x2, 'W_kN': W_payload_kN, 'Y_tip_max': Y_tip_max,
        'H_lift': H_lift, 'Work_kJ': work_product_kJ, 'Theta_max': theta_max_deg,
        'L_min_req': L_min_req
    }
    return results, "OK"

# --- OPTIMIZATION SEARCH LOOP ---

def optimize_design():
    """Iterates through possible a and x2 values to maximize Work Product."""

    best_work = -1.0
    best_design = None
    
    # Search range centered around your input (1050mm) and extending to 1580mm
    a_x2_range = range(900, int(L_BAR) + 1, 10)
    
    for val in a_x2_range:
        a = val
        x2 = val # Enforce a = x2 geometry for maximum angular sweep

        results, status = calculate_kinematics(a, x2)

        if status == "OK" and results['Work_kJ'] > best_work:
            best_work = results['Work_kJ']
            best_design = results
            
    return best_design

# Run the optimization
optimal_design = optimize_design()

# --- PRINT RESULTS ---

print("--- IMA55 RN05 LINKAGE OPTIMIZATION RESULTS ---")
print(f"Goal: Maximize Work Product (W * H) while enforcing L_min >= L_body ({L_BODY_MIN} mm)")
print(f"Fixed Actuator Force (F_actuator): {MAX_ACTUATOR_FORCE_N / 1000:.1f} kN")
print(f"Fixed Max Link Length (L_max): {L_MAX_PHYSICAL:.1f} mm\n")

if optimal_design:
    print("--- OPTIMAL DESIGN PARAMETERS (MAX WORK) ---")
    print(f"Input Arm (a): {optimal_design['a']:.1f} mm")
    print(f"Actuator Base (x2): {optimal_design['x2']:.1f} mm")
    print(f"")
    print("--- PERFORMANCE METRICS ---")
    print(f"Max Payload (W): {optimal_design['W_kN']:.2f} kN")
    print(f"Max Final Height (Y_tip_max): {optimal_design['Y_tip_max']:.1f} mm")
    print(f"Total Lift (H): {optimal_design['H_lift']:.1f} mm")
    print(f"Max Angle (Theta_max): {optimal_design['Theta_max']:.2f} degrees")
    print(f"Final Work Product: {optimal_design['Work_kJ']:.2f} kJ (MAXIMUM)")
    print(f"")
    print("--- FEASIBILITY CHECKS ---")
    print(f"Required L_min (L_min_req): {optimal_design['L_min_req']:.1f} mm")
    print(f"Physical L_body limit: {L_BODY_MIN} mm")
else:
    print("Optimization failed to find a valid solution space that meets all constraints.")
```

Running the code allows you to determine that the distance from P1 and P2 (denoted as x) is about 1040 mm. The distance from P1 and P3 along the bar (denoted as a) is also about 1040 mm.

These values allow the bar to lift a maximum load of 23.57 kN to a final height of about 1100 mm. 

#### Key Results:
- Pin Distances: P1–P2 ≈ 1040 mm, P1–P3 ≈ 1040 mm
- Maximum Payload: 23.57 kN
- Final Lift Height: ≈ 1100 mm
- Work Product: Maximized within all space and actuator constraints

The system is very sensitive to the placement of the pins. Small variations in distances between P1, P2, and P3 can cause the actuator to exceed stroke limits or drastically reduce payload. The final design represents an optimized balance that achieves substantial lift capacity and significant height gain.