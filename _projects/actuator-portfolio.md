---
layout: project
title: Linear Actuator Optimization Portfolio
description: optimizaition of height and weight for a linear-actuator lever system
technologies: [Python]
image: /assets/images/actuator-design-sketch.jpg
---

## Part 1 - Rigid Beam

In Homework 4 of ENGRD 2020 (Statics and Mechanics of Solids), I completed an assignment focusing on designing a lever-based lifting mechanism within a constrained **150 cm × 50 cm** 2D design space. Within this space, we had to design the system using only **three pins** (two of which are ground supports), a linear actuator from the given [Tolomatic catalog](https://www.tolomatic.com/wp-content/uploads/2022/05/2700-4000_29_IMA_cat.pdf), and a **rigid bar** of any fixed length. 


This project demonstrates how principles of statics and mechanics can be combined with computational optimization to design a constrained lever-actuator system. By systematically analyzing forces, geometry, and actuator specifications, I optimized pin placements and bar length to achieve maximum lifting height and payload within the design space.

### Problem Definition

Design a lever–actuator mechanism that operates entirely within a **150 cm × 50 cm** rectangular workspace and can lift the **maximum payload** to the **highest possible tip height**.  

The design must use:
- A single **rigid bar** of fixed length L<sub>bar</sub>,
- **Three pin joints**, two of which are fixed to the ground (P1, P2)
- A **linear actuator** from the Tolomatic catalog (using maximum thrust).

### Constraints and Objectives

| Category | Description |
|-----------|--------------|
| **Geometric Constraint** | All components must fit within the 150 cm × 50 cm box before actuator extension. The bar rotates about P1.|
| **Actuator Limits** | Tolomatic **IMA55 RN05**: max thrust = 35.81 kN, stroke = 457.2 mm, body length = 330 mm |
| **Mechanical Constraints** | All members are rigid; pin joints are frictionless. The actuator can only push/pull along its axis. |
| **Performance Objective** | Maximize W = F x d (force exerted by actuator and vertical lift height). |

### Design Degrees of Freedom (DoF)

1. L<sub>bar</sub>: bar length
2. x = P1-P2: ground-pin spacing (actuator base position).  
3. a = P1-P3: actuator-bar connection point.  
4. $\theta$<sub>min</sub>: initial bar angle (fits under 50 cm limit).  
5. $\theta$<sub>max</sub>: final bar angle determined by actuator stroke.  

When designing the system the trade-off between lift height and payload posed a significant challenge. The placement of the actuator attachment point on the bar (P3) relative to the pivot (P1) determines the performance of the system. Positioning P3 closer to P1 (smaller moment arm for the linear actuator's force) increases the bar’s angular sweep for a fixed actuator stroke, which raises the maximum tip height. However, the reduced moment arm lowers the actuator’s moment about P1, decreasing payload capacity. Placing P3 farther from P1 does the opposite; it results in a greater payload capacity but reduced angular sweep and thus lower maximum height.

Balancing these competing effects required the consideration of the moment about P1 from both the actuator and the applied load, along with the kinematics and geometry of the bar’s motion. The optimization determined feasible pin placements that produced the best compromise between payload and height.

The actuator was selected from the catalog by considering the listed stroke lengths and peak thrust values. The IMA55 RN05 linear actuator was chosen due to its maximum stroke length of 457.2 mm and a maximum force of 35.81 kN.

The length of the rigid bar was chosen to be 1580 mm because this is the maximum possible length the bar can be and still fit within the 150 cm x 50 cm design space. The bar will be along the diagonal of the design space at a starting angle of 18.3 degrees.

<img src="{{ "/assets/images/actuator-design-sketch.jpg" | relative_url }}" alt="Design Sketch" width="600">

The image shows the sketch of the design. P1 and P2 are the 2 ground support pins. P1 is attached to the bar and is the point about which the bar rotates when pushed by the linear actuator. P2 is the pin that attaches the linear actuator to the ground. P3 is the pin that connects the tip of the linear actuator to some point on the rigid bar. P1 is located at one end of the bar. 

### Statics & Kinetmatics Analysis

To determine the distance between P1 and P2 and between P1 and P3, the work done by the actuator (W = F x d) will be maximized. The force done by the actuator will be determined by taking the moment about P1. This will include the force exerted by the payload and the force exerted by the actuator about P1. The force exerted by the actuator is the same as the peak thrust value listed in the catalog. d, the vertical distance the tip of the rigid bar moves, is determined by using the Law of Cosines to find the angle, theta, the bar moves and its final height. 

For any bar angle $\theta$, the actuator length $L$ is determined from geometry:
$L(\theta) = \sqrt{a^2 + x^2 - 2 a x \cos{\theta}}$

Feasible motion requires:
$L_{min} = L(\theta_{min}) \geq L_{body}$
and
$L_{max} = L(\theta_{max}) \leq L_{body} + s$

Thus,
$\cos{\theta_{max}} = \dfrac{a^2 + x^2 - (L_{body} + s)^2}{2 a x}$

Taking moments about the pivot P1 gives the payload equation:
$W = F_a \cdot \dfrac{a}{L_{bar}}$

This shows that increasing $a$ boosts payload capacity by enlarging the actuator moment arm but also reduces angular motion and vertical lift.

The vertical position of the bar tip is:
$y = L_{bar} \cdot \sin{\theta}$

and the total vertical lift is:
$H = L_{bar} \cdot (\sin{\theta_{max}} - \sin{\theta_{min}})$

To evaluate performance, the optimization maximizes:
$U = W \times H$

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

## Part 2 - Flexible Beam (Deflection and Sizing)

In this part, we are reconsidering the problem from the original assignment. The rigid bar is now treated as a **beam** that bends under the transverse components of the actuator and payload forces.

### Assumptions and Setup

- Linear-elastic, small-deflection (Euler–Bernoulli).  
- Critical position near $\theta_{min}=18.3°$.  
- Actuator-bar angle $\alpha≈70°$.  
- Beam length $L=1.58 \text{m}$ with loads:
  - tip load $W_\perp$ at $x=L$,  
  - point load $F_\perp$ at $x=a=1.04 \text{m}$.

Known: $W=23.57 \text{kN}$, $F_a=35.81 \text{kN}$.

Transverse components:  
$W_\perp=W\cos{\theta_{min}}$ → $W_\perp≈22.4 \text{kN}$  
$F_\perp=F_a\sin{\alpha}$ → $F_\perp≈33.6 \text{kN}$

---

### Tip Deflection

For a cantilever:

$\delta_{W}=\dfrac{W_\perp L^3}{3 E I}$ (end load)  

$\delta_{F}=\dfrac{F_\perp a^2(3L-a)}{6 E I}$ (point load)

Total deflection:  
$\delta_{tip}=\dfrac{1}{E I}\left(\dfrac{W_\perp L^3}{3}+\dfrac{F_\perp a^2(3L-a)}{6}\right)$

Define $K=\dfrac{W_\perp L^3}{3}+\dfrac{F_\perp a^2(3L-a)}{6}$, so $\delta_{tip}=K/(E I)$.

Numerically, $K \approx 5.19 \times 10^{4}~\text{N}\cdot\text{m}^3$.

Allowable deflection (2% of length): $\delta_{allow}=0.02L=0.0316 \text{m}$.

Required moment of inertia:  
$I_{min}=K/(E \delta_{allow})$

| Material (from Appendix C) | $E$ (GPa) | $I_{min}$ (m⁴) |
|-----------|----------|----------------|
| Steel (A36) | 200 | 8.2 × 10⁻⁶ |
| Aluminum 6061-T6 | 70 | 2.4 × 10⁻⁵ |
| Titanium (6% Al, 4% V) | 115 | 1.4 × 10⁻⁵ |
| Magnesium AZ31 (extruded) | 45 | 3.7 × 10⁻⁵ |

**Maximum computed deflection:**  
Using $E=70~\text{GPa}$ (Al 6061-T6) and $I = I_{min} = 2.4\times10^{-5}~\text{m}^4$,  
$\delta_{tip} = \dfrac{K}{E I} \approx \dfrac{5.19\times10^{4}}{(70\times10^{9})(2.4\times10^{-5})} \approx 0.031~\text{m}$  
→ **$\delta_{tip} \approx 31~\text{mm}$**, matching the 2 % limit.

---

### Mass-Efficient Beam Design

For a square tube: $I=(b^4-(b-2t)^4)/12$, $A=b^2-(b-2t)^2$, mass per length $m'=\rho A$.

A square tube was chosen because hollow sections are more stiff per unit mass compared to solid bars. Additionally, a square tube has more structural advantages compared to a circular tube because flat faces make it easy to mount actuator, pins, and plates. Square and rectangular tubes are weaker in torsion, but torsion is not being considered in this design since it is less critical. 

**Best choice:** Aluminum 6061-T6  
This alloy offers the best combination of stiffness-to-weight and availability among the tabulated metals.

**Sized section satisfying both deflection and strength:**

- Required stiffness: $I \ge I_{min}(Al)=2.4\times10^{-5}~\text{m}^4$  
- Yield strength: $\sigma_y ≈ 240~\text{MPa}$  
- Maximum bending moment $M_{max} ≈ 53.5~\text{kN}\cdot\text{m}$  
- Required section modulus $S_{req}=M_{max}/\sigma_y ≈ 2.23\times10^{-4}~\text{m}^3$

Choosing a square tube with  
$b≈250 \text{ mm}$, $t≈2.8 \text{ mm}$  

then  
$I≈2.9×10^{-5} \text{m}^4 (≥I_{min})$,  
$S=\dfrac{I}{b/2}=2.32×10^{-4} \text{m}^3 (≥S_{req})$,  
$\delta≈25\text{–}26 \text{ mm} < \delta_{allow}$.

Mass per length (thin-wall):  
$A≈4bt≈0.0028 \text{m}^2$,  
$m'=\rho A≈2710×0.0028≈7.6 \text{kg/m}$  
→ **Total mass ≈ 12.0 kg** for $L=1.58 \text{m}$.

**Alternatives (for comparison):**
- Steel (A36): $I_{min}$ small but density 7860 kg/m³ → ≈ 40 kg for equal stiffness.  
- Titanium (6Al-4V): stronger and stiffer ($E=115 \text{GPa}$) but denser → ≈ 15 kg.  
- Magnesium AZ31: light but very low $E$ → requires large section, similar total mass.

---

### Stress Check

$M_{max} \approx W_\perp L + F_\perp (L - a) \approx 53.5~\text{kN}\cdot\text{m}$

$S=\dfrac{I}{b/2}=2.32\times10^{-4}~\text{m}^3$  

$\sigma_{max}=\dfrac{M_{max}}{S}\approx2.31\times10^{8}~\text{Pa}=231~\text{MPa}$  
$\sigma_{max}<\sigma_y(6061\text{-T6})=240~\text{MPa}$ → **Safe.**

*(If greater margin is needed, increase wall thickness $t$ to 3.0 mm.)*

---

### Final Design Summary

- Beam material: **Aluminum 6061-T6** square tube 250×250×2.8 mm, $L=1.58 \text{m}$  
- Deflection: ≈ 25–26 mm (< 2% limit)  
- Mass: ≈ 12 kg  
- Yield strength: 240 MPa, $\sigma_{max}$ = 231 MPa → OK  
- Actuator: IMA55 RN05 (35.81 kN thrust, 457.2 mm stroke)  
- Pin spacing: P1–P2 ≈ P1–P3 ≈ 1040 mm  
- Angles: 18.3° → 44°  
- Tip height: ≈ 1100 mm, Payload: ≈ 23.6 kN

The image shows a schematic (not drawn to scale) of the deflected beam as well as a cross-section of the beam being used.

<img src="{{ "assets/images/flexible-actuator.png" | relative_url }}" alt="Design Sketch with Cross-Section" width="600">

---

### Interpretation
The rigid-bar optimization (Step 1) defined the geometry for maximum lift and payload;  
the beam analysis (Step 2) ensured the structure remains stiff and safe under load while minimizing mass.  
Within the materials listed in the reference tables, **Aluminum 6061-T6** provides the optimal balance of weight, stiffness, and manufacturability for this actuator-driven lifting mechanism.
