# Power System – Li-ion Batteries (3S Configuration)

The robot’s power system is based on **three (3) Li-ion 18650 cells connected in series (3S)**. This configuration provides a nominal voltage of **11.1 V** and a fully charged voltage of **12.6 V**, matching the requirements of the motor driver, servo system, and DC-DC converter.

Before selecting Li-ion as our solution, we compared multiple battery technologies commonly used in robotics.

---

# 1 Comparison of Battery Technologies

---

## a) Lithium Batteries (Primary / Non-Rechargeable)

Lithium primary cells are known for **very high energy density**, long shelf life, and excellent temperature tolerance. They come mainly in **AA, AAA, and 9 V** formats.

### Pros
- **2,700–3,400 mAh per AA cell** (highest among disposable cells)  
- Up to **4× longer life** than alkaline batteries  
- Very **lightweight**  
- Excellent performance under **high-drain** and **low-temperature** conditions  
- **10–12 year** shelf life  
- Low risk of leakage during storage

### Cons
- Significantly more expensive  
- **Highly flammable** and require special disposal  
- Not available in larger formats (C, D, etc.)  
- Not rechargeable → unsuitable for robotics

---

## b) Lithium-Ion (Li-ion) Batteries (Rechargeable)

Li-ion cells provide **high energy density**, **high discharge capability**, and **long cycle life**, making them ideal for portable electronics and robotics.

### Technical Characteristics
- **Specific energy:** 100–265 Wh/kg  
- **Energy density:** 250–693 Wh/L  
- **Cycle life:** 300–500 charge cycles (typical), depending on chemistry  
- **Discharge cycles:** up to 500–7000 across chemistries  
- **Components:** cathode, anode, electrolyte, separator  
- Rechargeable and stable under controlled conditions

### Pros
- **High energy and power density**  
- **Rechargeable**, long service life  
- Good efficiency under high load  
- Suitable for 3S, 4S, or multi-cell packs  
- Chooseable mAh capacity (typically 2,500–3,500+ mAh)

### Cons
- Moderate flammability (requires BMS protection)  
- Requires safe charging and storage practices  
- Slightly heavier than LiPo for equivalent capacity

---

## c) Lithium Polymer (LiPo) Batteries (Rechargeable)

LiPo batteries are widely used in RC models due to their **high discharge rates** and **extremely high power density**.

### Pros
- **Very high discharge rate** (burst current capability)  
- **Lightweight** and compact  
- **Low self-discharge**  
- **High voltage per cell:** 3.7 V nominal (11.1 V for 3S)

### Cons
- **Prone to fire** when punctured, overcharged, or damaged  
- More expensive than Li-ion  
- Require strict handling: storage bags, balance charging  
- Disposal requires salt-water neutralization  
- Lower mechanical robustness

---

## d) Alkaline Batteries (Primary)

Alkaline cells are common and inexpensive, but not suitable for high-drain robotics applications.

### Pros
- Widely available, low cost  
- **2,500 mAh+** per AA cell  
- Good shelf life (5–10 years)  
- Leak-free under normal conditions  
- Easy disposal

### Cons
- **Heavy and bulky**  
- Poor performance in **high-drain** applications  
- Voltage sag under load  
- Not rechargeable  
- Potential for leakage in long-term use  

---

# 2 Final Evaluation and Selection

A summary rule-of-thumb from the comparison:

> **For highest voltage under load → choose LiPo**  
> **For highest energy density per weight → choose Li-ion**  
> **For better safety → choose Li-ion**

For our robot, **Li-ion 18650 cells were the optimal choice** because:

- They provide **high capacity (2500–3500 mAh)**  
- They deliver **sufficient discharge current** for the motor and servo  
- They are **safer** and more stable than LiPo  
- 3S configuration provides a **perfect voltage match** for the drivetrain and electronics  
- They have long cycle life and predictable discharge behavior

Therefore:

# **We chose Li-ion 18650 cells (3S pack) for our vehicle.**

---

# 3 Electrical Characteristics and Protection System

The power subsystem includes the following components:

---

## a) Battery Pack (3 × 18650 Li-ion Cells in Series – 3S)
- **Nominal voltage:** 11.1 V  
- **Fully charged voltage:** 12.6 V  
- **Capacity:** 2500–3500 mAh  
- **Continuous current capability:** depends on cell type (typically 10–20 A)

This provides ample power for the motor, servo, Pi, and sensors.

---

## b) 3S Li-ion Battery Management System (BMS) – 10 A

The robot uses a **3S 10A BMS module** providing:

### Protection Features
- **Overcharge protection**  
- **Over-discharge cutoff**  
- **Short-circuit protection**  
- **Cell balancing** (important for multi-cell packs)

### Output Characteristics
- Continuous current: **6–8 A**  
- Peak: up to **10 A**  

This ensures safe operation and extends cell lifespan.

---

## c) DC-DC Step-Down Converter (LM2596S – 5 V / 3 A)

Generates the regulated **5 V rail** for:

- Raspberry Pi 5  
- ToF sensors  
- PCA9685 servo driver  
- Ultrasonic sensors  
- Indicators and buttons

### LM2596S Specs
- **Input voltage:** 3.2–40 V  
- **Output voltage:** 1.3–35 V  
- **Max current:** 3 A  
- High efficiency (~80–90%)  

Provides a stable low-noise supply for all logic and peripherals.

---

# 4 Mechanical & Maintenance Considerations

### External Power Jack
An external DC jack allows:
- Charging the battery without opening the robot  
- Running the robot from an external supply during testing  
- Easier maintenance and safer charging workflow  

### Safety Handling
- Li-ion cells must be stored in a **sealed, fire-resistant container**  
- Avoid mechanical damage or reverse polarity  
- Charge only using protected circuits or dedicated chargers  

---

# 5 System Requirements Summary

The final power system includes:

- **3 × 18650 Li-ion cells (3S)**  
- **3S BMS module (10 A)**  
- **Main power switch**  
- **4 A system fuse** (protects wiring and electronics)  
- **LM2596 buck converter (5 V/3 A)**  
- **External power jack**  

This setup provides a safe, robust, and high-density power platform suitable for the entire robot.

---

# Summary

The selected **Li-ion 18650 3S system** offers the best balance between **safety**, **energy density**, **voltage compatibility**, and **current delivery** for our WRO robot. With proper BMS protection, fused power distribution, and a regulated 5 V rail, the system supports stable and efficient performance across all missions.

---
