## Prototype_2 — Integrated 4-in-1 ESC Lightweight Revision

Prototype_2 retains the same overall mission concept as Prototype_1, but introduces a major hardware refinement through a more compact and lighter electrical architecture.

### Main hardware change

The most important change in Prototype_2 is the replacement of the individual arm-mounted ESC layout with a centrally mounted:

**HAKRC 65A 2–8S BLHeli_32 4-in-1 Brushless ESC (30.5 × 30.5 mm)**

This replaces the earlier Prototype_1 configuration, which used separate **Hobbywing XRotor 40A ESCs** mounted individually with longer wires and additional bullet-style connector mass distributed along the arms.

### Additional hardware changes

Prototype_2 also introduces the following weight and packaging reductions:

- removal of the **Pi Camera**
- removal of the **Pi Camera mount**
- replacement of the Pi camera path with an **analogue FPV camera**
- use of an **analogue FPV transmitter**
- replacement of the heavier **7-inch Waveshare HDMI display** with a lighter **3.5-inch GPIO LCD**
- cleaner centralised power and signal layout

### Companion-side power architecture

The updated Prototype_2 power architecture is:

- **4S 1550 mAh 100C** battery for the main propulsion system
- **Pixhawk PM02** for Pixhawk power and power measurement
- **3S 550 mAh 95C** battery for the companion-electronics rail
- **5V 15A UBEC** supplying:
  - Raspberry Pi 5 8GB
  - 3.5-inch GPIO LCD
  - Servo 1
  - Analogue FPV TX

The analogue FPV camera feeds the analogue FPV transmitter directly, while the Raspberry Pi remains connected to Pixhawk by **MAVLink**.

### Important technical clarification

Prototype_2 should not be described as producing more lift *because the new ESC is 65A*.  
The ESC itself does not create thrust.

Instead, the correct engineering explanation is that Prototype_2 can provide:

- **higher effective thrust-to-weight ratio**
- **more usable payload margin**
- **lower hover thrust requirement**
- **lower hover power demand**
- **cleaner aerodynamic packaging**
- **lower distributed mass on the arms**
- **reduced electrical losses in the wiring layout**

These improvements arise from the **integrated system redesign**, not from ESC current rating alone.

---

## Why Prototype_2 is theoretically better than Prototype_1

### 1. Lower all-up mass improves thrust-to-weight ratio

If total available thrust remains approximately the same, but aircraft mass decreases, then thrust-to-weight ratio increases.

\[
\text{TWR} = \frac{T_{total}}{W}
\]

where:

- \(T_{total}\) = total available thrust
- \(W = mg\) = aircraft weight

If the new aircraft mass is:

\[
m_{new} = m_{old} - \Delta m
\]

then:

\[
W_{new} = (m_{old} - \Delta m)g
\]

and the new thrust-to-weight ratio becomes:

\[
\text{TWR}_{new} = \frac{T_{total}}{(m_{old} - \Delta m)g}
\]

This means that any reduction in structure, wiring, display mass, camera mass, or mounting hardware directly increases the effective thrust margin.

### 2. Payload margin improves approximately by the amount of structural mass removed

If the propulsion system is unchanged, then reducing empty structural mass effectively frees part of that mass budget for payload or for improved safety margin.

If the empty aircraft mass is reduced by \(\Delta m\), then the theoretical payload margin increases by roughly the same order:

\[
\Delta m_{payload} \approx \Delta m
\]

This is one of the strongest theoretical advantages of Prototype_2.

### 3. Hover thrust requirement decreases

For a quadrotor in hover:

\[
T_{hover,total} = W = mg
\]

and thrust per motor is:

\[
T_{hover,motor} = \frac{mg}{4}
\]

So if the aircraft mass is reduced, each motor needs less thrust just to hover.  
This means lower steady-state loading in hover and more reserve for climb, disturbance rejection, and payload handling.

### 4. Ideal hover-induced power decreases with reduced weight

For a rotorcraft, ideal induced power in hover scales approximately with:

\[
P_i \propto \frac{W^{3/2}}{\sqrt{2\rho A_{total}}}
\]

where:

- \(W\) = aircraft weight
- \(\rho\) = air density
- \(A_{total}\) = total rotor disk area

Since the propellers remain the same, \(A_{total}\) is approximately unchanged. Therefore:

\[
\frac{P_{i,new}}{P_{i,old}} =
\left(\frac{W_{new}}{W_{old}}\right)^{3/2}
\]

This means even a modest reduction in aircraft weight can produce a meaningful reduction in ideal hover power demand.

### 5. Aerodynamic cleanliness improves

Prototype_1 used separate ESCs and associated wiring distributed outward along the arms.  
Prototype_2 centralises the ESC stage into a compact 4-in-1 board, which reduces the exposed component volume on the arms.

Parasite drag can be represented as:

\[
D_p = \frac{1}{2}\rho V^2 C_D A
\]

where:

- \(D_p\) = parasite drag
- \(\rho\) = air density
- \(V\) = forward speed
- \(C_D\) = drag coefficient
- \(A\) = effective frontal area

By reducing arm-mounted bulk, wire clutter, and external protrusions, Prototype_2 lowers the effective external drag area.  
This does not usually create a dramatic thrust increase in hover, but it can improve forward-flight cleanliness and slightly reduce unnecessary drag losses.

### 6. Rotational inertia around roll and pitch axes is reduced

One of the most important improvements is that mass is removed from the arms, which are far from the vehicle centre.

Rotational inertia is:

\[
I = \sum m r^2
\]

where:

- \(m\) = mass element
- \(r\) = distance from the centre of rotation

Mass removed from the arms has a disproportionately large effect because \(r\) is large.  
Therefore, moving from four separate arm-mounted ESCs to one central 4-in-1 ESC reduces roll/pitch inertia more effectively than removing the same mass near the centre.

This improves:

- attitude responsiveness
- control authority
- transient efficiency during manoeuvres
- dynamic stability under payload shifts

### 7. Electrical losses are reduced by shorter current paths

Resistive wiring losses follow:

\[
P_{loss} = I^2 R
\]

with resistance approximately:

\[
R = \rho_e \frac{L}{A_c}
\]

where:

- \(\rho_e\) = electrical resistivity
- \(L\) = conductor length
- \(A_c\) = conductor cross-sectional area

By shortening wire runs and removing additional connector interfaces, Prototype_2 reduces the total resistive path length and connector-related losses.

This can improve:

- electrical efficiency
- neatness of power distribution
- voltage stability under transient loading
- reliability of the propulsion layout

### 8. Companion-side mass and volume are also improved

Prototype_2 removes two relatively bulky subsystems from Prototype_1:

- Pi Camera + mount
- 7-inch Waveshare HDMI display

and replaces them with:

- analogue FPV camera
- lighter 3.5-inch GPIO LCD

This gives several theoretical advantages:

- lower all-up weight
- lower front-mounted mass
- lower frontal area
- cleaner companion-electronics packaging
- potentially lower compute-side overhead, since the Pi is no longer required to operate the same camera path in that hardware configuration

---

## Prototype_2 software note

The **software logic remains unchanged** in architecture and mission purpose.  
Prototype_2 is therefore a **hardware-refined version**, not a software redesign.

The repository still contains **three software versions**:

### Software Version 1
- Manual takeoff
- Mission started from GUI after stable takeoff
- Manual landing

### Software Version 2
- GUI-triggered mission
- Automatic takeoff
- Automatic mission execution
- Automatic landing

### Software Version 3
- Mission prepared first in GUI
- Manual takeoff
- Mission begins automatically after the aircraft reaches 4 metres altitude
- Manual landing

Thus, Prototype_2 mainly represents a **lighter, cleaner, more integrated hardware implementation** of the same software family.

---

## Correct academic conclusion for Prototype_2

Prototype_2 should be described as a **structural, electrical, and aerodynamic refinement** of Prototype_1.

Its expected advantages are not primarily due to the ESC’s higher current rating alone, but due to the following combined system effects:

- reduced aircraft mass
- reduced arm-mounted distributed mass
- reduced wiring and connector mass
- reduced parasite drag from cleaner arm geometry
- lower rotational inertia
- reduced electrical path losses
- lighter companion-electronics package
- improved effective thrust-to-weight ratio
- improved usable payload margin

Therefore, the academically correct statement is:

> Prototype_2 does not generate additional lift because of ESC rating alone.  
> Instead, it improves the aircraft’s effective lifting capability and efficiency by reducing structural mass, reducing external clutter, centralising propulsion electronics, and lowering the non-propulsive burden carried by the frame.
