# Quadrotor with Suspended Payload - Simulation Instructions

This repository contains the MATLAB/Simulink files to simulate the quadrotor-payload system using two different control strategies: Geometric Control and NFT-Sliding Mode Control. 

Follow the instructions below to generate plots, videos, and sensitivity tables.

---

## 1. Generating Simulation Plots

### A. Geometric Control
Open the `geometric_control_quadrotor_payload` folder and run the following commands in the MATLAB Command Window:

**1. Without disturbances:**
```matlab
data;
trajectory_smooth;
out = sim('geometric_control.slx');
plots;
```

**2. With disturbances:**
```matlab
data; 
disturbances;
trajectory_smooth;
out = sim('geometric_control_pertub.slx');
plots;
```

### B. Sliding Mode Control
Open the `NFTSlide_mode_control` folder and run the following commands in the MATLAB Command Window:

**1. Without disturbances:**
```matlab
data;
trajectory_smooth;
out = sim('slide_mode_control.slx');
plots;
```

**2. With disturbances:**
```matlab
data;
disturbances;
trajectory_smooth;
out = sim('slide_mode_control_with_pertu.slx');
plots;
```

---

## 2. Generating Simulation Videos

To generate 3D simulation videos, repeat the exact same steps and commands described in **Section 1**, but replace the `plots` command with `video_generator` at the end of the sequence. 

*Example for Geometric Control (without disturbances):*
```matlab
data;
trajectory_smooth;
out = sim('geometric_control.slx');
video_generator;
```

---

## 3. Generating Sensitivity Tables

To automatically generate the tables evaluating the controller's sensitivity, navigate to either the Geometric or Sliding Mode folder and run the following scripts in the Command Window:

*   **Parametric sensitivity table (without disturbances):**
    ```matlab
    variation_params;
    ```
*   **Disturbances sensitivity table (with nominal `data.m` parameters):**
    ```matlab
    scenarios_dist;
    ```

---

## 4. Customizing Parameters (Notes)

If you wish to manually evaluate parametric or disturbance sensitivities by generating custom plots or videos, you can tweak the system variables before running the simulations:

*   **Parametric Sensitivity:** Open `data.m` and vary the payload mass `m_L_plant` (named `m_plant` in the `slide_mode_control` folder), the cable stiffness `k_plant`, or the damping coefficient `c_plant`.
*   **Disturbance Sensitivity:** Open `disturbances.m` and vary the perturbation parameters (e.g., `Awind`, `Awind_0`, `A_tau`, `tau_phi_0`, `tau_theta_0`, `tau_psi_0`).