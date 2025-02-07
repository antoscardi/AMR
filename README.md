# Autonomous Mobile Robots (AMR) Project

## Overview
This is the Git repository for the **Autonomous Mobile Robots (AMR) project**, developed by **Antonio Scardino, Niccolò Piraino, and Marco Nacca**. The project is based on the principles and methodologies presented in the paper:

[Trajectory Generation for Minimum Closed-Loop State Sensitivity](https://ieeexplore.ieee.org/document/8460546)

Our implementation aims to **reduce parametric variation disturbances** affecting a **differential drive unicycle** controlled via **Dynamic Feedback Linearization (DFL)**. By optimizing the robot's trajectory, we minimize both the state and controller sensitivities, leading to more robust control and improved tracking accuracy.

---

## Project Goals
- Implement a trajectory optimization algorithm that minimizes closed-loop state sensitivity.
- Apply Dynamic Feedback Linearization (DFL) to control a differential drive robot.
- Reduce the impact of parametric uncertainties (e.g., wheel radius and distance variations) on robot performance.
- Compare results for different types of trajectories with and without integral action.
- Validate the approach using statistical analysis and Monte Carlo simulations.

---

## Implementation Details
### **1. Problem Formulation**
The optimization is formulated as follows:
- The system dynamics are modeled as a nonlinear state-space system:
  \[ \dot{q} = f(q, u, p) \]
- The control input follows a Dynamic Feedback Linearization (DFL) approach to track a reference trajectory \( r_d(t) \).
- Sensitivity matrices \( \Pi(t) \) are computed to measure the deviation of the system state due to parameter variations.
- The trajectory coefficients are iteratively updated using **gradient descent** to minimize the sensitivity matrix norm.

### **2. Optimization Process**
- The trajectory is generated using polynomial splines with **continuity constraints**.
- The **sensitivity dynamics** are integrated numerically.
- The **cost function** is defined as:
  \[ \text{Loss} = \frac{1}{2} \text{Tr}(\Pi^T(t_f) \Pi(t_f)) \]
- The optimization loop adjusts the trajectory coefficients to minimize this cost function.
- The results are evaluated in both **nominal** and **perturbed** scenarios.

### **3. Statistical Analysis**
- Monte Carlo simulations are used to evaluate robustness.
- Parameter variations are randomly sampled within **\[80\%, 120\%\]** of nominal values.
- The **mean and standard deviation** of errors are computed across simulations.
- Optimization performance is assessed based on error reduction.

---

## Results
### **Trajectory Examples**
#### **First Trajectory (Without Integral Action)**
![First Trajectory](images/first_trajectory.png)
- A beveled trajectory designed to emphasize the optimization effect.
- The optimization significantly reduces the **position error** in the perturbed case.
- **Error reduction:** From **3.66 cm** to **0.55 cm** (improvement: **3.11 cm**).

#### **Second Trajectory (With Integral Action)**
![Second Trajectory](images/second_trajectory.png)
- A different curvilinear trajectory.
- The integral action reduces steady-state error, further improving robustness.
- **Error reduction:** From **4.64 cm** to **3.70 cm** (improvement: **0.94 cm**).

### **Loss Function Convergence**
- The **gradient descent optimization** reduces the loss function progressively.
- The process stabilizes within **~55 iterations**.

### **Statistical Performance**
- **Monte Carlo results** confirm that the optimized trajectories outperform unoptimized ones across multiple parameter perturbations.
- **Position error reduction:** Up to **6.7 times smaller** in the optimized case.
- **Orientation error reduction:** Significant improvements, particularly in the second trajectory.

---

## How to Use
### **1. Setup**
Clone the repository and install required dependencies:
```bash
git clone https://github.com/your-repo/AMR_project.git
cd AMR_project
pip install -r requirements.txt  # If applicable
```

### **2. Running the Simulation**
Run the MATLAB script to execute the optimization and control simulation:
```matlab
run('AMR_simulation.m')
```

### **3. Visualization**
Plots of trajectories, error evolution, and sensitivity matrices are automatically generated.

---

## Demonstration Videos
Check out our **YouTube demonstration videos** showcasing the performance of the optimized trajectories:
- **First Trajectory Optimization:** [![YouTube](https://img.shields.io/badge/YouTube-First_Trajectory-red?style=flat&logo=youtube)](https://www.youtube.com/your_video_link_1)
- **Second Trajectory Optimization:** [![YouTube](https://img.shields.io/badge/YouTube-Second_Trajectory-red?style=flat&logo=youtube)](https://www.youtube.com/your_video_link_2)

---

### 👥 Project Contributors

-  [Marco Nacca](https://github.com/marconacca)
-  [Niccolò Piraino](https://github.com/Nickes10)
-  [Antonio Scardino](https://github.com/antoscardi)

---
