# 📈 Trajectory Generation for Minimum Closed-Loop State Sensitivity

## 📌 Overview
This is the repository for the project relative to the **Autonomous Mobile Robots (AMR)** course, taught by **Prof. Giuseppe Oriolo** at **La Sapienza University of Rome**. The project is based on the principles and methodologies presented in this 📄 [paper (P. R. Giordano et al.)](https://ieeexplore.ieee.org/document/8460546).

Our implementation aims to **reduce parametric variation disturbances** affecting a **differential drive unicycle** controlled via **Dynamic Feedback Linearization (DFL)**. By optimizing the robot's trajectory, we minimize both the state and controller sensitivities, leading to more robust control and improved tracking accuracy.

<p align="center">
<img width="250" alt="unicycle" src="https://github.com/user-attachments/assets/105522f9-d786-42dd-81c4-22eb072a7276" />
</p>

---

## 🎯 Project Goals
This project focuses on implementing a trajectory optimization algorithm that minimizes closed-loop state sensitivity. The approach applies **Dynamic Feedback Linearization (DFL)** to control a differential drive robot and reduces the impact of parametric uncertainties, such as wheel radius and distance variations. Different types of trajectories are compared, both with and without integral action, and the approach is validated using statistical analysis.

---

## 🔍 Implementation Details
### **1️⃣ Problem Formulation**
The system dynamics are modeled as a nonlinear state-space system:
```math
\dot{q} = f(q, u, p)
```
The control input follows a **Dynamic Feedback Linearization (DFL)** approach to track a reference trajectory $r_d(t)$. Sensitivity matrices $\Pi(t)$ are computed to measure deviations in the system state due to parameter variations. The trajectory coefficients are iteratively updated using **gradient descent** to minimize the sensitivity matrix norm.

### **2️⃣ Optimization Process**
Polynomial splines with **continuity constraints** are used to generate the trajectory. The **sensitivity dynamics** are integrated numerically, and the **cost function** is defined as:
```math
\text{Loss} = \frac{1}{2} \text{Tr}(\Pi^T(t_f) \Pi(t_f))
```
The optimization process adjusts the trajectory coefficients to minimize this cost function, with results evaluated in both **nominal** and **perturbed** scenarios.

### **3️⃣ Statistical Analysis**
Simulations evaluate robustness by randomly sampling parameter variations within **\[80\%, 120\%\]** of nominal values. The **mean and standard deviation** of errors are computed across simulations, and optimization performance is assessed based on error reduction.

---

## 📊 Results
### **📌 Trajectory Examples**
#### **📍 First Trajectory**
<p align="center">
<img src='https://github.com/user-attachments/assets/ddf4a7f8-ead7-49f8-ab56-22082a140531' width='50%'>
</p>
A beveled trajectory designed to emphasize the optimization effect. The optimization significantly reduces the **position error** in the perturbed case, decreasing from **3.66 cm** to **0.55 cm** (improvement: **3.11 cm**).

#### **📍 Second Trajectory**
<p align="center">
<img src='https://github.com/user-attachments/assets/3a0d2c21-235d-4f44-b44a-c64daaf57c59' width='60%'>
</p>

A different curvilinear trajectory where integral action reduces steady-state error, further improving robustness. The error is reduced from **4.64 cm** to **3.70 cm** (improvement: **0.94 cm**).

### **📉 Loss Function Convergence**
Gradient descent optimization progressively reduces the loss function, stabilizing within **~55 iterations**.

### **📊 Statistical Performance**
Results confirm that optimized trajectories outperform unoptimized ones across multiple parameter perturbations. Position error reduction is up to **6.7 times smaller**, with significant improvements in orientation error reduction, particularly in the second trajectory.

---

## 🛠️ How to Use
### **1️⃣ Setup**
Clone the repository and install required dependencies:
```bash
git clone https://github.com/your-repo/AMR_project.git
cd AMR_project
pip install -r requirements.txt  # If applicable
```

### **2️⃣ Running the Simulation**
Run the MATLAB script to execute the optimization and control simulation:
```matlab
run('AMR_simulation.m')
```

### **3️⃣ Visualization**
Plots of trajectories, error evolution, and sensitivity matrices are automatically generated.

---

## 🎥 Demonstration Videos
Check out our **YouTube demonstration videos** showcasing the performance of the optimized trajectories, with integral or NOT?????...:
- **First Trajectory Optimization:** [![YouTube](https://img.shields.io/badge/YouTube-First_Trajectory-red?style=flat&logo=youtube)](https://www.youtube.com/your_video_link_1)
- **Second Trajectory Optimization:** [![Watch the video](https://img.youtube.com/vi/m4e6AeSGSPI/maxresdefault.jpg)](https://youtu.be/m4e6AeSGSPI)

---

### 👥 Project Contributors

- [Marco Nacca](https://github.com/marconacca)  
- [Niccolò Piraino](https://github.com/Nickes10)  
- [Antonio Scardino](https://github.com/antoscardi)  

---
