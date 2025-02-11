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
A beveled trajectory designed to emphasize the optimization effect.
#### **📍 Second Trajectory**
<p align="center">
<img src='https://github.com/user-attachments/assets/3a0d2c21-235d-4f44-b44a-c64daaf57c59' width='60%'>
</p>

A different curvilinear trajectory where integral action reduces steady-state error, further improving robustness.

### **📉 Loss Function Convergence**
Gradient descent optimization progressively reduces the loss function, stabilizing within **~55 iterations**.

### **📊 Statistical Performance**
Optimized trajectories consistently outperform unoptimized ones, effectively mitigating parameter variations and reducing error. Across multiple simulations with varying perturbations, the optimization maintains a high success rate regarding the error with respect to the final pose of the robot.

---

# 🛠️ How to Use

## **1️⃣ Setup**
Clone the repository and initialize the required settings:

```bash
git clone https://github.com/antoscardi/AMR.git
```

Then, open MATLAB and run the setup script:

```matlab
run('run_once_settings.m')
```

## **2️⃣ Running the Simulation**
To execute the optimization and control simulation, run the following MATLAB scripts:

```matlab
run('c_IDEAL.m')
```

```matlab
run('c_PERTURBED_OPTIMIZED.m')
```
---

These scripts simulate the system under ideal and perturbed conditions, respectively.

## 🎥 Demonstration Videos
Check out our **YouTube demonstration videos** showcasing the performance of the optimized trajectories, with integral action.

- **First Trajectory Optimization:**

![traj_2](https://github.com/user-attachments/assets/c8c478e6-eff3-4e58-8aad-124fe044ad16)


- **Second Trajectory Optimization:**

![traj_1](https://github.com/user-attachments/assets/351291e4-eba6-47c7-bb0a-85790ae5df7d)


---

### 👥 Project Contributors

- [Marco Nacca](https://github.com/marconacca)  
- [Niccolò Piraino](https://github.com/Nickes10)  
- [Antonio Scardino](https://github.com/antoscardi)  

---
