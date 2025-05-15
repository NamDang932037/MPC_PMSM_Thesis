# MPC_PMSM_Thesis
This is my undergraduate thesis project, which focuses on implementing a Model Predictive Control (MPC) algorithm for trajectory tracking of position and velocity on a Permanent Magnet Synchronous Motor (PMSM). The control is based on a mechanical model of the PMSM.
The project successfully integrates the following components on the STM32H747 microcontroller:
MPC for trajectory tracking
Field-Oriented Control (FOC) for current regulation
Space Vector Pulse Width Modulation (SVPWM) for inverter control

✅ Achieved Results:
Maximum position tracking error: 2 degrees
Maximum steady-state position error: 0.25 degrees
Velocity tracking RMS error: under 5 rpm

This repository contains the full implementation and related materials for the project.

# S-Curve Trajectory
![Pos_SCurved](https://github.com/user-attachments/assets/267ad7a0-36c9-4b51-8081-252a84105c23)

![Vel_SCurved](https://github.com/user-attachments/assets/26f3dc91-2973-4a04-92e4-d0d639cbb7d2)

![Torque_Scurved_Real](https://github.com/user-attachments/assets/795f8766-475d-4bb4-9f1d-17c40e15bb7a)


# LSPB Trajectory
![Pos_LSPB](https://github.com/user-attachments/assets/90034016-bc4d-4b86-a6da-e9e4db638e2d)

![Vel_LSPB](https://github.com/user-attachments/assets/c6933c3a-afcb-4818-b038-61ad650ed30c)

![Te_LSPB](https://github.com/user-attachments/assets/4d25a41f-ed74-4321-a788-3a45851d22d2)


# Sin Trajectory
![Sine_Pos](https://github.com/user-attachments/assets/c156894b-75c2-4df9-99f7-b890c18f1512)

![Sine_Vel](https://github.com/user-attachments/assets/74bf6663-e9c1-4609-97df-481e4a33a9b8)

![Sine_Torque](https://github.com/user-attachments/assets/b632fe9d-3bea-494a-97b5-c1adf472ef7a)

# Step Trajectory
![Step_Velo](https://github.com/user-attachments/assets/290fbc58-aa04-49ba-952c-c806b6644571)

![StepTorque](https://github.com/user-attachments/assets/c15f93e1-dd16-47dd-bc1f-5ecde4c21d07)
