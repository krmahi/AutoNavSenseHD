

# SenseDrive

SenseDrive is an open-source initiative aimed at advancing autonomous vehicle localization and navigation to achieve Autonomous Driving Level 5 (AD LEVEL 5). By leveraging onboard sensors and high-definition geolocated maps, the project aspires to develop robust solutions for seamless navigation in complex environments.

---

## 🚗 Key Features

- **Lidar-Based SLAM**:Implements Lidar Simultaneous Localization and Mapping (SLAM) for real-time 3D environmental mapping
- **Visual SLAM**:Utilizes camera data to perform Visual SLAM, enhancing perception in GPS-denied environments
- **Sensor Fusion**:Combines data from multiple sensors to improve localization accuracy and reliability
- **Traffic Light Negotiation**:Incorporates logic for interpreting and responding to traffic signals
- **Lane Following Controller**:Provides algorithms for maintaining lane discipline under various driving conditions
- **Simulation Tools**:Includes Simulink models and test benches for validating algorithms in simulated environments

---

## 📁 Repository Structure
The repository is organized as follow:

- **Lidar_SLAM/** Contains modules related to Lidar-based SLA.
- **Visual_SLAM/** Houses components for Visual SLAM implementatio.
- **sensorfusion/** Includes scripts and models for sensor data fusio.
- **traffic_light_negotiation/** Comprises logic for handling traffic light interaction.
- **slprj/** Generated files from Simulink project.
- **Simulink Models**:
  - `LaneFollowingController.slxc`
  - `LidarSLAMIn3DSimulation.slxc`
  - `TLNWithUnrealTestBench.slxc`
  - `TrafficLightDecisionLogic.slxc`
  - `localizeAndControlUsingLidar.slxc`
  - `localizeUsingLidar.slxc`
- **Map Data**:
  - `lidarmap.fig`
  - `lidarmap.jpg`
  - `lidarmap.png`
- **Utilities**:
  - `GenerateImageDataOfParkingLot.slxc` Tool for generating parking lot image dat.

---

## 🛠️ Getting Started

1. **Prerequisites**:
  - MATLAB with Simulink suppot.
  - Toolboxes for Robotics, Computer Vision, and Automated Drivig.

2. **Installation**:
  - Clone the repositoy:
     ```bash
     git clone https://github.com/krmahi/SenseDrive.git
     ```

3. **Usage**:
  - Open desired `.slxc` models in Simulik.
  - Run simulations to test specific moduls.

---

## 📄 Licene

This project is licensed under the [Apache 2.0 License](https://github.com/krmahi/SenseDrive/blob/main/LICENE).

---

## 🤝 Contributng

Contributions are welcome! Please fork the repository and submit a pull request. For major changes, open an issue first to discuss proposed modificatons.

---

## 📬 Conact

For questions or collaborations, reach out to Mahesh Kumar at [mahesh.kr.2277@gmail.com](mailto:mahesh.kr.2277@gmailcom).
---

*Note: This README is based on the current structure and available information in the repository. As the project evolves, please update this document accordigly.*

--- 
