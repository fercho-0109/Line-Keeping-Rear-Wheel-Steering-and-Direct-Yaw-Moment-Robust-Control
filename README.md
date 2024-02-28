# Rear-Wheel-Steering-and-Direct-Yaw-Moment-Robust-LPV-Control

Code developed for "A. Marino, C. Tiriolo, - Active Rear Wheel Steering and Direct Yaw Moment Robust Control using Automated Driving Toolbox".  
Master's student "Calabria University".
For any questions or suggestions write to alexismarino0109@gmail.com
Paper in publication process.


# Abstract.
Considering the demand for vehicle stability control, an integrated control system of active rear-wheel steering (4WS) and direct yaw moment control (DYC) is presented in this paper. The proposed control system is integrated into an LPV model, which makes the vehicle follow the desired trajectory, using the state feedback and feedforward gains computed by H-inf and L1 of yaw rate, side slip angle, and tracking error states. Finally, numerical simulations based on MATLAB/Simulink and Automated Driving Toolbox1 experiments were performed with the proposed control strategy to identify its performance. The simulation and experimental results indicate that the handling stability of the 4WS vehicle is improved by the optimal controllers.


# Prerequisites
- The code was created and tested on the Matlab/Simulink 2023a environment
- Yalmip solver is necessary, use **install_mpt3** file to install 

# File description
The repository contains three files
1. **Matlab**: It contains the main programs and functions to run the program.
2. **install_mpt3**: it is the file used to install Yalmip solver.


# Example to run the experiment  
**"Rear Wheel Steering and Direct Yaw Moment Control"**
### Matlab/Simulink simulation 
1. Download the files or clone the repository. 
2. Open and run the Matlab file "**Model**".
3. Run the simulink file called "**model_line_detector_2**".
4. The Scope blocks should start to show the results in the base of the chosen control.
![image](https://github.com/fercho-0109/Rear-Wheel-Steering-and-Direct-Yaw-Moment-Robust-LPV-Control/assets/40362695/02a752cb-b89d-4233-816f-77db3fb4cae3)


