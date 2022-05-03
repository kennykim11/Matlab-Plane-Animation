# Matlab Plane Animation

This MATLAB class can take the outputs from RIT MECE-410 (Flight Dynamics) Project 4 and animate the aircraft motion using the resulting data in a 3D plot. It will help students visualize the meaning of the various plots and gain an intuitive understanding of the equations of motion.

Created by Kenny Kim, Brian Le, and Tyler Rodgers in Spring 2022.

![Alt Text](https://raw.githubusercontent.com/kennykim11/Matlab-Plane-Animation/main/ExtremeInput.gif)

## Usage
Place `PlaneAnimation.m` in the same folder as the Project 4 script and insert the following line of code. Change some of the variables/data types if need be.
```PlaneAnimation.run(Vt, alpha, beta, phi, theta, psi, time, "Extreme Input", false);```

## Parameters:
All vectors should have the same length.
 - `Vt`: Vector of true velocity data
 - `alpha`: Vector of angle of attack data in degrees
 - `beta`: Vector of sideslip data in degrees
 - `phi`: Vector of bank angle data in degrees
 - `theta`: Vector of pitch angle data in degrees
 - `psi`: Vector of heading angle data in degrees
 - `time`: Vector of time steps in seconds
 - `plotTitle`: String of what to make the title of the plot
 - `exportVideo`: Boolean of whether frames of animation should be captured and saved in a .avi file

## Note:
The equations used in this class still need double-checking. Accuracy is not yet guarenteed.
