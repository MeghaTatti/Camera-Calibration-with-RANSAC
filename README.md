# Camera-Calibration-with-RANSAC
This project is about the calibration of camera - "Non planar Calibration" and using RANSAC algorithm for strong estimation.

## Project Description:
The project extracts feature points from the calibration target and displays them on the image using its respective function.
The calibration process is implemented from a text file (3D points) and its corelated 2D points. The program shows both intrinsic and extrinsic parameters given the correspondence between image and world along with the mean square error between the computed positions of the image points.RANSAC algorithm is implemented for strong estimation which should specify the automatic estimate of the number of draws and the probability where a data point will be an inlier.

## Implementation details
The implementation of both parts is as follows:

The first program creates the points and writes it into two different files.
The data being generated will be used by the second program and performs the calibration process and outputs the intrinsic and extrinsic parameters and mean square error.
I programmed the equations and everything as I have explained previously.
Since I use the functions from the part two in the RANSAC algorithm the parameters I get are not the same as the ones the professor posted online but the algorithm itself should be right.
As parameter I used n = 9, just some more then the 6 needed, and for the loops I realized that doing more did not improve the results so I just left 3. However, this could be wrong in part because of the parameters.

