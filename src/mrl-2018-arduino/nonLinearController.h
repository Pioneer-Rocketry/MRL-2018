
#ifndef NONLINEARCONTROLLER_H
#define NONLINEARCONTROLLER_H

class nonLinearController{
public: 

//varibles for calculating V1 
  static double integrator_X_history[2];
  static double diff_X_history[6];
  
// varibles for calculating V2
  static double integrator_Y_history[2];
  static double diff_Y_history[6];
  
// varibles for calculating V3
  static double integrator_Z_history[2];
  static double diff_Z_1_history[6];
  static double diff_Z_2_history[6];
  
// current linear states
  static double eta[3];
  
// current B matrix
  static double B_matrix[3];
  
// current A matrix
  static double Ainverse[9];
// controller Gains
  static double k1[4];
  static double k2[4];

// This is the main function to be called after sampling
// it will go through with current measurments and find
// the required control efferet needed.(in radians)
// INPUT:  e_X - euler angle about the x-axis
//         e_Y - euler angle about the y-axis
//         e_Z - euler angle about the z-axis
//         w_X - angular velocity about the x-axis
//         w_Y - angular velocity about the y-axis
//         w_Z - angular velocity about the z-axis
// *assume zero        e_X_SP - the set point(value to track) for the eular X-axis
// *assume zero        e_Y_SP - the set point(value to track) for the eular X-axis
//         e_Z_SP - the set point(value to track) for the eular X-axis
//         Ts - sampling time
static void controlEffert( double e_X, double e_Y, double e_Z, double w_X,
                           double w_Y, double w_Z,
                           double e_Z_SP, double Ts, double velocity, double * con_effert );

private:

// current trig calculations
  static double cos_x2;
  static double sin_x2;
  static double tan_x2;
  static double cos_x3;
  static double sin_x3;





  
    
  // This function will preform a efficent differentiator and
  // DOES NOT UPDATE THE PAST INPUTS OF THE SIGNAL
  // Input: x_n - current input
  //        xHistory[] - array of size six that has the last
  //                     6 data points
  static double differentiator(double x_n, double *xHistory);
    

  // this function implements a simpson's integrator (IIR)
  // and will update the history of the  input 
  // DOES NOT UPDATE THE PAST INPUTS OF THE INPUT signal
  // Input: x_n - current input value
  //        xHistory - the past two values of the input
  //        yHistory - the past two values of the output
  //        t_sample - the sampling time of the system
  // Output: y_n - the current output value of the integrator
  static double simpsonIntegrator( double x_n, double t_sample,
                            double *xHistory, double *yHistory);

                            
  // This function will shift and add in the current value into 
  // the history array that is passed to it.
  // INPUT:  x_n - current value_comp
  // OUTPUT: modifies array 
  static void updateHistory( double x_n, double *history, int length);


                            
  //Calculate the value of the linearized states that are not online
  // INPUT:  x2 - the euler angle about the y-axis
  //         x3 - the euler angle about the z-axis
  //         x4 - the angular velocity about the x-axis
  //         x5 - the angular velocity about the y-axis
  //         x6 - the angular velocity about the z-axis
  // OUTPUT: out1 - 3 element aray that contians the second linearized state for each 
  //                subsystem
  static void phiPart(double x2, double x3, double x4, double x5, double x6, double *out1);


  // calculate the current value of the inverse of the decoupling matrix
  // INPUTS:  v - the current velocity SQUARED
  //          x2 - the euler angle about the y-axis
  //          x3 - the euler angle about the z-axis
  // OUTPUTS: out1 - an array of length 9 that contains a 3x3 matrix
  //                 split as the first COLUMN in the first three indexs
  //                 followed by the elements of the second column leaving 
  //                 the last three elements of the array being the last column
  static void Ainv(double v, double x2, double x3, double out1[9]);


  // this function will calculate the current B matrix 
  // INPUT:  x2 - the euler angle about the y-axis
  //         x3 - the euler angle about the z-axis
  //         x4 - the angular velocity about the x-axis
  //         x5 - the angular velocity about the y-axis
  //         x6 - the angular velocity about the z-axis
  // OUTPUT: 
  static void Bmax(double x2, double x3, double x4, double x5, double x6, double *Bx);




};//end of class Controller

#endif
