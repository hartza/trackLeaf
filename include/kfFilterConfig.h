#ifndef FILTERCONFIG_H
#define FILTERCONFIG_H
  
  // **** Sizes
  
  const int STATE_SIZE = 4;                       //state: [a, b, c, d]
  const int INPUT_SIZE = 4;                       //input: [a, b, c, d]
  const int MEAS_SIZE  = 4;         //measurment: [a, b, c, d]
  
  // **** System noise median
  
  const double MU_SYSTEM_NOISE_A     = 0.0;
  const double MU_SYSTEM_NOISE_B     = 0.0;
  const double MU_SYSTEM_NOISE_C     = 0.0;
  const double MU_SYSTEM_NOISE_D  = 0.0;
  
  // **** System noise covariance
  
  const double SIGMA_SYSTEM_NOISE_A     = 0.001;
  const double SIGMA_SYSTEM_NOISE_B     = 0.001;
  const double SIGMA_SYSTEM_NOISE_C     = 0.001;
  const double SIGMA_SYSTEM_NOISE_D     = 0.001;
  
  // **** Measurement noise median
  
  const double MU_MEAS_NOISE_A     = 0.0;
  const double MU_MEAS_NOISE_B     = 0.0;
  const double MU_MEAS_NOISE_C     = 0.0;
  const double MU_MEAS_NOISE_D     = 0.0;
  
  // **** Measurement noise covariance
  
  const double SIGMA_MEAS_NOISE_A     = 0.1;
  const double SIGMA_MEAS_NOISE_B     = 0.1;
  const double SIGMA_MEAS_NOISE_C     = 0.1;
  const double SIGMA_MEAS_NOISE_D     = 0.1;
  
  // **** Prior median
  
  const double MU_PRIOR_A     = 0.0;
  const double MU_PRIOR_C     = 0.0;
  const double MU_PRIOR_B     = 0.0;
  const double MU_PRIOR_D  = 0.0;
  
  // **** Prior covariance
  
  const double SIGMA_PRIOR_A     = 100.0;
  const double SIGMA_PRIOR_B     = 100.0;
  const double SIGMA_PRIOR_C     = 100.0;
  const double SIGMA_PRIOR_D  = 100.0;
  
#endif
  
  