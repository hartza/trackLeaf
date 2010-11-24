  #include "kfFilter.h"
  
  PoseEstimationFilter::PoseEstimationFilter():
  staticSysNoiseMu(STATE_SIZE),
  staticSysNoiseCov(STATE_SIZE),
  H(MEAS_SIZE,STATE_SIZE),
  staticMeasNoiseMu(MEAS_SIZE),
  staticMeasNoiseCov(MEAS_SIZE),
  priorMu(STATE_SIZE),
  priorCov(STATE_SIZE)
  {
    
    // **** Linear system model
      // Create the matrix A for the linear system model
  BFL::Matrix A(4,4);
  A(1,1) = 1.0;
  A(1,2) = 0.0;
  A(1,3) = 0.0;
  A(1,4) = 0.0;
  A(2,1) = 0.0;
  A(2,2) = 1.0;
  A(2,3) = 0.0;
  A(2,4) = 0.0;
  A(3,1) = 0.0;
  A(3,2) = 0.0;
  A(3,3) = 1.0;
  A(3,4) = 0.0;
  A(4,1) = 0.0;
  A(4,2) = 0.0;
  A(4,3) = 0.0;
  A(4,4) = 1.0;
/*   BFL:: Matrix B(2,2);
  B(1,1) = cos(0.8);
  B(1,2) = 0.0;
  B(2,1) = sin(0.8);
  B(2,2) = 0.0;

  BFL::vector<BFL::Matrix> AB(4);
  AB[0] = A;
  AB[1] = B;*/
  // initialize mean and covariance
    initStaticSysModel();
    
    // create the static motion model
    staticSysUncertainty = new BFL::Gaussian(staticSysNoiseMu, staticSysNoiseCov);
    staticSysPdf   = new BFL::LinearAnalyticConditionalGaussian(A,*staticSysUncertainty);
    staticSysModel = new BFL::LinearAnalyticSystemModelGaussianUncertainty(staticSysPdf);
    
    // ****  Initialise measurement model
    
    // create matrix H for linear measurement model
    initH();
    
    // initialize the measurement noise 
    initStaticMeasModel();
    
    // create the static measurement model
    staticMeasUncertainty = new BFL::Gaussian(staticMeasNoiseMu, staticMeasNoiseCov);
    staticMeasPdf = new BFL::LinearAnalyticConditionalGaussian(H, *staticMeasUncertainty);
    staticMeasModel = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(staticMeasPdf);
    
    // **** Prior density 
    
    // Continuous Gaussian prior (for Kalman filters)
    initPriorModel();
    
    priorCont = new BFL::Gaussian(priorMu, priorCov);
    
    // ****  Construction of the filter
    
    ekfFilter = new BFL::ExtendedKalmanFilter(priorCont);
  };
  
  PoseEstimationFilter::~PoseEstimationFilter()
  {
    
  };
  
  void PoseEstimationFilter::updateFromMeasurement(const BFL::ColumnVector& measurement,
						   const BFL::ColumnVector& measNoiseMu,
						   const BFL::SymmetricMatrix& measNoiseCov)
						   {
  // create the measurement model
  BFL::Gaussian                                          * measUncertainty;
  BFL::LinearAnalyticConditionalGaussian                 * measPdf;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty * measModel;

  measUncertainty = new BFL::Gaussian(measNoiseMu, measNoiseCov);
  measPdf = new BFL::LinearAnalyticConditionalGaussian(H, *measUncertainty);
  measModel = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(measPdf);

  // update the filter from measurement
  ekfFilter->Update(measModel, measurement);
}

void PoseEstimationFilter::updateFromMeasurement(const BFL::ColumnVector& measurement)
{
  // update filter with static measurement model
  ekfFilter->Update(staticMeasModel, measurement);
}
						   
void PoseEstimationFilter::updateFromMotion(const BFL::ColumnVector& input,
const BFL::ColumnVector& inputNoiseMu,
const BFL::SymmetricMatrix& inputNoiseCov)
{
/*  // create the motion model
  BFL::Gaussian * sysUncertainty;
  BFL::NonLinearAnalyticConditionalGaussian6D * sysPdf;
  BFL::AnalyticSystemModelGaussianUncertainty * sysModel;
  
  sysUncertainty = new BFL::Gaussian(inputNoiseMu, inputNoiseCov);
  sysPdf   = new BFL::NonLinearAnalyticConditionalGaussian6D(*sysUncertainty);
  sysModel = new BFL::AnalyticSystemModelGaussianUncertainty(sysPdf);
  
  // update the filter from motion
  ekfFilter->Update(sysModel, input);*/
}

void PoseEstimationFilter::updateFromMotion(const BFL::ColumnVector& input)
{
  // update filter with static system model
  ekfFilter->Update(staticSysModel, input);
}

void PoseEstimationFilter::getPosteriorMean(BFL::ColumnVector& mean)
{
  BFL::Pdf<BFL::ColumnVector> * posterior = ekfFilter->PostGet();
  mean = posterior->ExpectedValueGet();
}

void PoseEstimationFilter::getPosteriorCovariance(BFL::SymmetricMatrix& covariance)
{
  BFL::Pdf<BFL::ColumnVector> * posterior = ekfFilter->PostGet();
  covariance = posterior->CovarianceGet();
}

void PoseEstimationFilter::initStaticSysModel()
{
  staticSysNoiseMu(1) = MU_SYSTEM_NOISE_A;
  staticSysNoiseMu(2) = MU_SYSTEM_NOISE_B;
  staticSysNoiseMu(3) = MU_SYSTEM_NOISE_C;
  staticSysNoiseMu(4) = MU_SYSTEM_NOISE_D;
  
  staticSysNoiseCov = 0.0;
  staticSysNoiseCov(1,1) = SIGMA_SYSTEM_NOISE_A;
  staticSysNoiseCov(1,2) = 0.0;
  staticSysNoiseCov(1,3) = 0.0;
  staticSysNoiseCov(1,4) = 0.0;
  
  staticSysNoiseCov(2,1) = 0.0;
  staticSysNoiseCov(2,2) = SIGMA_SYSTEM_NOISE_B;
  staticSysNoiseCov(2,3) = 0.0;
  staticSysNoiseCov(2,4) = 0.0;
  
  staticSysNoiseCov(3,1) = 0.0;
  staticSysNoiseCov(3,2) = 0.0;
  staticSysNoiseCov(3,3) = SIGMA_SYSTEM_NOISE_C;
  staticSysNoiseCov(3,4) = 0.0;
  
  staticSysNoiseCov(4,1) = 0.0;
  staticSysNoiseCov(4,2) = 0.0;
  staticSysNoiseCov(4,3) = 0.0;
  staticSysNoiseCov(4,4) = SIGMA_SYSTEM_NOISE_D;
  
}

void PoseEstimationFilter::initH()
{
  H = 0.0;
  
  H(1,1) = 1.0;
  H(1,2) = 0.0;
  H(1,3) = 0.0;
  H(1,4) = 0.0;
  
  H(2,1) = 0.0;
  H(2,2) = 1.0;
  H(2,3) = 0.0;
  H(2,4) = 0.0;
  
  H(3,1) = 0.0;
  H(3,2) = 0.0;
  H(3,3) = 1.0;
  H(3,4) = 0.0;
  
  H(4,1) = 0.0;
  H(4,2) = 0.0;
  H(4,3) = 0.0;
  H(4,4) = 1.0;

}

void PoseEstimationFilter::initStaticMeasModel()
{
  staticMeasNoiseMu(1) = MU_MEAS_NOISE_A;
  staticMeasNoiseMu(2) = MU_MEAS_NOISE_B;
  staticMeasNoiseMu(3) = MU_MEAS_NOISE_C;
  staticMeasNoiseMu(4) = MU_MEAS_NOISE_D;
  
  staticMeasNoiseCov(1,1) = SIGMA_MEAS_NOISE_A;
  staticMeasNoiseCov(1,2) = 0.0;
  staticMeasNoiseCov(1,3) = 0.0;
  staticMeasNoiseCov(1,4) = 0.0;
  
  staticMeasNoiseCov(2,1) = 0.0;
  staticMeasNoiseCov(2,2) = SIGMA_MEAS_NOISE_B;
  staticMeasNoiseCov(2,3) = 0.0;
  staticMeasNoiseCov(2,4) = 0.0;
  
  staticMeasNoiseCov(3,1) = 0.0;
  staticMeasNoiseCov(3,2) = 0.0;
  staticMeasNoiseCov(3,3) = SIGMA_MEAS_NOISE_C;
  staticMeasNoiseCov(3,4) = 0.0;
  
  staticMeasNoiseCov(4,1) = 0.0;
  staticMeasNoiseCov(4,2) = 0.0;
  staticMeasNoiseCov(4,3) = 0.0;
  staticMeasNoiseCov(4,4) = SIGMA_MEAS_NOISE_D;
}

void PoseEstimationFilter::initPriorModel()
{
  priorMu(1) = MU_PRIOR_A;
  priorMu(2) = MU_PRIOR_B;
  priorMu(3) = MU_PRIOR_C;
  priorMu(4) = MU_PRIOR_D;
  
  priorCov(1,1) = SIGMA_PRIOR_A;
  priorCov(1,2) = 0.0;
  priorCov(1,3) = 0.0;
  priorCov(1,4) = 0.0;
  
  priorCov(2,1) = 0.0;
  priorCov(2,2) = SIGMA_PRIOR_B;
  priorCov(2,3) = 0.0;
  priorCov(2,4) = 0.0;
  
  priorCov(3,1) = 0.0;
  priorCov(3,2) = 0.0;
  priorCov(3,3) = SIGMA_PRIOR_C;
  priorCov(3,4) = 0.0;
  
  priorCov(4,1) = 0.0;
  priorCov(4,2) = 0.0;
  priorCov(4,3) = 0.0;
  priorCov(4,4) = SIGMA_PRIOR_D;
}
