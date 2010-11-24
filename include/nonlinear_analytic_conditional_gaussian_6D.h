  #ifndef POSE_ESTIMATION_NONLINEAR_ANALYTIC_CONDITIONAL_GAUSSIAN_6D_H
  #define POSE_ESTIMATION_NONLINEAR_ANALYTIC_CONDITIONAL_GAUSSIAN_6D_H
  
  #include <pdf/analyticconditionalgaussian_additivenoise.h>
  #include <wrappers/rng/rng.h> // Wrapper around several rng libraries
  
  #define NUMCONDARGUMENTS 2
  
  namespace BFL
  {
    
    class NonLinearAnalyticConditionalGaussian6D : 
    public AnalyticConditionalGaussianAdditiveNoise
    {
      public:
	
	NonLinearAnalyticConditionalGaussian6D(const Gaussian& additiveNoise);
	
	virtual ~NonLinearAnalyticConditionalGaussian6D();
	
	// redefine virtual functions
	virtual MatrixWrapper::ColumnVector    ExpectedValueGet() const;
	virtual MatrixWrapper::Matrix          dfGet(unsigned int i) const;
    };
    
  } // End namespace BFL
  
  #endif
  
