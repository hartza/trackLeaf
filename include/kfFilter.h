#ifndef EKFFILTER_H
#define EKFFILTER_H

#include <stdio.h>

// **** BFL includes

#include <filter/extendedkalmanfilter.h>

#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>

// **** Other includes
#include "kfFilterConfig.h"

//using namespace MatrixWrapper::;
//using namespace BFL;
//using namespace std;

/**
* encapsulates a kalman filter (by default a extended KF) implmented in bfl library
* code from/inspirated in: 
* * http://people.mech.kuleuven.be/~tdelaet/bfl_doc/getting_started_guide/
* * http://www.ros.org/wiki/ccny-ros-pkg
*/

class PoseEstimationFilter
{
  private:
    
    // **** system model
    
    BFL::ColumnVector       staticSysNoiseMu;
    BFL::SymmetricMatrix staticSysNoiseCov;
    
    BFL::Gaussian * staticSysUncertainty;
    BFL::LinearAnalyticConditionalGaussian * staticSysPdf;
    BFL::AnalyticSystemModelGaussianUncertainty * staticSysModel;
    
    // **** measurement model
    
    BFL::Matrix H;
    
    BFL::ColumnVector    staticMeasNoiseMu;
    BFL::SymmetricMatrix staticMeasNoiseCov;
    
    BFL::Gaussian * staticMeasUncertainty;
    BFL::LinearAnalyticConditionalGaussian                 * staticMeasPdf;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty * staticMeasModel;
    
    // **** prior density
    
    BFL::ColumnVector priorMu;
    BFL::SymmetricMatrix priorCov;
    
    BFL::Gaussian * priorCont;
    
    // **** filter
    
    BFL::ExtendedKalmanFilter * ekfFilter;
    
    void initStaticSysModel();
    
    /**
    * Inicialitza la matriu del model de mesura
    * S'inicialitza diagonal: implica que mesurem l'estat, segons
    * Z_k=H*X_k+v_k
    */
    void initH();
    
    /**
    * Inicialitza el soroll a la mesura
    */
    void initStaticMeasModel();
    void initPriorModel();
    
  public:
    
    /**
    * Crea i incialitza el filtre
    * amb els valors del fitxer filterConfig.h
    */
    PoseEstimationFilter();
    
    virtual ~PoseEstimationFilter();
    
    void updateFromMeasurement(const BFL::ColumnVector& measurement);
    void updateFromMeasurement(const BFL::ColumnVector& measurement,
			       const BFL::ColumnVector& measNoiseMu,
			       const BFL::SymmetricMatrix& measNoiseCov);
			       
    void updateFromMotion(const BFL::ColumnVector& input);
    void updateFromMotion(const BFL::ColumnVector& input,
  		          const BFL::ColumnVector& inputNoiseMu,
			  const BFL::SymmetricMatrix& inputNoiseCov);
					     
    void getPosteriorMean(BFL::ColumnVector& mean);
    void getPosteriorCovariance(BFL::SymmetricMatrix& covariance);
};
 
#endif //POSE_ESTIMATION_POSE_ESTIMATION_FILTER_H
