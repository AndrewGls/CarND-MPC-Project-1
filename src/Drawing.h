//======================================================================================================================
#ifndef MPC_DRAWING_H
#define MPC_DRAWING_H
//======================================================================================================================
#include "Eigen-3.3/Eigen/Core"
#include <vector>
//======================================================================================================================
class ViewPort;
//======================================================================================================================
namespace NDrawing
{
  Eigen::MatrixXd STransformToScreenCoordinates(const ViewPort& vp, const Eigen::MatrixXd& aCoords);
  void SDrawWayPoints(ViewPort& vp, const Eigen::MatrixXd& aCoords);
  void SDrawPolyFit(ViewPort& vp, const Eigen::VectorXd& aCoeffs);
  void SDrawFit(ViewPort& vp, const std::vector<double>& result);
  void SDrawSteering(ViewPort& vp, double delta, double v);
}

//======================================================================================================================
#endif //MPC_DRAWING_H
//======================================================================================================================
