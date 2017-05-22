//======================================================================================================================
#include "Drawing.h"
#include "Color.h"
#include "ViewPort.h"
#include "Eigen-3.3/Eigen/Core"
//======================================================================================================================

Eigen::MatrixXd NDrawing::STransformToScreenCoordinates(const ViewPort& vp, const Eigen::MatrixXd& aCoords)
{
  Eigen::Matrix2d S;
  const double kScale = 4.0;
  S << 0.0, kScale, -kScale, 0.0;

  Eigen::MatrixXd TransformedCoords = aCoords * S.transpose();
  TransformedCoords.rowwise() += Eigen::RowVector2d(vp.Width() / 2.0, vp.Height());
  return TransformedCoords;
}


//----------------------------------------------------------------------------------------------------------------------

void NDrawing::SDrawWayPoints(ViewPort& vp, const Eigen::MatrixXd& aCoords)
{
  vp.DrawMarkers(STransformToScreenCoordinates(vp, aCoords), TColor::SColorGreen(255), 4);
}


//----------------------------------------------------------------------------------------------------------------------

void NDrawing::SDrawFit(ViewPort& vp, const std::vector<double>& result)
{
  const int w = vp.Width();
  const int h = vp.Height();
  const double kScale = 4.0;
  vp.MoveTo(w/2, h-1);

  for (int i = 2; i < result.size(); i+=2)
  {
    const double x = w / 2 + kScale * result[i+1];
    const double y = h - kScale * result[i];
    const int kThickness = 2;
    vp.LineTo(x, y, TColor::SColorRed(), kThickness);
  }
}

//----------------------------------------------------------------------------------------------------------------------

void NDrawing::SDrawPolyFit(ViewPort& vp, const Eigen::VectorXd& aCoeffs)
{
  std::vector<double> vector_x, vector_y;

  const double delta_x = 4.0;
  double x = 0.0;
  double y = 0.0;

  while (x <= 128.0 && fabs(y) <= 64.0)
  {
    y = 0;
    for (int c = 0; c < aCoeffs.rows(); ++c)
    {
      y += aCoeffs[c] * pow(x, c);
    }
    x += delta_x;
    vector_x.push_back(x);
    vector_y.push_back(y);
  }

  Eigen::MatrixXd Coords(vector_x.size(), 2);
  Coords << Eigen::Map<Eigen::VectorXd>(vector_x.data(), vector_x.size()),
            Eigen::Map<Eigen::VectorXd>(vector_y.data(), vector_y.size());

  vp.DrawPolyLine(STransformToScreenCoordinates(vp, Coords), TColor::SColorGreen(128), 2);
}


//----------------------------------------------------------------------------------------------------------------------

void NDrawing::SDrawSteering(ViewPort& vp, double delta, double v)
{
  double psi = 0;
  double x = 0;
  double y = 0;
  const double Lf = 2.67;
  const double dt = 0.05;
  const int w = vp.Width();
  const int h = vp.Height();
  const double kScale = 4.0;
  vp.MoveTo(w/2, h-1);

  if (v < 0.25)
  {
    return;
  }

  double t = 0.0;

  while (t < 1.0)
  {
    x += v * cos(psi) * dt;
    y += v * sin(psi) * dt;
    psi += v/Lf * delta * dt;
    vp.LineTo(w/2 + kScale*y, h - kScale*x, TColor::SColorGray(224), 2);
    t += dt;
  }
}


//======================================================================================================================