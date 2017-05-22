//======================================================================================================================
#ifndef ViewPort_hpp
#define ViewPort_hpp
//======================================================================================================================
#include <string>
#include "Eigen-3.3/Eigen/Core"
class TColor;
//======================================================================================================================

class ViewPort
{
public:
  ViewPort(int width, int height, const std::string& aDisplayName);
  ~ViewPort();

  int Width() const;
  int Height() const;

  void Fill(const TColor& aColor);
  void DrawMarker(float aX, float aY, const TColor& aColor, int aThickness=1);
  void DrawMarkers(const Eigen::MatrixXd& aCoords, const TColor& aColor, int aThickness=1);
  void DrawPolyLine(const Eigen::MatrixXd& aCoords, const TColor& aColor, int aThickness=1);

  void MoveTo(float aX, float aY);
  void LineTo(float aX, float aY, const TColor& aColor, int aThickness=1);

  void Show();


private:
  class TImpl;
  TImpl* mpImpl;
};

//======================================================================================================================
#endif //ViewPort_hpp
//======================================================================================================================
