//======================================================================================================================
#include "Color.h"
#include "ViewPort.h"
//======================================================================================================================
#include <opencv2/opencv.hpp>
#include <string>
#include <assert.h>
//======================================================================================================================

class ViewPort::TImpl
{
public:
  TImpl(ViewPort* apInterface)
  {
    mpInterface = apInterface;
  }

  ~TImpl()
  {
    cv::destroyWindow(mDisplayName.c_str());
    mDisplayName = "";
  }

  void CreateViewPort(int width, int height)
  {
    mFrameBuffer = cv::Mat(height, width, CV_8UC3);
  }

  void CreateWindow(const std::string& aDisplayName)
  {
    mDisplayName = aDisplayName;
    cv::namedWindow(aDisplayName.c_str(), CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow"
  }

  void Show()
  {
    assert(mDisplayName.size());
    cv::imshow(mDisplayName.c_str(), mFrameBuffer);
    cv::waitKey(10);
  }

  void Fill(const TColor& aColor)
  {
    mFrameBuffer = cv::Scalar(aColor.Blue(), aColor.Green(), aColor.Red());
  }

  int Width() const {  return mFrameBuffer.cols; }
  int Height() const {  return mFrameBuffer.rows; }

  void DrawMarker(int aX, int aY, const TColor& aColor, int aThickness)
  {
    const cv::Point p1(aX-aThickness, aY-aThickness), p2(aX+aThickness, aY+aThickness);
    const cv::Point p3(aX+aThickness, aY-aThickness), p4(aX-aThickness, aY+aThickness);
    auto color = CVColor(aColor);
    cv::line(mFrameBuffer, p1, p2, color, aThickness);
    cv::line(mFrameBuffer, p3, p4, color, aThickness);
  }

  void DrawMarkers(const Eigen::MatrixXd& aCoords, const TColor& aColor, int aThickness)
  {
    for (int i = 0; i < aCoords.rows(); ++i)
    {
      DrawMarker(aCoords(i,0), aCoords(i,1), aColor, aThickness);
    }
  }


  void MoveTo(float aX, float aY) { mCursor = cvPoint(aX, aY); }

  void LineTo(float aX, float aY, const TColor& aColor, int aThickness)
  {
    cv::Point dst(aX,aY);
    cv::line(mFrameBuffer, mCursor, dst, CVColor(aColor), aThickness);
    mCursor = dst;
  }

private:
  cv::Scalar CVColor(const TColor& aColor) { return cv::Scalar(aColor.Blue(), aColor.Green(), aColor.Red()); }

private:
  cv::Mat mFrameBuffer;
  cv::Point mCursor;
  ViewPort* mpInterface = nullptr;
  std::string mDisplayName;
};

//======================================================================================================================

ViewPort::ViewPort(int width, int height, const std::string& aDisplayName)
{
  mpImpl = new TImpl(this);
  mpImpl->CreateViewPort(width, height);
  mpImpl->CreateWindow(aDisplayName);
}

//----------------------------------------------------------------------------------------------------------------------

ViewPort::~ViewPort()
{
  delete mpImpl;
  mpImpl = nullptr;
}


//----------------------------------------------------------------------------------------------------------------------

void ViewPort::Show()
{
  mpImpl->Show();
}


//----------------------------------------------------------------------------------------------------------------------

void ViewPort::Fill(const TColor& aColor)
{
  mpImpl->Fill(aColor);
}


//----------------------------------------------------------------------------------------------------------------------

int ViewPort::Width() const
{
  return mpImpl->Width();
}


//----------------------------------------------------------------------------------------------------------------------

int ViewPort::Height() const
{
  return mpImpl->Height();
}


//----------------------------------------------------------------------------------------------------------------------

void ViewPort::DrawMarker(float aX, float aY, const TColor& aColor, int aThickness)
{
  mpImpl->DrawMarker(aX, aY, aColor, aThickness);
}

//----------------------------------------------------------------------------------------------------------------------

void ViewPort::DrawMarkers(const Eigen::MatrixXd& aCoords, const TColor& aColor, int aThickness)
{
  mpImpl->DrawMarkers(aCoords, aColor, aThickness);
}


//----------------------------------------------------------------------------------------------------------------------

void ViewPort::MoveTo(float aX, float aY)
{
  mpImpl->MoveTo(aX, aY);
}


//----------------------------------------------------------------------------------------------------------------------

void ViewPort::LineTo(float aX, float aY, const TColor& aColor, int aThickness)
{
  mpImpl->LineTo(aX, aY, aColor, aThickness);
}


//----------------------------------------------------------------------------------------------------------------------

void ViewPort::DrawPolyLine(const Eigen::MatrixXd& aCoords, const TColor& aColor, int aThickness)
{
  for (int i=0; i < aCoords.rows(); ++i)
  {
    if (i == 0)
    {
      MoveTo(aCoords(i,0), aCoords(i,1));
    }
    else
    {
      LineTo(aCoords(i,0), aCoords(i,1), aColor, aThickness);
    }
  }
}

//======================================================================================================================
