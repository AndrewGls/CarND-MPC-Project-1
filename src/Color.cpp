//======================================================================================================================
#include "Color.hpp"
//======================================================================================================================

TColor::TColor(TColorChannel aRed, TColorChannel aGreen, TColorChannel aBlue, TColorChannel aAlpha)
{
  mColorVector.push_back(aRed);
  mColorVector.push_back(aGreen);
  mColorVector.push_back(aBlue);
  mColorVector.push_back(aAlpha);
}


//----------------------------------------------------------------------------------------------------------------------

TColor TColor::SColorWhite(TColorChannel aAlpha)
{
  return TColor(255,255,255,aAlpha);
}


//----------------------------------------------------------------------------------------------------------------------

TColor TColor::SColorBlack(TColorChannel aAlpha)
{
  return TColor(0,0,0,aAlpha);
}


//----------------------------------------------------------------------------------------------------------------------

TColor TColor::SColorGray(TColorChannel aLightness, TColorChannel aAlpha)
{
  return TColor(aLightness,aLightness,aLightness,aAlpha);
}


//----------------------------------------------------------------------------------------------------------------------

TColor TColor::SColorRed(TColorChannel aLightness, TColorChannel aAlpha)
{
  return TColor(aLightness,0,0,aAlpha);
}


//----------------------------------------------------------------------------------------------------------------------

TColor TColor::SColorGreen(TColorChannel aLightness, TColorChannel aAlpha)
{
  return TColor(0,aLightness,0,aAlpha);
}


//----------------------------------------------------------------------------------------------------------------------

TColor TColor::SColorBlue(TColorChannel aLightness, TColorChannel aAlpha)
{
  return TColor(0,0,aLightness,aAlpha);
}


//----------------------------------------------------------------------------------------------------------------------

TColor TColor::SColorYellow(TColorChannel aLightness, TColorChannel aAlpha)
{
  return TColor(aLightness, aLightness, 0, aAlpha);
}


//----------------------------------------------------------------------------------------------------------------------

TColor::TColorChannel TColor::Red() const
{
  return mColorVector[0];
}

//----------------------------------------------------------------------------------------------------------------------


TColor::TColorChannel TColor::Green() const
{
  return mColorVector[1];
}

//----------------------------------------------------------------------------------------------------------------------


TColor::TColorChannel TColor::Blue() const
{
  return mColorVector[2];
}

//----------------------------------------------------------------------------------------------------------------------


TColor::TColorChannel TColor::Alpha() const
{
  return mColorVector[3];
}

//======================================================================================================================
