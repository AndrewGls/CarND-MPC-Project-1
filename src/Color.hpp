//======================================================================================================================
#ifndef Color_hpp
#define Color_hpp
//======================================================================================================================
#include <vector>
//======================================================================================================================

class TColor
{
public:
  using TColorChannel = unsigned char;
public:
  TColor(TColorChannel aRed, TColorChannel aGreen, TColorChannel aBlue, TColorChannel aAlpha=255);

  static TColor SColorWhite(TColorChannel aAlpha=255);
  static TColor SColorBlack(TColorChannel aAlpha=255);
  static TColor SColorGray(TColorChannel aLightness, TColorChannel aAlpha=255);
  static TColor SColorRed(TColorChannel aLightness=255, TColorChannel aAlpha=255);
  static TColor SColorGreen(TColorChannel aLightness=255, TColorChannel aAlpha=255);
  static TColor SColorBlue(TColorChannel aLightness=255, TColorChannel aAlpha=255);
  static TColor SColorYellow(TColorChannel aLightness=255, TColorChannel aAlpha=255);

  TColorChannel Red() const;
  TColorChannel Green() const;
  TColorChannel Blue() const;
  TColorChannel Alpha() const;

private:
   std::vector<TColorChannel> mColorVector;
};



//======================================================================================================================
#endif // Color_hpp
//======================================================================================================================
