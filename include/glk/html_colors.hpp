#ifndef GLK_COLORS_HPP
#define GLK_COLORS_HPP

#include <vector>
#include <Eigen/Core>

namespace glk {

enum class COLORNAME {
  AliceBlue = 0,
  AntiqueWhite,
  Aqua,
  Aquamarine,
  Azure,
  Beige,
  Bisque,
  Black,
  BlanchedAlmond,
  Blue,
  BlueViolet,
  Brown,
  BurlyWood,
  CadetBlue,
  Chartreuse,
  Chocolate,
  Coral,
  CornflowerBlue,
  Cornsilk,
  Crimson,
  Cyan,
  DarkBlue,
  DarkCyan,
  DarkGoldenRod,
  DarkGray,
  DarkGrey,
  DarkGreen,
  DarkKhaki,
  DarkMagenta,
  DarkOliveGreen,
  DarkOrange,
  DarkOrchid,
  DarkRed,
  DarkSalmon,
  DarkSeaGreen,
  DarkSlateBlue,
  DarkSlateGray,
  DarkSlateGrey,
  DarkTurquoise,
  DarkViolet,
  DeepPink,
  DeepSkyBlue,
  DimGray,
  DimGrey,
  DodgerBlue,
  FireBrick,
  FloralWhite,
  ForestGreen,
  Fuchsia,
  Gainsboro,
  GhostWhite,
  Gold,
  GoldenRod,
  Gray,
  Grey,
  Green,
  GreenYellow,
  HoneyDew,
  HotPink,
  IndianRed,
  Indigo,
  Ivory,
  Khaki,
  Lavender,
  LavenderBlush,
  LawnGreen,
  LemonChiffon,
  LightBlue,
  LightCoral,
  LightCyan,
  LightGoldenRodYellow,
  LightGray,
  LightGrey,
  LightGreen,
  LightPink,
  LightSalmon,
  LightSeaGreen,
  LightSkyBlue,
  LightSlateGray,
  LightSlateGrey,
  LightSteelBlue,
  LightYellow,
  Lime,
  LimeGreen,
  Linen,
  Magenta,
  Maroon,
  MediumAquaMarine,
  MediumBlue,
  MediumOrchid,
  MediumPurple,
  MediumSeaGreen,
  MediumSlateBlue,
  MediumSpringGreen,
  MediumTurquoise,
  MediumVioletRed,
  MidnightBlue,
  MintCream,
  MistyRose,
  Moccasin,
  NavajoWhite,
  Navy,
  OldLace,
  Olive,
  OliveDrab,
  Orange,
  OrangeRed,
  Orchid,
  PaleGoldenRod,
  PaleGreen,
  PaleTurquoise,
  PaleVioletRed,
  PapayaWhip,
  PeachPuff,
  Peru,
  Pink,
  Plum,
  PowderBlue,
  Purple,
  RebeccaPurple,
  Red,
  RosyBrown,
  RoyalBlue,
  SaddleBrown,
  Salmon,
  SandyBrown,
  SeaGreen,
  SeaShell,
  Sienna,
  Silver,
  SkyBlue,
  SlateBlue,
  SlateGray,
  SlateGrey,
  Snow,
  SpringGreen,
  SteelBlue,
  Tan,
  Teal,
  Thistle,
  Tomato,
  Turquoise,
  Violet,
  Wheat,
  White,
  WhiteSmoke,
  Yellow,
  YellowGreen,
  NUM_COLORNAMES
};

Eigen::Vector4i html_color(COLORNAME color);
Eigen::Vector4f html_colorf(COLORNAME color);

const std::array<const char*, static_cast<size_t>(COLORNAME::NUM_COLORNAMES)>& html_color_names();
std::array<unsigned char, 3> html_color_value(COLORNAME color);

// HTML color names
inline Eigen::Vector4f AliceBlue() {
  return html_colorf(COLORNAME::AliceBlue);
}
inline Eigen::Vector4f AntiqueWhite() {
  return html_colorf(COLORNAME::AntiqueWhite);
}
inline Eigen::Vector4f Aqua() {
  return html_colorf(COLORNAME::Aqua);
}
inline Eigen::Vector4f Aquamarine() {
  return html_colorf(COLORNAME::Aquamarine);
}
inline Eigen::Vector4f Azure() {
  return html_colorf(COLORNAME::Azure);
}
inline Eigen::Vector4f Beige() {
  return html_colorf(COLORNAME::Beige);
}
inline Eigen::Vector4f Bisque() {
  return html_colorf(COLORNAME::Bisque);
}
inline Eigen::Vector4f Black() {
  return html_colorf(COLORNAME::Black);
}
inline Eigen::Vector4f BlanchedAlmond() {
  return html_colorf(COLORNAME::BlanchedAlmond);
}
inline Eigen::Vector4f Blue() {
  return html_colorf(COLORNAME::Blue);
}
inline Eigen::Vector4f BlueViolet() {
  return html_colorf(COLORNAME::BlueViolet);
}
inline Eigen::Vector4f Brown() {
  return html_colorf(COLORNAME::Brown);
}
inline Eigen::Vector4f BurlyWood() {
  return html_colorf(COLORNAME::BurlyWood);
}
inline Eigen::Vector4f CadetBlue() {
  return html_colorf(COLORNAME::CadetBlue);
}
inline Eigen::Vector4f Chartreuse() {
  return html_colorf(COLORNAME::Chartreuse);
}
inline Eigen::Vector4f Chocolate() {
  return html_colorf(COLORNAME::Chocolate);
}
inline Eigen::Vector4f Coral() {
  return html_colorf(COLORNAME::Coral);
}
inline Eigen::Vector4f CornflowerBlue() {
  return html_colorf(COLORNAME::CornflowerBlue);
}
inline Eigen::Vector4f Cornsilk() {
  return html_colorf(COLORNAME::Cornsilk);
}
inline Eigen::Vector4f Crimson() {
  return html_colorf(COLORNAME::Crimson);
}
inline Eigen::Vector4f Cyan() {
  return html_colorf(COLORNAME::Cyan);
}
inline Eigen::Vector4f DarkBlue() {
  return html_colorf(COLORNAME::DarkBlue);
}
inline Eigen::Vector4f DarkCyan() {
  return html_colorf(COLORNAME::DarkCyan);
}
inline Eigen::Vector4f DarkGoldenRod() {
  return html_colorf(COLORNAME::DarkGoldenRod);
}
inline Eigen::Vector4f DarkGray() {
  return html_colorf(COLORNAME::DarkGray);
}
inline Eigen::Vector4f DarkGrey() {
  return html_colorf(COLORNAME::DarkGrey);
}
inline Eigen::Vector4f DarkGreen() {
  return html_colorf(COLORNAME::DarkGreen);
}
inline Eigen::Vector4f DarkKhaki() {
  return html_colorf(COLORNAME::DarkKhaki);
}
inline Eigen::Vector4f DarkMagenta() {
  return html_colorf(COLORNAME::DarkMagenta);
}
inline Eigen::Vector4f DarkOliveGreen() {
  return html_colorf(COLORNAME::DarkOliveGreen);
}
inline Eigen::Vector4f DarkOrange() {
  return html_colorf(COLORNAME::DarkOrange);
}
inline Eigen::Vector4f DarkOrchid() {
  return html_colorf(COLORNAME::DarkOrchid);
}
inline Eigen::Vector4f DarkRed() {
  return html_colorf(COLORNAME::DarkRed);
}
inline Eigen::Vector4f DarkSalmon() {
  return html_colorf(COLORNAME::DarkSalmon);
}
inline Eigen::Vector4f DarkSeaGreen() {
  return html_colorf(COLORNAME::DarkSeaGreen);
}
inline Eigen::Vector4f DarkSlateBlue() {
  return html_colorf(COLORNAME::DarkSlateBlue);
}
inline Eigen::Vector4f DarkSlateGray() {
  return html_colorf(COLORNAME::DarkSlateGray);
}
inline Eigen::Vector4f DarkSlateGrey() {
  return html_colorf(COLORNAME::DarkSlateGrey);
}
inline Eigen::Vector4f DarkTurquoise() {
  return html_colorf(COLORNAME::DarkTurquoise);
}
inline Eigen::Vector4f DarkViolet() {
  return html_colorf(COLORNAME::DarkViolet);
}
inline Eigen::Vector4f DeepPink() {
  return html_colorf(COLORNAME::DeepPink);
}
inline Eigen::Vector4f DeepSkyBlue() {
  return html_colorf(COLORNAME::DeepSkyBlue);
}
inline Eigen::Vector4f DimGray() {
  return html_colorf(COLORNAME::DimGray);
}
inline Eigen::Vector4f DimGrey() {
  return html_colorf(COLORNAME::DimGrey);
}
inline Eigen::Vector4f DodgerBlue() {
  return html_colorf(COLORNAME::DodgerBlue);
}
inline Eigen::Vector4f FireBrick() {
  return html_colorf(COLORNAME::FireBrick);
}
inline Eigen::Vector4f FloralWhite() {
  return html_colorf(COLORNAME::FloralWhite);
}
inline Eigen::Vector4f ForestGreen() {
  return html_colorf(COLORNAME::ForestGreen);
}
inline Eigen::Vector4f Fuchsia() {
  return html_colorf(COLORNAME::Fuchsia);
}
inline Eigen::Vector4f Gainsboro() {
  return html_colorf(COLORNAME::Gainsboro);
}
inline Eigen::Vector4f GhostWhite() {
  return html_colorf(COLORNAME::GhostWhite);
}
inline Eigen::Vector4f Gold() {
  return html_colorf(COLORNAME::Gold);
}
inline Eigen::Vector4f GoldenRod() {
  return html_colorf(COLORNAME::GoldenRod);
}
inline Eigen::Vector4f Gray() {
  return html_colorf(COLORNAME::Gray);
}
inline Eigen::Vector4f Grey() {
  return html_colorf(COLORNAME::Grey);
}
inline Eigen::Vector4f Green() {
  return html_colorf(COLORNAME::Green);
}
inline Eigen::Vector4f GreenYellow() {
  return html_colorf(COLORNAME::GreenYellow);
}
inline Eigen::Vector4f HoneyDew() {
  return html_colorf(COLORNAME::HoneyDew);
}
inline Eigen::Vector4f HotPink() {
  return html_colorf(COLORNAME::HotPink);
}
inline Eigen::Vector4f IndianRed() {
  return html_colorf(COLORNAME::IndianRed);
}
inline Eigen::Vector4f Indigo() {
  return html_colorf(COLORNAME::Indigo);
}
inline Eigen::Vector4f Ivory() {
  return html_colorf(COLORNAME::Ivory);
}
inline Eigen::Vector4f Khaki() {
  return html_colorf(COLORNAME::Khaki);
}
inline Eigen::Vector4f Lavender() {
  return html_colorf(COLORNAME::Lavender);
}
inline Eigen::Vector4f LavenderBlush() {
  return html_colorf(COLORNAME::LavenderBlush);
}
inline Eigen::Vector4f LawnGreen() {
  return html_colorf(COLORNAME::LawnGreen);
}
inline Eigen::Vector4f LemonChiffon() {
  return html_colorf(COLORNAME::LemonChiffon);
}
inline Eigen::Vector4f LightBlue() {
  return html_colorf(COLORNAME::LightBlue);
}
inline Eigen::Vector4f LightCoral() {
  return html_colorf(COLORNAME::LightCoral);
}
inline Eigen::Vector4f LightCyan() {
  return html_colorf(COLORNAME::LightCyan);
}
inline Eigen::Vector4f LightGoldenRodYellow() {
  return html_colorf(COLORNAME::LightGoldenRodYellow);
}
inline Eigen::Vector4f LightGray() {
  return html_colorf(COLORNAME::LightGray);
}
inline Eigen::Vector4f LightGrey() {
  return html_colorf(COLORNAME::LightGrey);
}
inline Eigen::Vector4f LightGreen() {
  return html_colorf(COLORNAME::LightGreen);
}
inline Eigen::Vector4f LightPink() {
  return html_colorf(COLORNAME::LightPink);
}
inline Eigen::Vector4f LightSalmon() {
  return html_colorf(COLORNAME::LightSalmon);
}
inline Eigen::Vector4f LightSeaGreen() {
  return html_colorf(COLORNAME::LightSeaGreen);
}
inline Eigen::Vector4f LightSkyBlue() {
  return html_colorf(COLORNAME::LightSkyBlue);
}
inline Eigen::Vector4f LightSlateGray() {
  return html_colorf(COLORNAME::LightSlateGray);
}
inline Eigen::Vector4f LightSlateGrey() {
  return html_colorf(COLORNAME::LightSlateGrey);
}
inline Eigen::Vector4f LightSteelBlue() {
  return html_colorf(COLORNAME::LightSteelBlue);
}
inline Eigen::Vector4f LightYellow() {
  return html_colorf(COLORNAME::LightYellow);
}
inline Eigen::Vector4f Lime() {
  return html_colorf(COLORNAME::Lime);
}
inline Eigen::Vector4f LimeGreen() {
  return html_colorf(COLORNAME::LimeGreen);
}
inline Eigen::Vector4f Linen() {
  return html_colorf(COLORNAME::Linen);
}
inline Eigen::Vector4f Magenta() {
  return html_colorf(COLORNAME::Magenta);
}
inline Eigen::Vector4f Maroon() {
  return html_colorf(COLORNAME::Maroon);
}
inline Eigen::Vector4f MediumAquaMarine() {
  return html_colorf(COLORNAME::MediumAquaMarine);
}
inline Eigen::Vector4f MediumBlue() {
  return html_colorf(COLORNAME::MediumBlue);
}
inline Eigen::Vector4f MediumOrchid() {
  return html_colorf(COLORNAME::MediumOrchid);
}
inline Eigen::Vector4f MediumPurple() {
  return html_colorf(COLORNAME::MediumPurple);
}
inline Eigen::Vector4f MediumSeaGreen() {
  return html_colorf(COLORNAME::MediumSeaGreen);
}
inline Eigen::Vector4f MediumSlateBlue() {
  return html_colorf(COLORNAME::MediumSlateBlue);
}
inline Eigen::Vector4f MediumSpringGreen() {
  return html_colorf(COLORNAME::MediumSpringGreen);
}
inline Eigen::Vector4f MediumTurquoise() {
  return html_colorf(COLORNAME::MediumTurquoise);
}
inline Eigen::Vector4f MediumVioletRed() {
  return html_colorf(COLORNAME::MediumVioletRed);
}
inline Eigen::Vector4f MidnightBlue() {
  return html_colorf(COLORNAME::MidnightBlue);
}
inline Eigen::Vector4f MintCream() {
  return html_colorf(COLORNAME::MintCream);
}
inline Eigen::Vector4f MistyRose() {
  return html_colorf(COLORNAME::MistyRose);
}
inline Eigen::Vector4f Moccasin() {
  return html_colorf(COLORNAME::Moccasin);
}
inline Eigen::Vector4f NavajoWhite() {
  return html_colorf(COLORNAME::NavajoWhite);
}
inline Eigen::Vector4f Navy() {
  return html_colorf(COLORNAME::Navy);
}
inline Eigen::Vector4f OldLace() {
  return html_colorf(COLORNAME::OldLace);
}
inline Eigen::Vector4f Olive() {
  return html_colorf(COLORNAME::Olive);
}
inline Eigen::Vector4f OliveDrab() {
  return html_colorf(COLORNAME::OliveDrab);
}
inline Eigen::Vector4f Orange() {
  return html_colorf(COLORNAME::Orange);
}
inline Eigen::Vector4f OrangeRed() {
  return html_colorf(COLORNAME::OrangeRed);
}
inline Eigen::Vector4f Orchid() {
  return html_colorf(COLORNAME::Orchid);
}
inline Eigen::Vector4f PaleGoldenRod() {
  return html_colorf(COLORNAME::PaleGoldenRod);
}
inline Eigen::Vector4f PaleGreen() {
  return html_colorf(COLORNAME::PaleGreen);
}
inline Eigen::Vector4f PaleTurquoise() {
  return html_colorf(COLORNAME::PaleTurquoise);
}
inline Eigen::Vector4f PaleVioletRed() {
  return html_colorf(COLORNAME::PaleVioletRed);
}
inline Eigen::Vector4f PapayaWhip() {
  return html_colorf(COLORNAME::PapayaWhip);
}
inline Eigen::Vector4f PeachPuff() {
  return html_colorf(COLORNAME::PeachPuff);
}
inline Eigen::Vector4f Peru() {
  return html_colorf(COLORNAME::Peru);
}
inline Eigen::Vector4f Pink() {
  return html_colorf(COLORNAME::Pink);
}
inline Eigen::Vector4f Plum() {
  return html_colorf(COLORNAME::Plum);
}
inline Eigen::Vector4f PowderBlue() {
  return html_colorf(COLORNAME::PowderBlue);
}
inline Eigen::Vector4f Purple() {
  return html_colorf(COLORNAME::Purple);
}
inline Eigen::Vector4f RebeccaPurple() {
  return html_colorf(COLORNAME::RebeccaPurple);
}
inline Eigen::Vector4f Red() {
  return html_colorf(COLORNAME::Red);
}
inline Eigen::Vector4f RosyBrown() {
  return html_colorf(COLORNAME::RosyBrown);
}
inline Eigen::Vector4f RoyalBlue() {
  return html_colorf(COLORNAME::RoyalBlue);
}
inline Eigen::Vector4f SaddleBrown() {
  return html_colorf(COLORNAME::SaddleBrown);
}
inline Eigen::Vector4f Salmon() {
  return html_colorf(COLORNAME::Salmon);
}
inline Eigen::Vector4f SandyBrown() {
  return html_colorf(COLORNAME::SandyBrown);
}
inline Eigen::Vector4f SeaGreen() {
  return html_colorf(COLORNAME::SeaGreen);
}
inline Eigen::Vector4f SeaShell() {
  return html_colorf(COLORNAME::SeaShell);
}
inline Eigen::Vector4f Sienna() {
  return html_colorf(COLORNAME::Sienna);
}
inline Eigen::Vector4f Silver() {
  return html_colorf(COLORNAME::Silver);
}
inline Eigen::Vector4f SkyBlue() {
  return html_colorf(COLORNAME::SkyBlue);
}
inline Eigen::Vector4f SlateBlue() {
  return html_colorf(COLORNAME::SlateBlue);
}
inline Eigen::Vector4f SlateGray() {
  return html_colorf(COLORNAME::SlateGray);
}
inline Eigen::Vector4f SlateGrey() {
  return html_colorf(COLORNAME::SlateGrey);
}
inline Eigen::Vector4f Snow() {
  return html_colorf(COLORNAME::Snow);
}
inline Eigen::Vector4f SpringGreen() {
  return html_colorf(COLORNAME::SpringGreen);
}
inline Eigen::Vector4f SteelBlue() {
  return html_colorf(COLORNAME::SteelBlue);
}
inline Eigen::Vector4f Tan() {
  return html_colorf(COLORNAME::Tan);
}
inline Eigen::Vector4f Teal() {
  return html_colorf(COLORNAME::Teal);
}
inline Eigen::Vector4f Thistle() {
  return html_colorf(COLORNAME::Thistle);
}
inline Eigen::Vector4f Tomato() {
  return html_colorf(COLORNAME::Tomato);
}
inline Eigen::Vector4f Turquoise() {
  return html_colorf(COLORNAME::Turquoise);
}
inline Eigen::Vector4f Violet() {
  return html_colorf(COLORNAME::Violet);
}
inline Eigen::Vector4f Wheat() {
  return html_colorf(COLORNAME::Wheat);
}
inline Eigen::Vector4f White() {
  return html_colorf(COLORNAME::White);
}
inline Eigen::Vector4f WhiteSmoke() {
  return html_colorf(COLORNAME::WhiteSmoke);
}
inline Eigen::Vector4f Yellow() {
  return html_colorf(COLORNAME::Yellow);
}
inline Eigen::Vector4f YellowGreen() {
  return html_colorf(COLORNAME::YellowGreen);
}

}  // namespace glk

#endif