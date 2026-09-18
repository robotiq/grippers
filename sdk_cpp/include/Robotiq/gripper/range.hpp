// Copyright (c) 2026 Robotiq, Inc.
//
// Licensed under the BSD-3-Clause license; see LICENSE for details.

#pragma once

#include <limits>

namespace Robotiq {

//! \ingroup units
//! \brief The closed interval [lower, upper] that every scaled quantity in a
//!        DeviceProfile takes, so a new quantity costs one member rather
//!        than a pair, and the unit conversions share one ratio arithmetic
//!        instead of keeping a copy each.
//!
//! A greatly simplified fdm::Range: no Eigen support, no vector operations
//! and no assertions, and constexpr throughout so a profile can be a
//! compile-time constant.
//!
//! Nothing here rejects an inverted or empty interval. isValid() reports one
//! and the caller decides, which is how the unit conversions answer
//! std::nullopt rather than scaling by a span they cannot use.
template <class T>
class Range
{
public:
   constexpr Range(T lower, T upper)
      : _lower{lower}
      , _upper{upper}
   {
   }

   [[nodiscard]] constexpr T lower() const { return _lower; }
   [[nodiscard]] constexpr T upper() const { return _upper; }

   //! Always double: an integral range still spans a real distance, and the
   //! ratios below divide by it.
   [[nodiscard]] constexpr double span() const { return static_cast<double>(_upper) - static_cast<double>(_lower); }

   [[nodiscard]] constexpr bool contains(T value) const { return value >= _lower && value <= _upper; }

   [[nodiscard]] constexpr T clamp(T value) const
   {
      if(value < _lower)
      {
         return _lower;
      }
      return value > _upper ? _upper : value;
   }

   //! Where \p value sits in the interval: 0 at lower(), 1 at upper().
   //! Unclamped, and meaningless on an interval isValid() rejects.
   [[nodiscard]] constexpr double ratioAt(T value) const
   {
      return (static_cast<double>(value) - static_cast<double>(_lower)) / span();
   }

   //! The value \p ratio of the way through the interval. Unclamped.
   [[nodiscard]] constexpr double valueAt(double ratio) const { return static_cast<double>(_lower) + ratio * span(); }

   //! An interval the arithmetic above can use: a span above zero and below
   //! infinity. A non-finite limit leaves the span non-finite, so this one
   //! test covers NaN and infinity at either end.
   [[nodiscard]] constexpr bool isValid() const { return span() > 0.0 && span() <= std::numeric_limits<double>::max(); }

private:
   T _lower;
   T _upper;
};

} // namespace Robotiq
