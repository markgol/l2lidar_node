//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: RangeCallSegments.h
//
// Purpose:    Unitree L2 non-linear range correction.
//
//             Loads a Version 2.0.0 range calibration file containing a
//             model representation of the range correction
//             function. During loading, the model is validated and converted
//             into an internal range correction LUT for high-speed runtime
//             correction.
//
//             The runtime interface operates entirely in millimeters.
//
//  Background:
//  Unitree does not provide range correction for th L2.  The L2 exhibits
//  nonlinear range response.
//
//  Solution:
//  The L2calibration class implements the calibration to be used
//  for realtime range correction.  This cuurently is a piecewise cubic spline fit.
//
//  V2.0.0RC1 2026-08-20 Adding range calibration class to be used in the L2lidar class
//                          This will align with L2diagnostics V2.0.0
//                          It will only include the application of the range correction methods
//                          It does not include the creation and calibration procedures
//                            that generates the calibration dataset used in the application of
//                            range correction methods.
//
//--------------------------------------------------------

//--------------------------------------------------------
// GPL-3.0 license
//
// This file is part of L2LidarClass.
//
// L2LidarClass is free software : you can redistribute it and /or modify it under
// the terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// L2LidarClass is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with L2diagnsotic.
// If not, see < https://www.gnu.org/licenses/>.
//--------------------------------------------------------
#pragma once
#include <vector>
#include <array>
#include <string>

// This should not be changed
static constexpr uint32_t MAX_RANGE_MODEL_FIELDS = 10;

//=====================================================================
// Cubic Spline Segment
//
// f(x) = a + b(x-x0) + c(x-x0)^2 + d(x-x0)^3
// Valid for x0 <= x <= x1
//
//  double x0 = 0.0; // field 0
//  double x1 = 0.0; // field 1
//  double a = 0.0; // field 2
//  double b = 0.0; // field 3
//  double c = 0.0; // field 4
//  double d = 0.0; // field 5
//=====================================================================
//=====================================================================
// Range Model Fields Segments
//=====================================================================
struct RangeModelFields
{
    uint32_t fieldCount {0};
    std::array<double, MAX_RANGE_MODEL_FIELDS> fields {};
};

// everything in candidate is in meters

struct RangeCalibrationCandidate
{
    // This structure supports units in either meters or mm
    // but they units can not be mixed
    // They are all meters or all mms
    // Only the constructor of this structure defines which untis
    std::string calibrationMethod {""};
    std::vector<RangeModelFields> segments;

    double minRange {0.0};
    double maxRange {0.0};

    double minCalRange {0.0};
    double maxCalRange {0.0};

    double rmsResidual {0.0};

    bool valid {false};
};
