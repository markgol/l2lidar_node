//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: L2calibration.h
//
// Purpose     Unitree L2 non-linear range correction.
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
//  V2.1.0  2026-08-27  Changed calibration file so that range correction
//                          optional.  This allows just metadata to be saved
//                          which includes the overrride biases.
//                      Changed files/names to reflect generalization
//                          of the calibration file rather than RangeCorrection file
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

#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <array>
#include "RangeCalSegments.h"

#define NUM_ALPHA_ANGLES_IN_SCAN 300
//=====================================================================
// non classed structures
//=====================================================================

//---------------------------------------------------------------------
// Range Calibration Metadata structure
//---------------------------------------------------------------------
struct CalibrationInfo
{
    // This structure supports units in either meters or mm
    // but they units can not be mixed
    // They are all meters or all mms
    // Only the constructor of this structure defines which units
    std::string Version;
    std::string Date;

    std::string Sensor;
    std::string SensorID;
    std::string Firmware;

    std::string CreatedBy;

    std::string RangeCalMethod;
    std::string CalibrationDescription;

    int32_t RangeBias = 0;
    double RangeScale = 0.001;
    double AlphaAngleBias = 1.75;
    double AlphaAngleStepSize = 0.602;
    double ThetaAngleBias = 120.0;
    double BetaAngle = 0.25;
    double XiAngle = 0.25;

    uint32_t NumberOfSegments = 0;

    double MinRange = 0.0;
    double MaxRange = 0.0;

    double MinTrustedRange = -1.0; // not initialized

    double MinCalRange = 0.0;
    double MaxCalRange = 0.0;

    double RMSResidual = 0.0;
};

//=====================================================================
// non classed definitions
//=====================================================================
static constexpr const char* META_VERSION                 = "Version";
static constexpr const char* META_DATE                    = "Date";
static constexpr const char* META_SENSOR                  = "Sensor";
static constexpr const char* META_SENSOR_ID               = "SensorID";
static constexpr const char* META_RANGE_BIAS              = "RangeBias";
static constexpr const char* META_RANGE_SCALE             = "RangeScale";
static constexpr const char* META_ALPHA_ANGLE_BIAS        = "AlphaAngleBias";
static constexpr const char* META_ALPHA_ANGLE_STEP        = "AlphaAngleStepSize";
static constexpr const char* META_THETA_ANGLE_BIAS        = "ThetaAngleBias";
static constexpr const char* META_BETA_ANGLE              = "BetaAngle";
static constexpr const char* META_XI_ANGLE                = "XiAngle";
static constexpr const char* META_RANGE_CALIBRATION_METHOD = "RangeCorrectionMethod";
static constexpr const char* META_CALIBRATION_DESCRIPTION = "CalibrationDescription";
static constexpr const char* META_FIRMWARE                = "Firmware";
static constexpr const char* META_CREATEDBY               = "CreatedBy";
static constexpr const char* META_NUM_OF_RANGE_SEGMENTS   = "NumberRangeSegments";
static constexpr const char* META_MIN_RANGE               = "MinRange";
static constexpr const char* META_MAX_RANGE               = "MaxRange";
static constexpr const char* META_MIN_TRUSTED_RANGE       = "MinTrustedRange";
static constexpr const char* META_CAL_MIN_RANGE           = "MinCalRange";
static constexpr const char* META_CAL_MAX_RANGE           = "MaxCalRange";
static constexpr const char* META_RMS_RESIDUAL            = "RMSResidual";
static constexpr const char* SECTION_RANGE_MODEL          ="# RANGE MODEL";
static constexpr const char* SECTION_ALPHA_ANGLE_LUT      ="# ALPHA ANGLE LUT";
static constexpr const char* SECTION_CALIBRATION_POINTS   ="# CALIBRATION POINTS";

static constexpr uint32_t MAX_ALPHA_ANGLE_LUT_FIELDS = 2;

// calibration methods
static constexpr const char* METHOD_CUBIC_SPLINE = "CubicSpline";
static constexpr double SEGMENT_CONNECTION_TOLERANCE = 1.0e-12;

// parser constants for Cubic Spline
static constexpr std::array<const char*, 6> CUBICSPLINE_FIELDS =
    {
        "x0",
        "x1",
        "a",
        "b",
        "c",
        "d"
};

// implementation constants for cubic spline
static constexpr uint32_t FIELD_X0 = 0;
static constexpr uint32_t FIELD_X1 = 1;
static constexpr uint32_t FIELD_A  = 2;
static constexpr uint32_t FIELD_B  = 3;
static constexpr uint32_t FIELD_C  = 4;
static constexpr uint32_t FIELD_D  = 5;

// only one type of CALIBRATION_POINT_FIELDS
static constexpr std::array<const char*, 3> CALIBRATION_POINT_FIELDS =
    {
        "MeasuredRange",
        "TrueRange",
        "Correction"
};

// implementation constants for ALPHA ANLGE LUT
static constexpr uint32_t FIELD_INDEX = 0;
static constexpr uint32_t FIELD_ANGLE = 1;

// only one type of ALPHA ANLGE LUT
static constexpr std::array<const char*, 2> ALPHA_ANLGE_LUT_FIELDS =
    {
        "FastScanIndex",
        "RelativeAngle"
};

//=====================================================================
// Alpha Angle LUT Fields Segments
//=====================================================================
struct AlphaAngleLUTFields
{
    uint32_t fieldCount {0};

    std::array<double, MAX_ALPHA_ANGLE_LUT_FIELDS> fields {};
};

//---------------------------------------------------------------------
// class L2calibration definition
//---------------------------------------------------------------------
class L2calibration
{
public:

    // methods

    L2calibration();
    ~L2calibration() = default;

    L2calibration(const L2calibration&) = delete;
    L2calibration& operator=(const L2calibration&) = delete;

    L2calibration(L2calibration&&) noexcept = default;
    L2calibration& operator=(L2calibration&&) noexcept = default;

    //---------------------------------------------------------------------
    // Range Correction methods
    //---------------------------------------------------------------------
    // standard method of loading range correction calibration
    bool LoadCalibration(const std::string& filename);

    // range correction methods
    const std::vector<double>& GetRangeCorrectionLUT() const noexcept;
                            // LUT in mm for every possible range return value

    const std::vector<std::string> GetWarnings() {
        return mWarnings;
    };

    const std::vector<std::string> GetErrors() {
        return mErrors;
    };

    const CalibrationInfo& GetCalibrationInfo() const noexcept
    {
        return mCalibrationInfo;
    }

    void ClearCalibration();

    bool IsAlphaAngleLUTloaded();
    const std::vector<double>& GetAlphaAngleLUT() const noexcept;

private: // structures


    //=====================================================================
    // Range Model Fields Segments
    //=====================================================================
    struct RangeModelFields
    {
        uint32_t fieldCount {0};

        std::array<double, MAX_RANGE_MODEL_FIELDS> fields {};
    };

    //=====================================================================
    // Original Calibration Point
    //=====================================================================

    struct RangeCalibrationPoint
    {
        double MeasuredRange = 0.0;
        double TrueRange = 0.0;
        double Correction = 0.0;
    };

private: // functions

    bool ReadCalibrationFile(const std::string& filename);
    bool ValidateMeta(const CalibrationInfo& info);
    bool ValidateRangeCalibration(const std::vector<RangeModelFields> RangeCorrectionModel);
    bool ValidateAlphaAngleLUT();
    bool BuildCubicSplineLUT(const std::vector<RangeModelFields>& RangeCorrectionModel);
    bool ValidateCubicSpline(const std::vector<RangeModelFields> RangeCorrectionModel);
    bool BuildRangeCorrectionLUT(const std::vector<RangeModelFields> RangeCorrectionModel);

    //-----------------------------------------
    // Parser methods and constructs
    //-----------------------------------------

    void ResetParserState();

    enum class ParseState
    {
        Metadata,
        RangeModel,
        CalibrationPoints,
        AlphaAngleLUT
    };

    enum class CalibrationMethod
    {
        Unknown,
        CubicSpline
    };

    static std::string Trim(const std::string& text);

    static bool SplitKeyValue(
        const std::string& line,
        std::string& key,
        std::string& value);

    bool ParseMetadataLine(
        const std::string& line,
        uint32_t lineNumber);

    bool ParseRangeModelLine(
        const std::string& line,
        uint32_t lineNumber);

    bool ParseAlphaAngleLUTline(
        const std::string& line,
        uint32_t lineNumber);

    bool ParseCubicSplineLine(
        const std::string& line,
        uint32_t lineNumber);

    bool ParseCalibrationPointLine(
        const std::string& line,
        uint32_t lineNumber);

    bool ParseMetadataField(
        const std::string& key,
        const std::string& value,
        uint32_t lineNumber);

    static bool IsCommentLine(const std::string& line);

    static void SplitCSV(
        const std::string& line,
        std::vector<std::string>& fields);

    static bool ParseDouble(
        const std::string& text,
        double& value);

    static bool ParseUInt32(
        const std::string& text,
        uint32_t& value);

    static bool ParseInt32(
        const std::string& text,
        int32_t& value);

private: // variables

    ParseState mParseState = ParseState::Metadata;
    CalibrationInfo mCalibrationInfo; // this instance uses mm as units

    CalibrationMethod mMethod = CalibrationMethod::Unknown;

    bool mRangeModelHeaderRead {false};
    uint32_t mExpectedModelFieldCount {0};
    std::vector<RangeModelFields> mRangeCorrectionModel;

    bool mAlphaAngleLUTheaderRead {false};
    uint32_t mAlphaAngleLUTFieldCount {0};
    std::vector<AlphaAngleLUTFields> mAlphaAngleLUTfields;
    std::vector<double> mAlphaAngleLUT;
    bool mAlphaLUTvalid {false};

    bool mCalibrationPointsHeaderRead {false};
    uint32_t mExpectedCalibrationPointFieldCount {0};
    std::vector<RangeCalibrationPoint> mRangeCalibrationPoints;

    std::vector<double> mRangeCorrectionLUT;
    double mMinCalibratedRange = 65535.0;
    double mMaxCalibratedRange = 0.0;

    std::vector<std::string> mWarnings;
    std::vector<std::string> mErrors;
};

