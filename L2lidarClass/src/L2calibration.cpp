//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: L2calibration.cpp
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
//  The L2RangeCorrection class implements the range calibration to be used
//  for realtime range correction.  This cuurently is a piecewise cubic spline fit.
//
//  V2.0.0 RC1 2026-08-20 Adding range calibration class to be used in the L2lidar class
//                          This will align with L2diagnostics V2.0.0
//                          It will only include the application of the range correction methods
//                          It does not include the creation and calibration procedures
//                            that generates the calibration dataset used in the application of
//                            range correction methods.
//  V2.0.1  2026-08-24 Implemented application of the alpha angle LUT
//                     Added Alpha Angle step size override
//                     Removed unused code
//  V2.1.0  2026-08-27  Changed calibration file so that range correction
//                          optional.  This allows just metadata to be saved
//                          which includes the overrride biases.
//                      Changed files/names to reflect generalization
//                          of the calibration file rather than RangeCorrection file
//  V2.1.1  2026-09-01  Fixed logic validation error for MinTrustedRange
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
#include "L2calibration.h"
#include <fstream>
#include <stdexcept>
#include "quaternion.h"

//=============================================================================
//
// Constructor
//
//=============================================================================
L2calibration::L2calibration()
{
    ClearCalibration();
}

//=============================================================================
//
// Clear
//
// Restores the object to its default constructed state.
//
//=============================================================================
void L2calibration::ClearCalibration()
{
    mRangeCalibrationPoints.clear();
    mRangeCorrectionLUT.clear();
    mCalibrationInfo = CalibrationInfo();
    mAlphaAngleLUTfields.clear();
    mAlphaAngleLUTFieldCount = 0;
}

//--------------------------------------------------------
//  LoadCalibration
//--------------------------------------------------------
bool L2calibration::LoadCalibration(
    const std::string& filename)
{
    ResetParserState();

    // read range calibration file
    if(!ReadCalibrationFile(filename))
        return false;

    // validate Meta data
    if(!ValidateMeta(mCalibrationInfo))
        return false;

    if(mCalibrationInfo.RangeCalMethod!="None") {
        // validate range calibration data
        if(!ValidateRangeCalibration(mRangeCorrectionModel))
            return false;

        // build range correction LUT
        if(!BuildCubicSplineLUT(mRangeCorrectionModel)) {
            return false;
        }
    } else {
        mRangeCorrectionModel.clear();
        mRangeCorrectionLUT.clear();
        mWarnings.emplace_back(
            "Calibration file does not contain Range model section\n");
    }

    if(!ValidateAlphaAngleLUT()) {
        mAlphaAngleLUTfields.clear();
        mAlphaAngleLUT.clear();
        mAlphaLUTvalid = false;
        return false;
    }
    // if the alpha angles LUT fields is validated
    // the angles needs to be copied to the real LUT
    if(mAlphaLUTvalid) {
        // copy to incremental angles to mAlphaAngleLUT
        double next = 0.0;
        for(int i=0; i<NUM_ALPHA_ANGLES_IN_SCAN; i++) {
            mAlphaAngleLUT.emplace_back(next);
            next += mAlphaAngleLUTfields[i].fields[1] * DEG_TO_RAD;
        }
    }

    return true;
}

//--------------------------------------------------------
//  ReadCalibrationFile
//--------------------------------------------------------
bool L2calibration::ReadCalibrationFile(
    const std::string& filename)
{    
    std::ifstream file(filename);
    if(!file) {
        mErrors.push_back("Unable to open range calibration file: " + filename);
        return false;
    }

    std::string line;
    uint32_t lineNumber {0};

    while (std::getline(file, line)) {
        lineNumber++;
        line = Trim(line);
        if (line.empty()) {
            continue;
        }
        if (IsCommentLine(line)) {
            continue;
        }

        if (line == SECTION_RANGE_MODEL) {
            mParseState = ParseState::RangeModel;
            continue;
        }

        if (line == SECTION_CALIBRATION_POINTS) {
            mParseState = ParseState::CalibrationPoints;
            continue;
        }

        if (line == SECTION_ALPHA_ANGLE_LUT) {
            mParseState = ParseState::AlphaAngleLUT;
            continue;
        }

        switch (mParseState)
        {
        case ParseState::Metadata:
            if (!ParseMetadataLine(line,lineNumber))
                return false;

            break;

        case ParseState::RangeModel:
            if (!ParseRangeModelLine(line,lineNumber))
                return false;

            break;

        case ParseState::CalibrationPoints:
            if (!ParseCalibrationPointLine(line,lineNumber))
                return false;
            break;

        case ParseState::AlphaAngleLUT:
            if (!ParseAlphaAngleLUTline(line,lineNumber))
                return false;

            break;
        }

    }
    return true;
}

//--------------------------------------------------------
//  ParseMetadataLine
//--------------------------------------------------------
bool L2calibration::ParseMetadataLine(
    const std::string& line,
    uint32_t lineNumber)
{
    std::string key;
    std::string value;

    if (!SplitKeyValue(line, key, value)) {
        mErrors.push_back(
            "Line " +
            std::to_string(lineNumber) +
            ": Invalid metadata format.\n");
        return false;
    }

    if (key == META_VERSION) {
        mCalibrationInfo.Version = value;
    }
    else if (key == META_DATE) {
        mCalibrationInfo.Date = value;
    }
    else if (key == META_SENSOR) {
        mCalibrationInfo.Sensor = value;
    }
    else if (key == META_SENSOR_ID) {
        mCalibrationInfo.SensorID = value;
    }
    else if (key == META_FIRMWARE) {
        mCalibrationInfo.Firmware = value;
    }
    else if (key == META_CREATEDBY) {
        mCalibrationInfo.CreatedBy = value;
    }
    else if (key == META_RANGE_CALIBRATION_METHOD) {
        mCalibrationInfo.RangeCalMethod = value;
    } else if (key == META_CALIBRATION_DESCRIPTION) {
        mCalibrationInfo.CalibrationDescription = value;
    } else if (key == META_RANGE_BIAS) {
        if (!ParseInt32(value, mCalibrationInfo.RangeBias)) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid RangeBias value.\n");
            return false;
        }
    } else if (key == META_RANGE_SCALE) {
        if (!ParseDouble(value, mCalibrationInfo.RangeScale)) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid RangeScale value.\n");
            return false;
        }
    } else if (key == META_ALPHA_ANGLE_BIAS) {
        if (!ParseDouble(value, mCalibrationInfo.AlphaAngleBias)) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid AlphaAngleBias value.\n");
            return false;
        }
    } else if (key == META_ALPHA_ANGLE_STEP) {
        if (!ParseDouble(value, mCalibrationInfo.AlphaAngleStepSize)) {
            mErrors.push_back(

                "Line " +
                std::to_string(lineNumber) +
                ": Invalid AlphaAngleBias value.\n");
            return false;
        }
    } else if (key == META_THETA_ANGLE_BIAS) {
        if (!ParseDouble(value, mCalibrationInfo.ThetaAngleBias)) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid ThetaAngleBias value.\n");
            return false;
        }
    } else if (key == META_BETA_ANGLE) {
        if (!ParseDouble(value, mCalibrationInfo.BetaAngle)) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid BetaAngle value.\n");
            return false;
        }
    } else if (key == META_XI_ANGLE) {
        if (!ParseDouble(value, mCalibrationInfo.XiAngle)) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid XiAngle value.\n");
            return false;
        }
    } else if (key == META_MIN_RANGE) {
        if (!ParseDouble(value, mCalibrationInfo.MinRange)) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid MinRange value.\n");
            return false;
        }
    } else if (key == META_MAX_RANGE) {
        if (!ParseDouble(value, mCalibrationInfo.MaxRange)) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid MaxRange value.\n");
            return false;
        }
    } else if (key == META_MIN_TRUSTED_RANGE) {
        double minTrusted;
        bool success = ParseDouble(value, minTrusted);
        if (!success) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid MinTrustedRange value.\n");
            return false;
        }
        mCalibrationInfo.MinTrustedRange = minTrusted;
    } else if (key == META_CAL_MIN_RANGE) {
        if (!ParseDouble(value, mCalibrationInfo.MinCalRange)) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid MinCalRange value.\n");
            return false;
        }
    } else if (key == META_CAL_MAX_RANGE) {
        if (!ParseDouble(value, mCalibrationInfo.MaxCalRange)) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid MaxCalRange value.\n");
            return false;
        }
    } else if (key == META_NUM_OF_RANGE_SEGMENTS) {
        if (!ParseUInt32(value, mCalibrationInfo.NumberOfSegments)) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid NumberOfSegments value.\n");
            return false;
        }
    } else if (key == META_RMS_RESIDUAL) {
        if (!ParseDouble(value, mCalibrationInfo.RMSResidual)) {
            mErrors.push_back(
                "Line " +
                std::to_string(lineNumber) +
                ": Invalid RMSResidual value.\n");
            return false;
        }
    } else {
        mWarnings.push_back(
            "Line " +
            std::to_string(lineNumber) +
            ": Unknown metadata field \"" +
            key +
            "\".\n");
    }

    return true;
}

//--------------------------------------------------------
//  ParseRangeModelLine
//--------------------------------------------------------
bool L2calibration::ParseRangeModelLine(
    const std::string& line,
    uint32_t lineNumber)
{
    if (mCalibrationInfo.RangeCalMethod == METHOD_CUBIC_SPLINE) {
        return ParseCubicSplineLine(line, lineNumber);
    } else {
        mErrors.emplace_back(
            "Unsupported range calibration method: " +
            mCalibrationInfo.RangeCalMethod);
        return false;
    }

    return true;
}

//--------------------------------------------------------
//  ParseAlphaAngleLUTline
//--------------------------------------------------------
bool L2calibration::ParseAlphaAngleLUTline(
    const std::string& line,
    uint32_t lineNumber)
{
    if (!mAlphaAngleLUTheaderRead) {

        std::vector<std::string> fields;

        SplitCSV(line, fields);

        if (fields.size() != ALPHA_ANLGE_LUT_FIELDS.size()) {
            mErrors.emplace_back(
                "Line " + std::to_string(lineNumber) +
                ": Invalid Alpha angle LUT header field count.\n" +
                "Expected: " + std::to_string(ALPHA_ANLGE_LUT_FIELDS.size()) +
                " labels but found: " + std::to_string(fields.size())
                );
            return false;
        }

        for (size_t i = 0; i < ALPHA_ANLGE_LUT_FIELDS.size(); ++i)
        {
            if (fields[i] != ALPHA_ANLGE_LUT_FIELDS[i]) {
                mErrors.emplace_back(
                    "Line " +
                    std::to_string(lineNumber) +
                    ": Alpha angle LUT header field " +
                    std::to_string(i + 1) +
                    " expected \"" +
                    ALPHA_ANLGE_LUT_FIELDS[i] +
                    "\" but found \"" +
                    fields[i] +
                    "\".");
                return false;
            }
        }

        mAlphaAngleLUTFieldCount =
            static_cast<uint32_t>(fields.size());

        mAlphaAngleLUTheaderRead = true;

        return true;
    }

    //
    // parse numeric values
    //
    std::vector<std::string> fields;

    SplitCSV(line, fields);

    if (fields.size() != mAlphaAngleLUTFieldCount) {
        mErrors.emplace_back(
            "Line " + std::to_string(lineNumber) +
            ": Invalid Alpha angle LUT field count.\n"
            "Expected: " +
            std::to_string(mAlphaAngleLUTFieldCount) +
            " fields but found: " +
            std::to_string(fields.size()));
        return false;
    }

    double field_dbl[MAX_ALPHA_ANGLE_LUT_FIELDS];

    // loop for field count
    // parse x0
    for(uint32_t i=0; i<mAlphaAngleLUTFieldCount; i++)
    {
        if (!ParseDouble(fields[i], field_dbl[i])) {
            mErrors.emplace_back(
                "Line " + std::to_string(lineNumber) +
                ": Invalid Alpha angle LUT coefficient: field# " + std::to_string(i+1) + " : " +
                fields[i]);
            return false;
        }
    }
    // AlphaAngleLUTFields
    // mAlphaAngleLUT

    AlphaAngleLUTFields angles;

    angles.fieldCount = mAlphaAngleLUTFieldCount;
    for(uint32_t i=0; i<angles.fieldCount; i++)
    {
        angles.fields[i] = field_dbl[i];
    }

    mAlphaAngleLUTfields.emplace_back(std::move(angles));
    return true;
}

//--------------------------------------------------------
//  ParseCalibrationPointLine
//--------------------------------------------------------
bool L2calibration::ParseCalibrationPointLine(
    const std::string& line,
    uint32_t lineNumber)
{
    if (!mCalibrationPointsHeaderRead)
    {
        std::vector<std::string> fields;

        SplitCSV(line, fields);

        if (fields.size() != CALIBRATION_POINT_FIELDS.size()) {
            mErrors.emplace_back(
                "Line " + std::to_string(lineNumber) +
                ": Invalid Calibration point header field count.\n" +
                "Expected: " + std::to_string(CALIBRATION_POINT_FIELDS.size()) +
                " labels but found: " + std::to_string(fields.size())
                );
            return false;
        }

        for (size_t i = 0; i < CALIBRATION_POINT_FIELDS.size(); ++i)
        {
            if (fields[i] != CALIBRATION_POINT_FIELDS[i]) {
                mErrors.emplace_back(
                    "Line " +
                    std::to_string(lineNumber) +
                    ": CALIBRATION POINT header: field #" +
                    std::to_string(i + 1) +
                    " expected: " +
                    CALIBRATION_POINT_FIELDS[i] +
                    " but found: " + fields[i]);
                return false;
            }
        }

        mExpectedCalibrationPointFieldCount =
            static_cast<uint32_t>(fields.size());

        mCalibrationPointsHeaderRead = true;

        return true;
    }

    //
    // parse numeric values
    //
    std::vector<std::string> fields;

    SplitCSV(line, fields);

    if (fields.size() != mExpectedCalibrationPointFieldCount) {
        mErrors.emplace_back(
            "Line " + std::to_string(lineNumber) +
            ": Invalid CALIBRATION POINT field count.\n"
            "Expected: " +
            std::to_string(mExpectedCalibrationPointFieldCount) +
            " fields but found: " +
            std::to_string(fields.size()));
        return false;
    }

    double field_dbl[MAX_RANGE_MODEL_FIELDS];

    // loop for field count
    for(uint32_t i=0; i<mExpectedCalibrationPointFieldCount; i++)
    {
        if (!ParseDouble(fields[i], field_dbl[i])) {
            mErrors.emplace_back(
                "Line " + std::to_string(lineNumber) +
                ": Invalid Calibration points: field# " + std::to_string(i+1) + " : " +
                fields[i]);
            return false;
        }
    }
    RangeCalibrationPoint datapoint;

    datapoint.MeasuredRange = field_dbl[0];
    datapoint.TrueRange = field_dbl[1];
    datapoint.Correction = field_dbl[2];

    mRangeCalibrationPoints.emplace_back(std::move(datapoint));

    return true;
}

//--------------------------------------------------------
//  Trim
//--------------------------------------------------------
std::string L2calibration::Trim(const std::string& text)
{
    auto first = std::find_if_not(
        text.begin(),
        text.end(),
        [](unsigned char c)
        {
            return std::isspace(c);
        });

    auto last = std::find_if_not(
                    text.rbegin(),
                    text.rend(),
                    [](unsigned char c)
                    {
                        return std::isspace(c);
                    }).base();

    if (first >= last) {
        return {};
    }

    return std::string(first, last);
}

//--------------------------------------------------------
//  SplitKeyValue
//--------------------------------------------------------
bool L2calibration::SplitKeyValue(
    const std::string& line,
    std::string& key,
    std::string& value)
{
    key.clear();
    value.clear();

    if (line.empty()) {
        return false;
    }

    std::string text = line;

    if (text.front() == '#') {
        text.erase(0, 1);
    }

    text = Trim(text);

    const size_t comma = text.find(',');

    if (comma == std::string::npos) {
        return false;
    }

    key = Trim(text.substr(0, comma));
    value = Trim(text.substr(comma + 1));

    return !key.empty();
}

//--------------------------------------------------------
//  IsCommentLine
//--------------------------------------------------------
bool L2calibration::IsCommentLine(const std::string& line)
{
    if (line.size() >= 2) {
        if (line.compare(0, 2, "##") == 0)
            return true;

        if (line.compare(0, 2, "//") == 0)
            return true;
    }

    if (!line.empty()) {
        if (line[0] == ';')
            return true;
    }

    return false;
}

//--------------------------------------------------------
//  IsCommentLine
//--------------------------------------------------------
void L2calibration::SplitCSV(
    const std::string& line,
    std::vector<std::string>& fields)
{
    fields.clear();

    size_t start = 0;

    while (true)
    {
        size_t comma = line.find(',', start);

        if (comma == std::string::npos) {
            fields.emplace_back(Trim(line.substr(start)));
            break;
        }

        fields.emplace_back(
            Trim(line.substr(start, comma - start)));

        start = comma + 1;
    }
}

//--------------------------------------------------------
//  ParseDouble
//--------------------------------------------------------
bool L2calibration::ParseDouble(
    const std::string& text,
    double& value)
{
    try
    {
        size_t index = 0;

        value = std::stod(text, &index);

        return index == text.length();
    }

    catch (const std::invalid_argument&)
    {
        return false;
    }

    catch (const std::out_of_range&)
    {
        return false;
    }
}

//--------------------------------------------------------
//  ParseUInt32
//--------------------------------------------------------
bool L2calibration::ParseUInt32(
    const std::string& text,
    uint32_t& value)
{
    try
    {
        size_t index = 0;

        unsigned long result = std::stoul(text, &index);

        if (index != text.length()) {
            return false;
        }

        value = static_cast<uint32_t>(result);

        return true;
    }

    catch (const std::invalid_argument&)
    {
        return false;
    }

    catch (const std::out_of_range&)
    {
        return false;
    }
}

//--------------------------------------------------------
//  ParseUInt32
//--------------------------------------------------------
bool L2calibration::ParseInt32(
    const std::string& text,
    int32_t& value)
{
    try
    {
        size_t index = 0;

        unsigned long result = std::stoul(text, &index);

        if (index != text.length())
        {
            return false;
        }

        value = static_cast<int32_t>(result);

        return true;
    }

    catch (const std::invalid_argument&)
    {
        return false;
    }

    catch (const std::out_of_range&)
    {
        return false;
    }
}

//--------------------------------------------------------
//  ResetParserState
//--------------------------------------------------------
void L2calibration::ResetParserState()
{
    // Reset parser state.
    mParseState = ParseState::Metadata;

    // Clear calibration data.
    mRangeModelHeaderRead = false;
    mExpectedModelFieldCount = 0;
    mRangeCorrectionModel.clear();

    mAlphaAngleLUTheaderRead = false;
    mAlphaAngleLUTFieldCount = 0;
    mAlphaAngleLUTfields.clear();

    mCalibrationPointsHeaderRead = false;
    mExpectedCalibrationPointFieldCount = 0;
    mRangeCalibrationPoints.clear();

    mRangeCorrectionLUT.clear();
    mMinCalibratedRange = 65535.0;
    mMaxCalibratedRange = 0.0;

    // Reset metadata.
    mCalibrationInfo = {};

    // Clear informational messages.
    mWarnings.clear();
    mErrors.clear();
}

//--------------------------------------------------------
//  ParseCubicSplineLine
//--------------------------------------------------------
bool L2calibration::ParseCubicSplineLine(
    const std::string& line,
    uint32_t lineNumber)
{
    if (!mRangeModelHeaderRead) {

        std::vector<std::string> fields;

        SplitCSV(line, fields);

        if (fields.size() != CUBICSPLINE_FIELDS.size()) {
            mErrors.emplace_back(
                "Line " + std::to_string(lineNumber) +
                ": Invalid Range MODEL header field count.\n" +
                "Expected: " + std::to_string(CUBICSPLINE_FIELDS.size()) +
                " labels but found: " + std::to_string(fields.size())
                );
            return false;
        }

        for (size_t i = 0; i < CUBICSPLINE_FIELDS.size(); ++i)
        {
            if (fields[i] != CUBICSPLINE_FIELDS[i]) {
                mErrors.emplace_back(
                    "Line " +
                    std::to_string(lineNumber) +
                    ": Range MODEL header field " +
                    std::to_string(i + 1) +
                    " expected \"" +
                    CUBICSPLINE_FIELDS[i] +
                    "\" but found \"" +
                    fields[i] +
                    "\".");
                return false;
            }
        }

        mExpectedModelFieldCount =
            static_cast<uint32_t>(fields.size());

        mRangeModelHeaderRead = true;

        return true;
    }

    //
    // parse numeric values
    //
    std::vector<std::string> fields;

    SplitCSV(line, fields);

    if (fields.size() != mExpectedModelFieldCount) {
        mErrors.emplace_back(
            "Line " + std::to_string(lineNumber) +
            ": Invalid Range MODEL field count.\n"
            "Expected: " +
            std::to_string(mExpectedModelFieldCount) +
            " fields but found: " +
            std::to_string(fields.size()));
        return false;
    }

    double field_dbl[MAX_RANGE_MODEL_FIELDS];

    // loop for field count
    // parse x0
    for(uint32_t i=0; i<mExpectedModelFieldCount; i++)
    {
        if (!ParseDouble(fields[i], field_dbl[i])) {
            mErrors.emplace_back(
                "Line " + std::to_string(lineNumber) +
                ": Invalid Range MODEL coefficient: field# " + std::to_string(i+1) + " : " +
                fields[i]);
            return false;
        }
    }
    RangeModelFields segment;

    segment.fieldCount = mExpectedModelFieldCount;
    for(uint32_t i=0; i<segment.fieldCount; i++)
    {
        segment.fields[i] = field_dbl[i];
    }

    mRangeCorrectionModel.emplace_back(std::move(segment));

    return true;
}

//--------------------------------------------------------
//  ValidateMeta
//--------------------------------------------------------
bool L2calibration::ValidateMeta(const CalibrationInfo& info)
{
    if(info.MinTrustedRange<0){
        mWarnings.emplace_back(
            "MinTrustedRange not set\n"
            );
    }

    if(info.MinRange >= info.MaxRange) {
        mErrors.emplace_back(
            "MinRange >= MaxRange"
            );
        return false;
    }
    if((info.MinTrustedRange >= 0 ) && info.MinTrustedRange < info.MinRange) {
        // MinTrustRange = -1 means not specified
        mErrors.emplace_back(
            "MinTrustedRange < MinRange"
            );
        return false;
    }

    if(info.RangeCalMethod!="None") {
        if(info.MinCalRange >= info.MaxCalRange) {
            mErrors.emplace_back(
                "MinCalRange >= MaxCalRange"
                );
            return false;
        }
    }

    if(info.MinTrustedRange >= info.MaxRange) {
        mErrors.emplace_back(
            "MinTrustedRange >= MaxRange"
            );
        return false;
    }
    return true;
}

//--------------------------------------------------------
//  ValidateRangeCalibration
//--------------------------------------------------------
bool L2calibration::ValidateRangeCalibration(const std::vector<RangeModelFields> RangeCorrectionModel)
{    
    if (mCalibrationInfo.RangeCalMethod != "CubicSpline") {
        mErrors.emplace_back(
                "Unsupported range calibration method: " +
                mCalibrationInfo.RangeCalMethod);
        return false;
    }

    // NOTE: The current Cubic Spline implementation does requires
    // the spline segments to cover the full specified call range
    // or it will be flagged as a n error
    return ValidateCubicSpline(RangeCorrectionModel);
}

//--------------------------------------------------------
//  BuildRangeCorrectionLUT
//--------------------------------------------------------
bool L2calibration::BuildRangeCorrectionLUT(const std::vector<RangeModelFields> RangeCorrectionModel)
{

    if (mCalibrationInfo.RangeCalMethod != "CubicSpline") {
        mErrors.emplace_back(
            "Unsupported range calibration method: " +
            mCalibrationInfo.RangeCalMethod);
        return false;
    }

    return BuildCubicSplineLUT(RangeCorrectionModel);
}

//--------------------------------------------------------
//  ValidateCubicSpline
//--------------------------------------------------------
bool L2calibration::ValidateAlphaAngleLUT()
{
    //
    // Verify at least one model segment exists.
    //
    mAlphaLUTvalid = false;
    mAlphaAngleLUT.clear();

    if (mAlphaAngleLUTfields.empty()) {
        mWarnings.emplace_back(
            "Calibration file does not contain Alpha Angle LUT section");
        return true;
    }
    //
    // Verify entries is NUM_ALPHA_ANGLES_IN_SCAN
    //
    if (mAlphaAngleLUTfields.size() != NUM_ALPHA_ANGLES_IN_SCAN) {
        mErrors.emplace_back(
            "Alpha Angle LUT section contains " +
            std::to_string(mRangeCorrectionModel.size()) +
            " angles.\n" +
            "Requirement is " +
            std::to_string(NUM_ALPHA_ANGLES_IN_SCAN) +
            " only found: " +
            std::to_string(mCalibrationInfo.NumberOfSegments));
        return false;
    }

    //
    // Verify each index.
    //
    int lastindexnum = mAlphaAngleLUTfields[0].fields[FIELD_INDEX];
    if(lastindexnum!=0) {
        mErrors.emplace_back(
            "AlphaAngleLUT[0] index number must be 0");
        return false;
    }
    for (size_t i = 1; i < mAlphaAngleLUTfields.size(); ++i)
    {
        const auto& entry = mAlphaAngleLUTfields[i];

        const int indexnum = entry.fields[FIELD_INDEX];

        if (lastindexnum != (indexnum-1)) {
            mErrors.emplace_back(
                "AlphaAngleLUT["
                +std::to_string(i)+ "] index is not in sequential order");
            return false;
        }

        lastindexnum = indexnum;
    }

    mAlphaLUTvalid = true;

    return true;
}

//--------------------------------------------------------
//  ValidateCubicSpline
//--------------------------------------------------------
bool L2calibration::ValidateCubicSpline(
            const std::vector<RangeModelFields> RangeCorrectionModel)
{
    //
    // Verify at least one model segment exists.
    //
    if (RangeCorrectionModel.empty()) {
        mErrors.emplace_back(
            "Range MODEL section contains no CubicSpline segments.");
        return false;
    }


    // Verify segment count matches metadata only if not a CandidateModel
    if (RangeCorrectionModel.size() !=
        mCalibrationInfo.NumberOfSegments) {
        mErrors.emplace_back(
            "Range MODEL section contains " +
            std::to_string(mRangeCorrectionModel.size()) +
            " segments.\n" +
            "Metadata specifies NumberOfSegments = " +
            std::to_string(mCalibrationInfo.NumberOfSegments));
        return false;
    }

    // Verify MinCalRange >= MinRange
    if(mCalibrationInfo.MinCalRange < mCalibrationInfo.MinRange) {
        mErrors.emplace_back(
            "Min Calibration range: " +
            std::to_string(mCalibrationInfo.MinCalRange) +
            " < device Min Range " +
            std::to_string(mCalibrationInfo.MinRange)
            );
        return false;
    }

    // Verify MaxCalRange <= MaxRange
    if(mCalibrationInfo.MaxCalRange > mCalibrationInfo.MaxRange) {
        mErrors.emplace_back(
            "Max Calibration range: " +
            std::to_string(mCalibrationInfo.MaxCalRange) +
            " > device Max Range " +
            std::to_string(mCalibrationInfo.MaxRange)
            );
        return false;
    }

    // Make sure MinCalRange < MaxCalRange
    if(mCalibrationInfo.MinCalRange >= mCalibrationInfo.MaxCalRange) {
        mErrors.emplace_back(
            "Calibration range is invalid\n Min: " +
            std::to_string(mCalibrationInfo.MinCalRange) +
            " must be less than max: " +
            std::to_string(mCalibrationInfo.MaxCalRange));
        return false;
    }

    // The spline segments must encompass that cal range
    double StartCalRange;
    double EndCalRange;

    StartCalRange = mRangeCorrectionModel.front().fields[0]; // starting x0
    EndCalRange = mRangeCorrectionModel.back().fields[1]; // ending x1

    if(mCalibrationInfo.MinCalRange < StartCalRange) {
        mErrors.emplace_back(
            "Min Cal range: " +
            std::to_string(mCalibrationInfo.MinCalRange) +
            " must be >= the start of the first spline segment: " +
            std::to_string(StartCalRange));
        return false;
    }
    if(mCalibrationInfo.MaxCalRange > EndCalRange){
        mErrors.emplace_back(
            "Max Cal range: " +
            std::to_string(mCalibrationInfo.MinCalRange) +
            " must be <= the end of the last spline segment: " +
            std::to_string(EndCalRange));
        return false;
    }

    // Verify each segment.
    //
    // These values are diagnostic only
    double MinCalRange {65535.0};
    double MaxCalRange {0.0};

    for (size_t i = 0; i < RangeCorrectionModel.size(); ++i)
    {
        const auto& segment = RangeCorrectionModel[i];

        const double x0 = segment.fields[FIELD_X0];
        const double x1 = segment.fields[FIELD_X1];

        if (x1 <= x0) {
            mErrors.emplace_back(
                "Range MODEL segment " +
                std::to_string(i + 1) +
                ": x1 must be greater than x0.");
            return false;
        }

        if(MinCalRange > x0) MinCalRange = x0;
        if(MaxCalRange < x1) MaxCalRange = x1;
    }

    //
    // Verify adjacent segment continuity.
    //
    for (size_t i = 0; i + 1 < RangeCorrectionModel.size(); ++i)
    {
        const auto& current = RangeCorrectionModel[i];
        const auto& next    = RangeCorrectionModel[i + 1];

        const double currentEnd = current.fields[FIELD_X1];
        const double nextStart  = next.fields[FIELD_X0];

        if (std::abs(currentEnd - nextStart) >
            SEGMENT_CONNECTION_TOLERANCE) {
            mErrors.emplace_back(
                "Range MODEL segments " +
                std::to_string(i + 1) +
                " and " +
                std::to_string(i + 2) +
                " are not contiguous.");
            return false;
        }

        if (next.fields[FIELD_X0] <= current.fields[FIELD_X0]) {
            mErrors.emplace_back(
                "Range MODEL segments " +
                std::to_string(i + 1) +
                " and " +
                std::to_string(i + 2) +
                " are not increasing.");
            return false;
        }
    }

    return true;
}

//--------------------------------------------------------
//  BuildCubicSplineLUT
//
// Generate the range correction lookup table from the validated
// CubicSpline calibration model.
//
// Preconditions:
//   - CalibrationMethod == "CubicSpline"
//   - ValidateRangeCalibration() has succeeded.
//
// The generated LUT contains the corrected range (mm) for every
// possible raw range value (0..65535).
//
//--------------------------------------------------------
bool L2calibration::BuildCubicSplineLUT(const std::vector<RangeModelFields>& rangeCorrectionModel)
{
    if (rangeCorrectionModel.empty()) {
        return false;
    }


    //----------------------------------------------------
    // Select the LUT being generated.
    //----------------------------------------------------
    std::vector<double>& targetLUT = mRangeCorrectionLUT;

    targetLUT.resize(65536);

    //----------------------------------------------------
    // Determine the calibrated application range.
    //----------------------------------------------------
    const double minCalRange = mCalibrationInfo.MinCalRange;
    const double maxCalRange = mCalibrationInfo.MaxCalRange;

    if (!(minCalRange < maxCalRange)) {
        return false;
    }

    //----------------------------------------------------
    // Evaluate the spline at an arbitrary range within
    // the model extent.
    //----------------------------------------------------
    const auto evaluateSpline = [&rangeCorrectionModel](const double range, double& correction) -> bool
    {
        for (size_t segmentIndex = 0;
             segmentIndex < rangeCorrectionModel.size();
             ++segmentIndex)
        {
            const RangeModelFields& segment = rangeCorrectionModel[segmentIndex];

            const double x0 = segment.fields[FIELD_X0];
            const double x1 = segment.fields[FIELD_X1];
            const bool lastSegment = segmentIndex + 1 == rangeCorrectionModel.size();

            //
            // Internal segments use [x0, x1).
            // The final segment includes x1.
            //
            if (range < x0 || ((!lastSegment && range >= x1) || (lastSegment && range > x1))) {
                continue;
            }

            const double a = segment.fields[FIELD_A];
            const double b = segment.fields[FIELD_B];
            const double c = segment.fields[FIELD_C];
            const double d = segment.fields[FIELD_D];
            const double dx = range - x0;
            correction = ((d * dx + c) * dx + b) * dx + a;

            return true;
        }

        return false;
    };

    //----------------------------------------------------
    // Evaluate the correction at the operational
    // calibration limits. These values are used for
    // constant endpoint extrapolation.
    //----------------------------------------------------
    double minimumCorrection = 0.0;
    double maximumCorrection = 0.0;

    if (!evaluateSpline(minCalRange, minimumCorrection)) {
        return false;
    }

    if (!evaluateSpline(maxCalRange, maximumCorrection)){
        return false;
    }

    //----------------------------------------------------
    // Generate the lookup table.
    //----------------------------------------------------
    size_t currentSegment = 0;

    double x0 = rangeCorrectionModel[currentSegment].fields[FIELD_X0];
    double x1 = rangeCorrectionModel[currentSegment].fields[FIELD_X1];
    double a = rangeCorrectionModel[currentSegment].fields[FIELD_A];
    double b = rangeCorrectionModel[currentSegment].fields[FIELD_B];
    double c = rangeCorrectionModel[currentSegment].fields[FIELD_C];
    double d = rangeCorrectionModel[currentSegment].fields[FIELD_D];

    for (uint32_t rawRange = 0; rawRange < 65536; ++rawRange)
    {
        const double range = static_cast<double>(rawRange);

        //------------------------------------------------
        // Below the calibrated range, retain the
        // correction at MinCalRange.
        //------------------------------------------------
        if (range < minCalRange) {
            targetLUT[rawRange] = minimumCorrection;
            continue;
        }

        //------------------------------------------------
        // Above the calibrated range, retain the
        // correction at MaxCalRange.
        //------------------------------------------------
        if (range >= maxCalRange) {
            targetLUT[rawRange] = maximumCorrection;
            continue;
        }

        //------------------------------------------------
        // Advance to the spline segment containing the
        // current range.
        //------------------------------------------------
        while (range >= x1)
        {
            ++currentSegment;

            if (currentSegment >= rangeCorrectionModel.size()) {
                return false;
            }

            const RangeModelFields& segment = rangeCorrectionModel[currentSegment];

            x0 = segment.fields[FIELD_X0];
            x1 = segment.fields[FIELD_X1];

            a = segment.fields[FIELD_A];
            b = segment.fields[FIELD_B];
            c = segment.fields[FIELD_C];
            d = segment.fields[FIELD_D];
        }
        const double dx = range - x0;
        targetLUT[rawRange] = ((d * dx + c) * dx + b) * dx + a;
    }
    return true;
}

//--------------------------------------------------------
//  GetRangeCorrectionLUT
//--------------------------------------------------------
const std::vector<double>& L2calibration::GetRangeCorrectionLUT() const noexcept
{
    return mRangeCorrectionLUT;
}

//--------------------------------------------------------
//  IsAlphaAngleLUTvalid
//--------------------------------------------------------
bool L2calibration::IsAlphaAngleLUTloaded()
{
    return mAlphaLUTvalid;
}

//--------------------------------------------------------
//  GetAlphaAngleLUT
//--------------------------------------------------------
const std::vector<double>& L2calibration::GetAlphaAngleLUT() const noexcept
{
    return mAlphaAngleLUT;
}
