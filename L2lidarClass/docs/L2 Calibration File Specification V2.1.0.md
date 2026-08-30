# L2 Calibration File Specification for the L2LIdar class

> **Status:** Released
> 
> **Document Version:** 2.1.0 
> **Calibration File Version:** 2.1.0
> 
> This document is the normative specification for the Unitree L2 Range
> Calibration File used by `L2lidar class software`.

---

# Revision History

| Version | Date       | Description                                                                                                                                                                                                                         |
| ------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2.0.0   | 2026-08-21 | Initial release of the Unitree L2 Calibration File specification.                                                                                                                                                                   |
| 2.0.1   | 2026-08-25 | Added AlphaAngleStepSize override to metadata, this can be used when there is not AlphaAngleLUT and the stepsize is uniform.  Updated the Alpha angle LUT definition.  It contains the angle increment for the next elevation step. |
| 2.1.0   | 2026-08-27 | Changed specification to allow the range correction to be optional.                                                                                                                                                                 |

# 1. Introduction

This document defines the Version 2.0.0 calibration file format used to
describe the calibration corrections for the Unitree L2 LiDAR implemented in the L2lidar class software.

The calibration is represented by a model that maps measured
range to corrected range and maps measured angles to corrected angles of the fast scan angles (Alpha).

The calibration file contents:

- Meta data specifying the conditions to be used with the calibration corrections.  The Meta data includes all the calibration overrides.  The calibration overrides are always present even if there is no Range model or Elevation LUT.

- The Range model is converted into a runtime lookup table (LUT) used for efficient range correction.

- Optional map of the fast scan angles is a LUT for each of the 300 scan angles in the fast scan.

---

# 2. Scope

This specification defines:

- Calibration file format
- Metadata
- Model representation
- Calibration point storage
- Runtime processing requirements

This document does **not** define:

- Calibration generation
- Model fitting algorithms
- Sensor calibration procedures

---

# 3. Units

Unless otherwise specified:

| Quantity        | Units                 |
| --------------- | --------------------- |
| Raw Range       | millimeters (integer) |
| Measured Range  | millimeters           |
| Corrected Range | millimeters           |
| True Range      | millimeters           |
| Correction      | millimeters           |
| Angles          | degrees               |

The calibration file stores ditances values in millimeters and angle measurements in degrees.

Conversion to meters is performed elsewhere within the LiDAR processing
pipeline uing the RangeScale parameter.

---

# 4. File Format

The calibration file is an ASCII CSV text file.

UTF-8 encoding is recommended.

Blank lines are permitted.

Leading and trailing whitespace surrounding CSV fields should be ignored.

Metadata lines begin with:

```text
#
```

Section headers also begin with `#`.

Unknown metadata fields shall be ignored.

Applications may optionally generate a warning.

## Comments

Comment lines may appear anywhere in the file.

A comment line is any line whose first non-whitespace characters are one of:

- `##`
- `;`
- `//`

Comment lines are ignored by the parser and have no effect on the calibration data.

---

# 5. Metadata

Metadata consists of one key/value pair per line.

Example:

```text
# Version,2.1.0
```

## Required Metadata

| Field               | Description                              |
| ------------------- | ---------------------------------------- |
| Version             | Calibration file version                 |
| Date                | Calibration creation date                |
| Sensor              | Sensor model                             |
| CalibrationMethod   | Calibration generation method            |
| RangeBias           | Integer range bias (mm)                  |
| RangeScale          | conversion and scaling from mm to meters |
| AlphaAngleBias      | Offset for zero elevation (degrees)      |
| AlphaAngleStepSize  | Step size for elevation (degrees)        |
| ThetaAngleBias      | Offset for zero azimuth (degrees)        |
| BetaAngle           | degrees                                  |
| XiAngle             | degrees                                  |
| NumberRangeSegments | Number of Range model segments           |
| MinRange            | Minimum range (mm)                       |
| MaxRange            | Maximum range (mm)                       |
| MinCalRange         | Minimum calibrated range (mm)            |
| MaxCalRange         | Maximum calibrated range (mm)            |

## Optional Metadata

| Field                  | Description                                                         |
| ---------------------- | ------------------------------------------------------------------- |
| SensorID               | User-defined sensor identifier                                      |
| Firmware               | Sensor firmware version                                             |
| CreatedBy              | Application or user creating the calibration                        |
| CalibrationDescription | Free-form description                                               |
| MinTrustedRange        | Range values less than this may be returned but may not be reliable |
| RMSResidual            | RMS Range model fit residual (mm)                                   |

---

# 6. CalibrationMethod metadata

Version 2.1.0 defines:

If RANGE CORRECTION model data included in the file:

```text
RangeCorrectionMethod,CubicSpline
```

If no RANGE CORRRECTION model data included in the file:

```
RangeCorrectionMethod,None
```

Applications shall reject unsupported calibration methods.

---

# 7. RANGE MODEL Section (optional)

The RANGE MODEL section is an optional section.

The CALIBRATION POINTS section is only included if there is a RANGE MODEL section inclued in the file.

Section header:

```text
# RANGE MODEL
```

The MODEL section syntax is defined by the value of `CalibrationMethod`.



For RangeCorrectionMethod - CubicSpline the format is:

Column definition for a cubic spline:

```text
x0,x1,a,b,c,d
```

Each row defines one spline segment using floating point notation for each field.  For example:

```
1050.0, 1200.0,-3.3204, 0.9987, -2.10e-5, 1.50e-7
1200.0, 1300.0,-3.3204, 1.0, 0.0, 0.0
```

The spline equation is

```text
f(x)=a+b(x−x0)+c(x−x0)^2+d(x−x0)^3
```

valid over

```text
x0 ≤ x ≤ x1
```

where all values are expressed in millimeters.

## Requirements

- Segments shall be stored in increasing x order.
- Adjacent segments shall be continuous.
- The number of spline rows shall equal `NumberOfSegments`.

---

# 7. FAST SCAN ANGLE LUT Section (optional)

The FAST SCAN ANGLE LUT section is optional

Section header:

```
# ALPHA ANGLE LUT
```

Column definition:
    FastScanIndex, RelativeAngle

Each row defines one spline segment using floating point notation for each field. For example:

```
0, 0.6020
1, 0.6020
...
299, 0.6020
```

where
    relative angle[i] is the angle increment to the next elevation step.
    where i is the fast scan angle index

    Index 299 is not used since there is not next elevation step.

## Requirements

- Exactly 300 entries

- Indices exactly 0..299

- No duplicates or missing indices

- All angles finite

---

# 8. CALIBRATION POINTS Section (optional)

The CALIBRATION POINTS section is only included if there is a RANGE MODEL section inclued in the file.

Section header:

```text
# CALIBRATION POINTS
```

Column definition:

```text
MeasuredRange,TrueRange,Correction
```

where

```text
Correction = TrueRange − MeasuredRange
```

These points preserve the original calibration dataset.

They are intended for diagnostics and documentation.

Runtime correction does not require these points.

---

# 9. Runtime Processing

Runtime processing shall be performed as follows:

```text
RawRange (uint16)

        │

        ▼

RangeBias

        │

        ▼

Range Correction LUT

        │

        ▼

Corrected Range (mm)
```

The model shall **not** be evaluated during runtime.

The runtime LUT shall be generated during
`LoadRangeCalibration()`.

---

# 10. RangeScale

`RangeScale` is set after the range correction calibration is complete.

The nonlinear calibration operates entirely within the sensor's native
measurement domain (millimeters).

Conversion from millimeters to meters is set by RangeScales.

---

# 11. Version Compatibility

Applications shall reject unsupported major versions.

Unknown metadata fields should be ignored.

Future versions may introduce:

- Additional metadata fields
- Additional sections
- Additional calibration methods

while maintaining backward compatibility whenever practical.

---

# 12. Example

```text
# Version,2.0.0
# Date,2026-07-12
# Sensor,Unitree L2
# SensorID,L2-Front
# Firmware,1.3.4
# CreatedBy,L2Diagnostics 2.0
# RangeCorrectionMethod,CubicSpline
# CalibrationDescription,Indoor wall calibration
# RangeBias,-530
# RangeScale, 0.001
# AlphaAngleBias, 1.75
# AlphaAngleStepSize, 0.602
# ThetaAngleBias, 120.0
# BetaAngle, 0.25
# XiAngle, 0.25
# NumberOfSegments,48
# MinRange,150
# MaxRange,40000
# MinTrustedRange,1450
# MinCalRange,1650
# MaxCalRange,4670
# RMSResidual,8.12

# RANGE MODEL

x0,x1,a,b,c,d
1050.0,1125.0,-0.3204,0.9987,-2.10e-5,1.50e-7
...

# CALIBRATION POINTS
MeasuredRange,TrueRange,Correction
1053.2,1050.0,-3.2
...
```

---

# Appendix A – Design Philosophy

## A.1 Separation of Corrections

The Unitree L2 processing pipeline contains three independent L2 calibration corrections:

1. **Override parameters for builtin calibration parameters**
2. **Range Linearity Correction**
3. **Fast scan angle LUT**

---

## A.2 Native Measurement Domain

The calibration operates entirely in millimeters.

Maintaining the calibration within the sensor's native measurement domain
avoids introducing additional scaling errors and allows `RangeScale` to be
adjusted independently without regenerating the calibration.

---

## A.3 Authoritative Representation

The range model is the authoritative mathematical representation
of the range calibration.

The runtime range lookup table (LUT) is derived from the model during loading.

The range LUT is intentionally excluded from the calibration file because it can
always be regenerated.

---

## A.4 Runtime Performance

Runtime range correction shall consist of a single lookup operation.

Model evaluation occurs only during
`LoadRangeCalibration()`.

This minimizes computational cost while preserving the accuracy of the model.

---

## A.5 File Longevity

The calibration file stores only information that cannot be regenerated.

The file contains:

- Metadata
- Model coefficients
- Original calibration points

Derived runtime data, including lookup tables, are intentionally excluded.

---

## A.6 Design Goals

Version 2.0.1 was designed to satisfy the following objectives:

- Human-readable and version controlled
- Simple to parse
- Efficient at runtime
- Platform independent
- Extensible to future calibration methods
- Stable over long-term use

* * *

## A.7 Range limit heirarchy

**MinRange**
    Physical sensor return limit.
    Below this, no valid sensor measurement exists.

**MinTrustedRange**
    Empirically determined nearest range at which the
    returned measurement is sufficiently reliable and
    single-valued for practical use.  This is an optional parameter.
    It has a value of -1 if not set.

**MinCalRange**
    Lowest range actually covered by the generateds
    calibration model. (Note: This may be lower than MinTrustedRange)

**MaxCalRange**
    Highest range actually covered by the generated
    calibration model.

**MaxRange**
    Physical upper sensor return limit.
