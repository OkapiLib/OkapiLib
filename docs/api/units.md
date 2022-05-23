# Units API

OkapiLib features a units API that compares quantities using dimensional analysis. This means that
if you multiply two length units together, you get an area unit; or if you divide a length unit by
a time unit, you get a velocity unit. These are just examples, OkapiLib supports much more than
just these.

## Length Units

Type name: `QLength`

| Unit       | Name         | Literal |
|------------|--------------|---------|
| Meter      | `meter`      | `_m`    |
| Decimeter  | `decimeter`  |         |
| Centimeter | `centimeter` | `_cm`   |
| Millimeter | `millimeter` | `_mm`   |
| Kilometer  | `kilometer`  | `_km`   |
| Inch       | `inch`       | `_in`   |
| Foot       | `foot`       | `_ft`   |
| Yard       | `yard`       | `_yd`   |
| Mile       | `mile`       | `_mi`   |
| Tile       | `tile`       | `_tile` |

## Speed Units

Type name: `QSpeed`

| Unit             | Name   | Literal |
|------------------|--------|---------|
| Meter / second   | `mps`  | `_mps`  |
| Mile / hour      | `miph` | `_miph` |
| Kilometer / hour | `kmph` | `_kmph` |

## Acceleration Units

Type name: `QAcceleration`

| Unit              | Name   | Literal |
|-------------------|--------|---------|
| meters / second^2 | `mps2` | `_mps2` |
| gravity           | `G`    | `_G`    |

## Jerk Units

Type name: `QJerk`

## Angle Units

Type name: `QAngle`

| Unit   | Name     | Literal |
|--------|----------|---------|
| Radian | `radian` | `_rad`  |
| Degree | `degree` | `_deg`  |

## Angular Speed Units

Type name: `QAngularSpeed`

| Unit              | Name    | Literal |
|-------------------|---------|---------|
| Radian / second   | `radps` |         |
| Rotation / minute | `rpm`   | `_rpm`  |

## Angular Acceleration Units

Type name: `QAngularAcceleration`

## Angular Jerk Units

Type name: `QAngularJerk`

## Time Units

Type name: `QTime`

| Unit        | Name          | Literal |
|-------------|---------------|---------|
| Second      | `second`      | `_s`    |
| Millisecond | `millisecond` | `_ms`   |
| Minute      | `minute`      | `_min`  |
| Hour        | `hour`        | `_h`    |
| Day         | `day`         | `_day`  |

## Frequency Units

Type name: `QFrequency`

| Unit  | Name | Literal |
|-------|------|---------|
| Hertz | `Hz` | `_Hz`   |

## Area Units

Type name: `QArea`

| Unit         | Name          | Literal |
|--------------|---------------|---------|
| Kilometer^2  | `kilometer2`  |         |
| Meter^2      | `meter2`      |         |
| Decimeter^2  | `decimeter2`  |         |
| Centimeter^2 | `centimeter2` |         |
| Millimeter^2 | `millimeter2` |         |
| Inch^2       | `inch2`       |         |
| Foot^2       | `foot2`       |         |
| Mile^2       | `mile2`       |         |

## Volume Units

Type name: `QVolume`

| Unit         | Name          | Literal |
|--------------|---------------|---------|
| Kilometer^3  | `kilometer3`  |         |
| Meter^3      | `meter3`      |         |
| Decimeter^3  | `decimeter3`  |         |
| Centimeter^3 | `centimeter3` |         |
| Millimeter^3 | `millimeter3` |         |
| Inch^3       | `inch3`       |         |
| Foot^3       | `foot3`       |         |
| Mile^3       | `mile3`       |         |
| Litre^3      | `litre3`      |         |

## Force Units

Type name: `QForce`

| Unit       | Name         | Literal |
|------------|--------------|---------|
| Newton     | `newton`     | `_n`    |
| Poundforce | `poundforce` | `_lbf`  |
| Kilopond   | `kilopond`   | `_kp`   |

## Torque Units

Type name: `QTorque`

| Unit         | Name          | Literal |
|--------------|---------------|---------|
| Newton-meter | `newtonMeter` | `_nM`   |
| Foot-pound   | `footPound`   | `_ftLb` |
| Inch-pound   | `inchPound`   | `_inLb` |

## Pressure Units

Type name: `QPressure`

| Unit   | Name     | Literal |
|--------|----------|---------|
| Pascal | `pascal` | `_Pa`   |
| Bar    | `bar`    | `_bar`  |
| PSI    | `psi`    | `_psi`  |

## Mass Units

Type name: `QMass`

| Unit     | Name     | Literal |
|----------|----------|---------|
| Kilogram | `kg`     | `_kg`   |
| Gramme   | `gramme` | `_g`    |
| Tonne    | `tonne`  | `_t`    |
| Ounce    | `ounce`  | `_oz`   |
| Pound    | `pound`  | `_lb`   |
| Stone    | `stone`  | `_st`   |
