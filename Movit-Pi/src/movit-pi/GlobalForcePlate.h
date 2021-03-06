#ifndef GLOBAL_FORCE_PLATE_H
#define GLOBAL_FORCE_PLATE_H

#include "Utils.h"
#include "DataType.h"

class GlobalForcePlate
{
  public:
    //Constants - physical montage values
    enum Quadrant
    {
        FrontLeft = 0,
        FrontRight = 1,
        BackLeft = 2,
        BackRight = 3,
        Count = 4
    };

    GlobalForcePlate();

    float GetFx() { return _fx; }
    float GetFy() { return _fy; }
    float GetFz() { return _fz; }
    float GetMx() { return _mx; }
    float GetMy() { return _my; }
    float GetMz() { return _mz; }
    float GetMx1() { return _mx1; }
    float GetMy1() { return _my1; }

    Coord_t GetCenterOfPressure() { return _centerOfPressure; }

    void SetFx(float fx) { _fx = fx; }
    void SetFy(float fy) { _fy = fy; }
    void SetFz(float fz) { _fz = fz; }
    void SetMx(float mx) { _mx = mx; }
    void SetMy(float my) { _my = my; }
    void SetMz(float mz) { _mz = mz; }
    void SetMx1(float mx1) { _mx1 = mx1; }
    void SetMy1(float my1) { _my1 = my1; }

    void SetCenterOfPressure(Coord_t centerOfPressure) { _centerOfPressure = centerOfPressure; }

  private:
    //Force plate output signals
    float _fx12; //Force in X-Direction measured by SensorNo1 + SensorNo2
    float _fx34; //Force in X-Direction measured by SensorNo3 + SensorNo4
    float _fy14; //Force in Y-Direction measured by SensorNo1 + SensorNo4
    float _fy23; //Force in Y-Direction measured by SensorNo2 + SensorNo3
    float _fz1;  //
    float _fz2;  //Forces in Z-Direction
    float _fz3;  //measured by SensorNo1, ..., SensorNo4
    float _fz4;  //

    //Calculated parameters
    float _fx;  //Medio-lateral force
    float _fy;  //Anterior-posterior force
    float _fz;  //Vertical force
    float _mx;  //Plate Moment about X-Axis
    float _my;  //Plate Moment about Y-Axis
    float _mz;  //Plate Moment about Z-Axis
    float _mx1; //Plate Moment along X-Axis about top plate surface
    float _my1; //Plate Moment along Y-axis about top plate surface

    //Coordinate of the force application point (C.O.P.)
    Coord_t _centerOfPressure; //X-Coordinate of the application point
};

#endif // GLOBAL_FORCE_PLATE_H
