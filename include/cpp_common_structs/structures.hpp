#ifndef STRUCTURES_HPP
#define STRUCTURES_HPP

#include <vector>

struct obstacle{
    double x, y, radius;
    double s, d, t;
};

// This can be used for a generic rectangular bounding box
typedef struct ObjectStruct {
    unsigned int    object_id;
    double          x;
    double          y;
    double          z;
    double          heading;
    double          velocity;
    double   		length;
    double   		width;

    double s, d, t;
    double local_heading;
} ObjectStruct;

struct FrenetPath{
    std::vector<double> t, d, d_d, d_dd, d_ddd, s, s_d, s_dd, s_ddd;
    std::vector<double> x, y, yaw, ds, c;
    double cd, cv, cf; //costs
    bool empty; //true if no path available
};

struct Frenet2MPCInput{
    FrenetPath local_trajectory;
    std::vector<ObjectStruct> object_motion; //containing the obstacle position in the first index, then the prediction
};

#endif