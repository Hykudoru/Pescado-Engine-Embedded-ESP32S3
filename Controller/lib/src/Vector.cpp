#include "Vector.h"
#include <cmath>

template <typename T>
T DotProduct(Vector3<T> a, Vector3<T> b)
{
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

// Remember order matters. A X B != B X A
// Calculates a unit vector orthogonal/perpendicular to both A and B vectors
template <typename T>
Vector3<T> CrossProduct(Vector3<T> a, Vector3<T> b)
{
    /* Matrix
     |  i | j | k  |
     | a.x a.y a.z |
     | b.x b.y b.z |
    */
    
    Vector3<T> u;
    u.x = (a.y*b.z - a.z*b.y); //i 
    u.y = -(a.x*b.z) + (a.z*b.x);// -(a.x*b.z - a.z*b.x), //-j
    u.z = (a.x*b.y - a.y*b.x); //k
    /* To check is orthogonal: bool isPerpendicular = dotProduct(u, a) == 0 && dotProduct(u, b) == 0;
    */
    return u; 
}