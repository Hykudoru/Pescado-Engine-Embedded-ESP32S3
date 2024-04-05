#pragma once
#ifndef UTILITY_H
#define UTILITY_H
#include <Vector.h>
#include <math.h>
#include <vector>
#define List std::vector
extern bool DEBUGGING;
//const float PI = 3.14159265359f;
const float TAO = 2.0 * PI;
typedef void (*Callback)();

template <typename T>
class ManagedObjectPool
{
public:
    static List<T*> objects;
    static int count;

    ManagedObjectPool(T* obj)
    {
        if (obj)
        {
            ManagedObjectPool::objects.emplace_back(obj);
            count = ManagedObjectPool::objects.size();//count++;
        }
    }
    
    virtual ~ManagedObjectPool()
    {
        for (size_t i = 0; i < ManagedObjectPool<T>::objects.size(); i++)
        {
            if (this == ManagedObjectPool<T>::objects[i]) {
                ManagedObjectPool<T>::objects.erase(ManagedObjectPool<T>::objects.begin() + i);
                count = ManagedObjectPool<T>::objects.size();
                break;
            }
        }
    }
protected:
    static void addToPool(T* obj)
    {
        for (size_t i = 0; i < ManagedObjectPool<T>::objects.size(); i++)
        {
            if (obj == ManagedObjectPool<T>::objects[i]) {
                return;
            }
        }
        
        ManagedObjectPool<T>::objects.emplace_back(obj);
        count = ManagedObjectPool<T>::objects.size();//count++;
    }

    static void removeFromPool(T* obj)
    {
        for (size_t i = 0; i < ManagedObjectPool<T>::objects.size(); i++)
        {
            if (obj == ManagedObjectPool<T>::objects[i]) {
                ManagedObjectPool<T>::objects.erase(ManagedObjectPool<T>::objects.begin() + i);
                count = ManagedObjectPool<T>::objects.size();
                return;
            }
        }
    }
    

   
};
template <typename T>
List<T*> ManagedObjectPool<T>::objects = List<T*>();
template <typename T>
int ManagedObjectPool<T>::count = 0;

struct Plane
{
    Vec3 verts[3];
    Vec3 normal;

    Plane(){}

    Plane(Vec3 p1, Vec3 p2, Vec3 p3)
    {
        verts[0] = p1;
        verts[1] = p2;
        verts[2] = p3;

        Normal();
    }

    //NEEDS TESTING
    Plane(Vec3 pointOnPlane, Vec3 normal)
    {
        this->normal = normal;
        verts[0] = pointOnPlane;
        float D = DotProduct(normal, pointOnPlane);
        float x = pointOnPlane.x;
        float y = pointOnPlane.y;
        float z = pointOnPlane.z;
        
        if (normal.x == 0.0 && normal.y == 0.0 && normal.z == 1.0)
        {
            verts[1] = pointOnPlane + Vec3(1, 1, 0);
            verts[2] = pointOnPlane + Vec3(-1, 1, 0);
            return;
        }
        else if (normal.x == 0.0 && normal.y == 1.0 && normal.z == 0.0)
        {
            verts[1] = pointOnPlane + Vec3(1, 0, 1);
            verts[2] = pointOnPlane + Vec3(-1, 0, 1);
            return;
        }
        else if (normal.x == 1.0 && normal.y == 0.0 && normal.z == 0.0)
        {
            verts[1] = pointOnPlane + Vec3(0, 1, 1);
            verts[2] = pointOnPlane + Vec3(0, -1, 1);
            return;
        }

        if (normal.x != 0.0) {
            x = D / normal.x;
            verts[0] = Vec3(x, 0, 0);
        }
        if (normal.y != 0.0) {
            y = D / normal.y;
            verts[0] = Vec3(0, y, 0);
        }
        if (normal.x != 0.0) {
            z = D / normal.z;
            verts[0] = Vec3(0, 0, z);
        }
    }

    Vec3 Normal()
    {
        // Calculate triangle suface Normal
        Vec3 a = verts[2] - verts[0];
        Vec3 b = verts[1] - verts[0];
        normal = (CrossProduct(a, b)).Normalized();

        return normal;
    }
};

struct Range
{
    float min;
    float max;

    Range(float minimum, float maximum)
    {
        min = minimum;
        max = maximum;
    }
};

float Clamp(float value, float min, float max)
{
    if (value < min) {
        value = min;
    }
    else if (value > max) {
        value = max;
    }
    return value;
}

Vec3 RandomVector()
{
    Vec3 randomVector = Vec3(rand(), rand(), rand());

    return randomVector;
}

Vec3 RandomDirection()
{
    Vec3 vec = RandomVector();
    vec.Normalize();

    return vec;
}

float ToDeg(float rad) {
    return rad * 180.0 / PI;
}

float ToRad(float deg) {
    return deg * PI / 180.0;
}

Vec3 Reflect(Vec3 v, Vec3 n)
{
    return v + (n*2 * DotProduct(v*-1, n));
}

Vec3 ProjectOnPlane(Vec3 v, Vec3 n)
{
    Vec3 b = n * DotProduct(n, v);
    return v - b;
}

Range ProjectVertsOntoAxis(Vec3* verts, const int& count, Vec3& axis)
{
    float dist = DotProduct(verts[0], axis);
    float min = dist;
    float max = dist;
    for (size_t k = 1; k < count; k++)
    {
        dist = DotProduct(verts[k], axis);
        if (dist < min) {
            min = dist;
        }
        else if (dist > max) {
            max = dist;
        }
    }

    return Range(min, max);
}

Vec3 ClosestPoint(List<Vec3>& verts, Vec3& pointComparing, float* closestDistance = NULL)
{
    Vec3 closestPoint;
    float closestDist;
    for (size_t i = 0; i < verts.size(); i++)
    {
        if (i == 0) {
            closestPoint = verts[i];
            closestDist = (verts[i]-pointComparing).SqrMagnitude();
            continue;
        }

        Vec3 point = verts[i] - pointComparing;
        float dist = point.SqrMagnitude();
        if (dist < closestDist)
        {
            closestDist = dist;
            closestPoint = verts[i];
        }
    }

    if (closestDistance) {
        *closestDistance = closestDist;
    }

    return closestPoint;
}

bool LinePlaneIntersecting(Vec3& lineStart, Vec3& lineEnd, Plane& plane, Vec3* pointIntersecting)
{
    // Plane: N_x(x-x0) + N_y(y-y0) + N_z(z-z0) = 0
    // Point on plane: x0,y0,z0
    // Point on line: x,y,z
    // Parametric Line: P = P0 + Vt ---> lineEnd = lineStart + (lineEnd-lineStart)t
    // 1st. Paramiterize line like this ---> x = (P0_x + V_x*t), y =, z = ... 
    // 2nd. Plugin x,y,z it into plane equation and solve for t.
    // 3rd. Use t to find the point intersecting both the line and plane.
    Vec3 n = plane.Normal();
    Vec3 pointPlane = plane.verts[0];
    Vec3 v = lineEnd - lineStart;
    float divisor = DotProduct(n, v);
    if (divisor != 0.0) 
    {
        float t = DotProduct(n, pointPlane - lineStart) / divisor;    
        //if (t >= 0.0)
        {
            *pointIntersecting = lineStart + (v*t);
            return true;
        }
    }
    return false;
} 

bool PointInsideTriangle(const Vec3& p, const Vec3 triPoints[3])
{
    Vec3 A = triPoints[0];
    Vec3 B = triPoints[1];
    Vec3 C = triPoints[2];
    float divisor = (((B.x - A.x) * (C.y - A.y)) - ((B.y - A.y) * (C.x - A.x)));
    float divisor2 = (C.y - A.y);
    if (divisor == 0.0 || divisor2 == 0.0)
    {
        return false;
    }
    float w1 = (((p.x - A.x) * (C.y - A.y)) - ((p.y - A.y) * (C.x - A.x))) / divisor;
    float w2 = ((p.y - A.y) - (w1 * (B.y - A.y))) / divisor2;

    return ((w1 >= 0.0 && w2 >= 0.0) && (w1 + w2) <= 1.0);
}

//Needs Testing
void PlanesIntersecting(Vec3& normal1, Vec3& p1, Vec3& normal2, Vec3& p2)
{
    float D1 = DotProduct(normal1, p1);
    float D2 = DotProduct(normal2, p2);
    Vec3 v = CrossProduct(normal1, normal2);
    float y = (D2 * normal1.x - D1 * normal2.x) / (-normal2.x * normal1.y + normal2.y);
    float x = (D1 - normal1.y * y) / normal1.x;

    float t = 0;
    Vec3 line = Vec3(x, y, 0) + v * t;
}

Vec3 ClosestPointOnSphere(Vec3 center, float radius, Vec3 somePoint)
{
    Vec3 dir = (somePoint - center).Normalized();
    return center + dir * radius;
}
#endif