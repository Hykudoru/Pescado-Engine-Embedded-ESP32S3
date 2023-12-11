#pragma once
#ifndef UTILITY_H
#define UTILITY_H
#include <Vector.h>
#define List std::vector
extern bool DEBUGGING;

template <typename T>
class ManagedObjectPool
{
public:
    static List<T*> objects;
    static int count;

    ManagedObjectPool(T* obj)
    {
        //objects.emplace(objects.begin() + count++, obj);
        ManagedObjectPool::objects.emplace_back(obj);
        count++;
    }

    ~ManagedObjectPool()
    {
        for (size_t i = 0; i < ManagedObjectPool::objects.size(); i++)
        {
            if (this == ManagedObjectPool::objects[i]) {
                ManagedObjectPool::objects.erase(ManagedObjectPool::objects.begin() + i);
                break;
            }
        }
        count--;
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

    Plane(Vec3 pointOnPlane, Vec3 normal)
    {
        this->normal = normal;
        float D = DotProduct(normal, pointOnPlane);
        float xIntercept = D / normal.x;
        float yIntercept = D / normal.y;
        float zIntercept = D / normal.z;
        verts[0] = Vec3(xIntercept, 0, 0);
        verts[1] = Vec3(0, yIntercept, 0);
        verts[2] = Vec3(0, 0, zIntercept);
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

Range ProjectVertsOntoAxis(const Vec3 verts[], const int& count, Vec3& axis)
{
    float min = 0;
    float max = 0;
    for (size_t k = 0; k < count; k++)
    {
        float dist = DotProduct(verts[k], axis);
        if (k == 0)
        {
            min = dist;
            max = dist;
        }
        else if (dist < min) {
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

    float t = DotProduct(n, pointPlane - lineStart) / DotProduct(n, v);
    Vec3 pIntersect = lineStart + (v * t);

    if (DotProduct(pIntersect - lineStart, v) > 0.0)
    {
        *pointIntersecting = pIntersect;
        return true;
    }

    return false;
}

bool PointInsideTriangle(const Vec3& p, const Vec3 triPoints[3])
{
    Vec3 A = triPoints[0];
    Vec3 B = triPoints[1];
    Vec3 C = triPoints[2];
    float w1 = (((p.x - A.x) * (C.y - A.y)) - ((p.y - A.y) * (C.x - A.x))) / (((B.x - A.x) * (C.y - A.y)) - ((B.y - A.y) * (C.x - A.x)));
    float w2 = ((p.y - A.y) - (w1 * (B.y - A.y))) / (C.y - A.y);

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
#endif