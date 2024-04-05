#pragma once
#include<string>
#include <math.h>
#ifndef VECTOR_H
#define VECTOR_H

template <typename T>
class Vector3;

template <typename T>
class Vector4;

template <typename T>
T DotProduct(const Vector3<T>& a, const Vector3<T>& b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

template <typename T>
T DotProduct(const Vector4<T>& a, const Vector4<T>& b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
}

// Remember order matters. A X B != B X A
// Calculates a unit vector orthogonal/perpendicular to both A and B vectors
template <typename T>
Vector3<T> CrossProduct(const Vector3<T>& a, const Vector3<T>& b)
{
    /* Matrix
     |  i | j | k  |
     | a.x a.y a.z |
     | b.x b.y b.z |
    */

    Vector3<T> u;
    u.x = (a.y * b.z - a.z * b.y); //i 
    u.y = -(a.x * b.z) + (a.z * b.x);// -(a.x*b.z - a.z*b.x), //-j
    u.z = (a.x * b.y - a.y * b.x); //k
    /* To check is orthogonal: bool isPerpendicular = dotProduct(u, a) == 0 && dotProduct(u, b) == 0;
    */
    return u;
}

// ----------------------- Vector2 -------------------------
template <typename T>
class Vector2
{
public:
    T x;
    T y;
    static Vector2<T> zero;

    Vector2();
    Vector2(T xVal, T yVal);
    Vector2(T xy[]);

    T SqrMagnitude()
    {
        return x * x + y * y;
    }

    T Magnitude()
    {
        return sqrt(this->SqrMagnitude());
    }

    //--------- Normalize() vs Normalized() ---------
    // Normalize() modifies the original
    // Normalized() returns a normalized vector without modifying the original.

    void Normalize()
    {
        float length = this->Magnitude();
        if (length < 0.00001) {
            x = 0;
            y = 0;
        }
        else {
            x /= length;
            y /= length;
        }
    }

    Vector2<T> Normalized()
    {
        float length = this->Magnitude();
        return Vector2<T>((float)x / length, (float)y / length);
    }

    Vector2 operator+(const Vector2& other)
    {
        Vector2 vectorSum(this->x + other.x, this->y + other.y);
        return vectorSum;
    }

    Vector2 operator-(const Vector2& other)
    {
        Vector2 vectorDiff(this->x - other.x, this->y - other.y);
        return vectorDiff;
    }
    Vector2 operator-()//allows for -vec syntax
    {
        Vector2 vectorNeg(-this->x, -this->y);
        return vectorNeg;
    }

    Vector2 operator*(const T& scalar)
    {
        Vector2 scaledVector(this->x * scalar, this->y * scalar);
        return scaledVector;
    }

    Vector2& operator+=(const Vector2& other)
    {
        this->x += other.x;
        this->y += other.y;
        return *this;
    }

    Vector2& operator-=(const Vector2& other)
    {
        this->x -= other.x;
        this->y -= other.y;
        return *this;
    }

    Vector2& operator*=(const T scalar)
    {
        this->x *= scalar;
        this->y *= scalar;
        return *this;
    }

    Vector2& operator/=(const T divisor)
    {
        if (divisor != 0.0) {
            this->x /= divisor;
            this->y /= divisor;
        }
        return *this;
    }


    friend bool operator==(const Vector2& vecA, const Vector2& vecB) { return (vecA.x == vecB.x && vecA.y == vecB.y); }
    friend bool operator!=(const Vector2& vecA, const Vector2& vecB) { return (vecA.x != vecB.x || vecA.y != vecB.y); }

    operator Vector3<T>();
};

// Constructors
template <typename T>
Vector2<T>::Vector2()
{
    x = 0.0;
    y = 0.0;
}
template <typename T>
Vector2<T>::Vector2(T xVal, T yVal)
{
    x = xVal;
    y = yVal;
}

template <typename T>
Vector2<T>::Vector2(T xy[])
{
    x = xy[0];
    y = xy[1];
}
template <typename T>
Vector2<T> Vector2<T>::zero = { 0, 0 };

// ----------------------- Vector3 -------------------------
template <typename T>
class Vector3
{
public:
   // T v[3];
    T x;
    T y;
    T z;
    int size = 3;
    static Vector3<T> zero;
    static Vector3<T> one;

    Vector3();
    Vector3(T xVal, T yVal, T zVal);
    Vector3(T xyz[]);
    /*
    string ToString() 
    {
        string str = "";
        str += "(" + x + ", " + y + ", " + z + ")";
        str += "\n\r";
        return str;
    }*/

    T SqrMagnitude()
    {
        return x * x + y * y + z * z;
    }

    T Magnitude()
    {
        return sqrt(this->SqrMagnitude());
    }

    //--------- Normalize() vs Normalized() ---------
    // Normalize() modifies the original
    // Normalized() returns a normalized vector without modifying the original.

    void Normalize()
    {
        float length = this->Magnitude();
        if (length != 0.0) 
        {
            x /= length;
            y /= length;
            z /= length;
        }
    }

    Vector3<T> Normalized()
    {
        float length = this->Magnitude();
        Vector3<T> norm;
        if (length != 0.0)
        {
            norm.x = x/length;
            norm.y = y/length;
            norm.z = z/length;
        }
        
        return norm;
    }

    Vector3 operator+(const Vector3& other)
    {
        Vector3 vectorSum(this->x + other.x, this->y + other.y, this->z + other.z);
        return vectorSum;
    }

    Vector3 operator-(const Vector3& other)
    {
        Vector3 vectorDiff(this->x - other.x, this->y - other.y, this->z - other.z);
        return vectorDiff;
    }
    Vector3 operator-()//allows for -vec syntax
    {
        Vector3 vectorNeg(-this->x, -this->y, -this->z);
        return vectorNeg;
    }

    Vector3 operator*(const T& scalar)
    {
        Vector3 scaledVector(this->x * scalar, this->y * scalar, this->z * scalar);
        return scaledVector;
    }

    Vector3& operator+=(const Vector3& other)
    {
        this->x += other.x;
        this->y += other.y;
        this->z += other.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& other)
    {
        this->x -= other.x;
        this->y -= other.y;
        this->z -= other.z;
        return *this;
    }

    Vector3& operator*=(const T scalar)
    {
        this->x *= scalar;
        this->y *= scalar;
        this->z *= scalar;
        return *this;
    }

    Vector3& operator/=(const T divisor)
    {
        if (divisor != 0.0)
        {
            this->x /= divisor;
            this->y /= divisor;
            this->z /= divisor;
            return *this;
        }
    }

    friend bool operator==(const Vector3& vecA, const Vector3& vecB) { return (vecA.x == vecB.x && vecA.y == vecB.y && vecA.z == vecB.z); }
    friend bool operator!=(const Vector3& vecA, const Vector3& vecB) { return (vecA.x != vecB.x || vecA.y != vecB.y || vecA.z == vecB.z); }

    operator Vector2<T>();
    operator Vector4<T>();
};
template <typename T>
Vector3<T> Vector3<T>::zero = { 0, 0, 0 };
template <typename T>
Vector3<T> Vector3<T>::one = { 1, 1, 1 };

// Constructors
template <typename T>
Vector3<T>::Vector3()
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
}
template <typename T>
Vector3<T>::Vector3(T xVal, T yVal, T zVal)
{
    x = xVal;
    y = yVal;
    z = zVal;
}

template <typename T>
Vector3<T>::Vector3(T xyz[])
{
    x = xyz[0];
    y = xyz[1];
    z = xyz[2];
}

// ----------------------- Vector4 -------------------------
template <typename T>
class Vector4
{
public:
    T x = 0.0;
    T y = 0.0;
    T z = 0.0;
    T w = 1.0;

    Vector4();
    Vector4(T x, T y, T z, T w = 1.0);
    Vector4(T xyzw[]);
    Vector4(Vector3<T> vec3, T w);

    operator Vector3<T>();
    operator Vector2<T>();
};
// Constructors
template <typename T>
Vector4<T>::Vector4()
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    w = 1.0;
}
template <typename T>
Vector4<T>::Vector4(T x, T y, T z, T w)
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;
}

template <typename T>
Vector4<T>::Vector4(T xyzw[])
{
    x = xyzw[0];
    y = xyzw[1];
    z = xyzw[2];
    w = xyzw[3];
}
template <typename T>
Vector4<T>::Vector4(Vector3<T> vec3, T w)
{
    this->x = vec3.x;
    this->y = vec3.y;
    this->z = vec3.z;
    this->w = w;
}
//================================
// Casting
template <typename T>
Vector2<T>::operator Vector3<T>()
{
    Vector3<T> vec3(x, y, 0);
    return vec3;
}
// Casting
template <typename T>
Vector3<T>::operator Vector2<T>()
{
    Vector2<T> vec2(x, y);
    return vec2;
}
template <typename T>
Vector3<T>::operator Vector4<T>()
{
    Vector4<T> vec4(x, y, z, 1);
    return vec4;
}

template <typename T>
Vector4<T>::operator Vector3<T>()
{
    Vector3<T> vec3(x, y, z);
    return vec3;
}
template <typename T>
Vector4<T>::operator Vector2<T>()
{
    Vector2<T> vec2(x, y);

    return vec2;
}
#define Vec2 Vector2<float>
#define Vec3 Vector3<float>
#define Vec4 Vector4<float>
#define Euler Vec3;


template <typename T>//Syntax: scalar * vector
Vector2<T> operator*(const float& scalar, const Vector2<T>& vector)
{
    Vector2<T> scaledVector = vector;
    scaledVector.x = scaledVector.x * scalar;
    scaledVector.y = scaledVector.y * scalar;
    return scaledVector;
}

template <typename T> //Syntax: vector * scalar
Vector2<T> operator*(const Vector2<T>& vector, const float& scalar)
{
    Vector2<T> scaledVector = vector;
    scaledVector.x = scaledVector.x * scalar;
    scaledVector.y = scaledVector.y * scalar;
    return scaledVector;
}

template <typename T> //Syntax: scalar * vector
Vector3<T> operator*(const float& scalar, const Vector3<T>& vector)
{
    Vector3<T> scaledVector = vector;
    scaledVector.x = scaledVector.x * scalar;
    scaledVector.y = scaledVector.y * scalar;
    scaledVector.z = scaledVector.z * scalar;
    return scaledVector;
}

template <typename T> //Syntax: vector * scalar
Vector3<T> operator*(const Vector3<T>& vector, const float& scalar)
{
    Vector3<T> scaledVector = vector;
    scaledVector.x = scaledVector.x * scalar;
    scaledVector.y = scaledVector.y * scalar;
    scaledVector.z = scaledVector.z * scalar;
    return scaledVector;
}
#endif