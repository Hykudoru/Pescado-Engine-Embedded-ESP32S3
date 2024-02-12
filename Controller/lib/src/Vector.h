#ifndef VECTOR_H
#define VECTOR_H

template <typename T>
class Vector3;template <typename T>

T DotProduct(Vector3<T> a, Vector3<T> b);

template <typename T>
Vector3<T> CrossProduct(Vector3<T> a, Vector3<T> b);

//-------------------------------
// --------- Vector2 ------------
//-------------------------------
template <typename T>
class Vector2
{
    public:
    T x;
    T y;

    Vector2();
    Vector2(T xVal, T yVal);
    Vector2(T xy[]);

    T SqrMagnitude()
    {
        return x*x + y*y;
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
        return Vector2<T>((float)x/length, (float)y/length);
    }

    Vector2 operator+(const Vector2& other)
    {
        Vector2 vectorSum;
        vectorSum.x = this->x + other.x;
        vectorSum.y = this->y + other.y;
        return vectorSum;
    }

    Vector2 operator-(const Vector2& other)
    {
        Vector2 vectorDiff;
        vectorDiff.x = this->x - other.x;
        vectorDiff.y = this->y - other.y;
        return vectorDiff;
    }

    Vector2 operator*(const T& scalar)
    {
        Vector2 scaledVector;
        scaledVector.x = this->x * scalar;
        scaledVector.y = this->y * scalar;
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

    Vector2& operator/=(const T scalar)
    {
        if (scalar < 0.00001 && scalar > -0.00001) {
            scalar = 0.00001;
        }
        
        this->x /= scalar;
        this->y /= scalar;
        return *this;
    }

    operator Vector3<T>();
};

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

//-------------------------------
// --------- Vector3 ------------
//-------------------------------
template <typename T>
class Vector3
{
    public:
    T x;
    T y;
    T z;

    Vector3();
    Vector3(T xVal, T yVal, T zVal);
    Vector3(T xyz[]);

    T SqrMagnitude()
    {
        return x*x + y*y + z*z;
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
            z = 0;
        }
        else {
            x /= length;
            y /= length;
            z /= length;
        }
    }
    
    Vector3<T> Normalized()
    {
        float length = this->Magnitude();
        return Vector3<T>((float)x/length, (float)y/length, (float)z/length);
    }

    Vector3 operator+(const Vector3& other)
    {
        Vector3 vectorSum;
        vectorSum.x = this->x + other.x;
        vectorSum.y = this->y + other.y;
        vectorSum.z = this->z + other.z;
        return vectorSum;
    }

    Vector3 operator-(const Vector3& other)
    {
        Vector3 vectorDiff;
        vectorDiff.x = this->x - other.x;
        vectorDiff.y = this->y - other.y;
        vectorDiff.z = this->z - other.z;
        return vectorDiff;
    }

    Vector3 operator*(const T& scalar)
    {
        Vector3 scaledVector;
        scaledVector.x = this->x * scalar;
        scaledVector.y = this->y * scalar;
        scaledVector.z = this->z * scalar;
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

    Vector3& operator/=(const T scalar)
    {
        if (scalar < 0.00001 && scalar > -0.00001) {
            scalar = 0.00001;
        }
        
        this->x /= scalar;
        this->y /= scalar;
        this->z /= scalar;
        return *this;
    }

    operator Vector2<T>();
};

//================================
template <typename T>
Vector2<T>::operator Vector3<T>()
{
        Vector3<T> vec3;
        vec3.x = x;
        vec3.y = y;
        vec3.z = 0;
        return vec3;
}

template <typename T>
Vector3<T>::operator Vector2<T>()
{
        Vector2<T> vec2;
        vec2.x = x;
        vec2.y = y;
        return vec2;
}
//===============================

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
#endif

/*
using Vector3f = Vector3<float>;
float vec3[];
Vector3f m[3][3] = {
    {Vector3f()},
    {Vector3f()},
    {Vector3f()}
};
float identity[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1},
};
 
space coordinates xyz
body coordinates XYZ-++
*/