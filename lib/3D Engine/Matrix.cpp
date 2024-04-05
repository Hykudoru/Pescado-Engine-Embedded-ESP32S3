#include <Matrix.h>

float Matrix3x3::identity[3][3] {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};

float Matrix4x4::identity[4][4] {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
};


// A * B
Matrix3x3 operator*(const Matrix3x3& matrixA, const Matrix3x3& matrixB)
{
    Matrix3x3 matrix = Matrix3x3::Multiply(matrixA.m, matrixB.m);
    return matrix;
}
// M * p
Vector3<float> operator*(const Matrix3x3& matrix, const Vector3<float>& colVec)
{
    Vector3<float> result = Vector3<float>();
    Vector3<float> row1 = Vector3<float>(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2]);
    Vector3<float> row2 = Vector3<float>(matrix.m[1][0], matrix.m[1][1], matrix.m[1][2]);
    Vector3<float> row3 = Vector3<float>(matrix.m[2][0], matrix.m[2][1], matrix.m[2][2]);
    result.x = DotProduct(row1, colVec);
    result.y = DotProduct(row2, colVec);
    result.z = DotProduct(row3, colVec);

    return result;
}

// Roll-Pitch-Yaw x-y'-z''(intrinsic rotation) or z-y-x (extrinsic rotation)
Matrix3x3 RPY(float roll, float pitch, float yaw)
{
    //Matrix3x3 rotation = Matrix3x3::Multiply(Matrix3x3::Multiply(Matrix3x3::RotX(roll).m, Matrix3x3::RotY(pitch).m).m, Matrix3x3::RotZ(yaw).m);//Multiply(rotation.m, RotZ(PI/2.0).m);
    Matrix3x3 rotation = (Matrix3x3::RotX(roll) * Matrix3x3::RotY(pitch)) * Matrix3x3::RotZ(yaw);//Multiply(rotation.m, RotZ(PI/2.0).m);
    return rotation;
}

// Yaw-Pitch-Roll z-y'-x''(intrinsic rotation) or x-y-z (extrinsic rotation)
Matrix3x3 YPR(float roll, float pitch, float yaw)
{
    //Matrix3x3 rotation = Matrix3x3::Multiply(Matrix3x3::Multiply(Matrix3x3::RotZ(yaw).m, Matrix3x3::RotY(pitch).m).m, Matrix3x3::RotX(roll).m);//Multiply(rotation.m, RotZ(PI/2.0).m);
    Matrix3x3 rotation = (Matrix3x3::RotZ(yaw) * Matrix3x3::RotY(pitch)) * Matrix3x3::RotX(roll);//Multiply(rotation.m, RotZ(PI/2.0).m);

    return rotation;
}


// A * B
Matrix4x4 operator*(const Matrix4x4& matrixA, const Matrix4x4& matrixB)
{
    Matrix4x4 matrix = Matrix4x4::Multiply(matrixA, matrixB);
    return matrix;
}

// M * p
Vector4<float> operator*(const Matrix4x4& matrix, const Vector4<float>& colVec)
{
    Vector4<float> result = Vector4<float>();
    Vector4<float> row1 = Vector4<float>(matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3]);
    Vector4<float> row2 = Vector4<float>(matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3]);
    Vector4<float> row3 = Vector4<float>(matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3]);
    Vector4<float> row4 = Vector4<float>(matrix.m[3][0], matrix.m[3][1], matrix.m[3][2], matrix.m[3][3]);
    result.x = DotProduct(row1, colVec);
    result.y = DotProduct(row2, colVec);
    result.z = DotProduct(row3, colVec);
    result.w = DotProduct(row4, colVec);

    if (result.w != 0.0000)
    {
        result.x /= result.w;
        result.y /= result.w;
        result.z /= result.w;
    }

    return result;
}

//Matrix * floatMatrix
Matrix4x4 operator*(const Matrix4x4& matrixA, float matrixB[4][4])
{
    return Matrix4x4::Multiply(matrixA, matrixB);
}
//floatMatrix * Matrix
Matrix4x4 operator*(float matrixA[4][4], const Matrix4x4& matrixB)
{
    return Matrix4x4::Multiply(matrixA, matrixB);
}