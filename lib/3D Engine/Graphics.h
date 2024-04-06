#pragma once
#ifndef GRAPHICS_H
#define GRAPHICS_H
#include <FS.h>
#include <SPIFFS.h>
#include <Matrix.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <Utility.h>

using namespace std;

//extern U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI u8g2;
extern TFT_eSprite display;

class Graphics;
struct Plane;
struct Point;
struct Line;
struct  Triangle;
class Transform;
class Mesh;
class Camera;

struct Direction
{
    static Vec3 forward;
    static Vec3 back;
    static Vec3 right;
    static Vec3 left;
    static Vec3 up;
    static Vec3 down;
};
Vec3 Direction::forward = Vec3(0, 0, -1);
Vec3 Direction::back = Vec3(0, 0, 1);
Vec3 Direction::right = Vec3(1, 0, 0);
Vec3 Direction::left = Vec3(-1, 0, 0);
Vec3 Direction::up = Vec3(0, 1, 0);
Vec3 Direction::down = Vec3(0, -1, 0);

struct Color
{
    float r;
    float g;
    float b;
    float a;

    static Color black;
    static Color white;
    static Color gray;
    static Color red;
    static Color green;
    static Color blue;
    static Color pink;
    static Color purple;
    static Color yellow;
    static Color turquoise;
    static Color orange;

    Color()
    {
        this->r = 0.0;
        this->g = 0.0;
        this->b = 0.0;
        this->a = 1;
    }

    Color(float r, float g, float b, float a = 1.0)
    {
        this->r = r;
        this->g = g;
        this->b = b;
        this->a = a;
    }

    Color(Vec4 vec)
    {
        this->r = vec.x;
        this->g = vec.y;
        this->b = vec.z;
        this->a = vec.w;
    }

    static Color Random()
    {
        Color c = Color(Clamp(rand(), 0, 255), Clamp(rand(), 0, 255), Clamp(rand(), 0, 255));
        return c;
    }

    Color operator+(const Color& other)
    {
        Color color;
        color.r = Clamp(this->r + other.r, 0, 255);
        color.g = Clamp(this->g + other.g, 0, 255);
        color.b = Clamp(this->b + other.b, 0, 255);
        return color;
    }

    Color operator-(const Color& other)
    {
        Color color;
        color.r = Clamp(this->r - other.r, 0, 255);
        color.g = Clamp(this->g - other.g, 0, 255);
        color.b = Clamp(this->b - other.b, 0, 255);
        return color;
    }

    Color operator+(const Vec3& v3)
    {
        Color color;
        color.r = Clamp(this->r + v3.x, 0, 255);
        color.g = Clamp(this->g + v3.y, 0, 255);
        color.b = Clamp(this->b + v3.z, 0, 255);
        return color;
    }

    Color operator-(const Vec3& v3)
    {
        Color color;
        color.r = Clamp(this->r - v3.x, 0, 255);
        color.g = Clamp(this->g - v3.y, 0, 255);
        color.b = Clamp(this->b - v3.z, 0, 255);
        return color;
    }

    Color operator*(const float& scalar)
    {
        Color color;
        color.r = Clamp(this->r * scalar, 0, 255);
        color.g = Clamp(this->g * scalar, 0, 255);
        color.b = Clamp(this->b * scalar, 0, 255);
        return color;
    }

    Color& operator+=(const Color& other)
    {
        this->r = Clamp(this->r + other.r, 0, 255);
        this->g = Clamp(this->g + other.g, 0, 255);
        this->b = Clamp(this->b + other.b, 0, 255);
        return *this;
    }

    Color& operator+=(const Vec3& v3)
    {
        this->r = Clamp(this->r + v3.x, 0, 255);
        this->g = Clamp(this->g + v3.y, 0, 255);
        this->b = Clamp(this->b + v3.z, 0, 255);
        return *this;
    }

    Color& operator-=(const Color& other)
    {
        this->r = Clamp(this->r - other.r, 0, 255);
        this->g = Clamp(this->g - other.g, 0, 255);
        this->b = Clamp(this->b - other.b, 0, 255);
        return *this;
    }

    Color& operator-=(const Vec3& v3)
    {
        this->r = Clamp(this->r - v3.x, 0, 255);
        this->g = Clamp(this->g - v3.y, 0, 255);
        this->b = Clamp(this->b - v3.z, 0, 255);
        return *this;
    }

    Color& operator*=(const float scalar)
    {
        this->r = Clamp(this->r * scalar, 0, 255);
        this->g = Clamp(this->g * scalar, 0, 255);
        this->b = Clamp(this->b * scalar, 0, 255);
        return *this;
    }

    Color& operator/=(const float divisor)
    {
        if (divisor != 0.0)
        {
            this->r = Clamp(this->r / divisor, 0, 255);
            this->g = Clamp(this->g / divisor, 0, 255);
            this->b = Clamp(this->b / divisor, 0, 255);
            return *this;
        }
    }
    operator Vec3();
};

Color::operator Vec3()
{
    Vec3 vec(r, g, b);
    return vec;
}

Color Color::black = Color(0, 0, 0);
Color Color::white = Color(255, 255, 255);
Color Color::gray = Color(128, 128, 128);
Color Color::red = Color(255, 0, 0);
Color Color::green = Color(0, 255, 0);
Color Color::blue = Color(0, 0, 255);
Color Color::pink = Color(255, 0, 255);
Color Color::purple = Color(128, 0, 128);
Color Color::yellow = Color(255, 255, 0);
Color Color::turquoise = Color(0, 255, 255);
Color Color::orange = Color(255, 158, 0);

struct Material
{
    std::string name = "";
    Color color = Color::white;

    Material(std::string id = "", Color color = Color::white)
    {
        this->name = id;
        this->color = color;
    }
};

class Transform
{
public:
    Vec3 localScale = Vec3(1, 1, 1);
    Vec3 localPosition = Vec3(0, 0, 0);
    Matrix3x3 localRotation = Matrix3x3::identity;
    Transform* root = nullptr;
    Transform* parent = nullptr;

    Transform(const float& scale = 1, const Vec3& position = Vec3(0, 0, 0), const Vec3& rotationEuler = Vec3(0, 0, 0))
    {
        this->localScale.x = scale;
        this->localScale.y = scale;
        this->localScale.z = scale;
        this->localPosition = position;
        this->localRotation = YPR(rotationEuler.x, rotationEuler.y, rotationEuler.z);
        this->root = this;
    }

    Transform(const Vec3& scale, const Vec3& position = Vec3(0, 0, 0), const Matrix3x3& rotation = Matrix3x3::identity)
    {
        this->localScale = scale;
        this->localPosition = position;
        this->localRotation = rotation;
        this->root = this;
    }

    Vec3 Forward() { return Rotation() * Direction::forward; }
    Vec3 Back() { return Rotation() * Direction::back; }
    Vec3 Right() { return Rotation() * Direction::right; }
    Vec3 Left() { return Rotation() * Direction::left; }
    Vec3 Up() { return Rotation() * Direction::up; }
    Vec3 Down() { return Rotation() * Direction::down; }

    Matrix3x3 Rotation();

    Vec3 Position();

    Vec3 Scale();

    void SetParent(Transform* newParent, bool changeOfBasisTransition = true);

    IRAM_ATTR Matrix4x4 LocalScale4x4();

    IRAM_ATTR Matrix4x4 LocalScale4x4Inverse();

    IRAM_ATTR Matrix4x4 LocalRotation4x4();

    IRAM_ATTR Matrix4x4 LocalTranslation4x4();

    IRAM_ATTR Matrix4x4 LocalTranslation4x4Inverse();

    // 1:Scale, 2:Rotate, 3:Translate
    IRAM_ATTR Matrix4x4 TRS();

    // S^-1 * R^-1 * T^-1
    IRAM_ATTR Matrix4x4 TRSInverse();

    // 1:Rotate, 2:Translate
    IRAM_ATTR Matrix4x4 TR();

    // R^-1 * T^-1
    IRAM_ATTR Matrix4x4 TRInverse();
};

class Mesh : public Transform, public ManagedObjectPool<Mesh>
{
public:
    static int worldTriangleDrawCount;
    List<Vec3>* vertices;
    List<int>* indices;
    List<Triangle>* triangles;
    Color color = Color::white;
    bool ignoreLighting = false;
    bool forceWireFrame = false;
//Mesh(const Mesh& other) = delete;//disables copying
 

    Mesh(const float& scale = 1, const Vec3& position = Vec3(0, 0, 0), const Vec3& rotationEuler = Vec3(0, 0, 0))
        : Transform(scale, position, rotationEuler), ManagedObjectPool<Mesh>(this)
    {
        triangles = new List<Triangle>{};
        MapVertsToTriangles();
        SetColor(color);
    }
    
    Mesh(const Vec3& scale, const Vec3& position = Vec3(0, 0, 0), const Matrix3x3& rotation = Matrix3x3::identity)
        : Transform(scale, position, rotation), ManagedObjectPool<Mesh>(this)
    {
        triangles = new List<Triangle>{};
        MapVertsToTriangles();
        SetColor(color);
    }

    virtual ~Mesh()
    {
        delete vertices;
        delete indices;
        delete triangles;
    }

    bool SetVisibility(bool visible);
    void SetColor(Color&& c);

    void SetColor(Color& c);

    virtual IRAM_ATTR List<Triangle>* MapVertsToTriangles();

    //Convert to world coordinates
    List<Vec3> WorldVertices();

    void TransformTriangles();
};

List<Point>* pointBuffer = new List<Point>();
List<Line>* lineBuffer = new List<Line>();
List<Triangle>* triBuffer = new List<Triangle>();

static float worldScale = 1;
int screenWidth = 536;//128;
int screenHeight = 240;//64;

void ToScreenCoordinates(Vec3 &vec)
{
    static Vec2 origin = Vec2(0.5f*(float)screenWidth, 0.5f*(float)screenHeight);
    static float screenSpaceMatrix[4][4] = {
        {0.5f*screenWidth, 0, 0, origin.x},
        {0,-0.5f*screenHeight, 0, origin.y},
        {0, 0, 1.0f, 0},
        {0,0,0,0}
    };

    vec = screenSpaceMatrix*vec;
    /*
    static float screenXScalar = 0.5 * (float)screenWidth;
    static float screenYScalar = -0.5 * (float)screenHeight;
    vec.x *= screenXScalar;
    vec.y *= screenYScalar;
    vec += origin;*/
}

//-----------GRAPHICS---------------

class Graphics
{
private:
    static long hexColor;
public:
    static bool frustumCulling;
    static bool backFaceCulling;
    static bool invertNormals;
    static bool debugNormals;
    static bool debugVertices;
    static bool debugAxes;
    static bool debugBoxCollisions;
    static bool debugSphereCollisions;
    static bool debugPlaneCollisions;
    static bool perspective;
    static bool fillTriangles;
    static bool displayWireFrames;
    static bool lighting;
    static bool vfx;
    static bool matrixMode;

   static void SetDrawColor(Color& color)
    {
        //glColor3ub(color.x, color.y, color.z);
        SetDrawColor(color.r, color.g, color.b);
    }

    static void SetDrawColor(int r, int g, int b)
    {
        hexColor = ((r/255)*TFT_RED) + ((g/255) * TFT_GREEN) + ((b/255) * TFT_BLUE);
        //glColor3ub(r, g, b);
    }

    static void SetLineWidth(int width)
    {
        //glLineWidth(width);
    }

    static void SetPointSize(int size)
    {
       // glPointSize(size);
    }

    static void DrawPoint(Vec3 point)
    {
        // glBegin(GL_POINTS);
        // glVertex2f(point.x, point.y);
        // glEnd();
        ToScreenCoordinates(point);
        display.drawPixel(point.x, point.y, hexColor);
    }

    static void DrawLine(Vec3 from, Vec3 to)
    {
        
        // glBegin(GL_LINES);
        // glVertex2f(from.x, from.y);
        // glVertex2f(to.x, to.y);
        // glEnd();
        
        ToScreenCoordinates(from);
        ToScreenCoordinates(to);
        
        display.drawLine(from.x, from.y, to.x, to.y, hexColor);  
    }

    static void DrawTriangle(Vec3 p1, Vec3 p2, Vec3 p3)
    {
        // glBegin(GL_LINES);
        // glVertex2f(p1.x, p1.y);
        // glVertex2f(p2.x, p2.y);

        // glVertex2f(p2.x, p2.y);
        // glVertex2f(p3.x, p3.y);

        // glVertex2f(p3.x, p3.y);
        // glVertex2f(p1.x, p1.y);
        // glEnd();

        DrawLine(p1, p2);
        DrawLine(p2, p3);
        DrawLine(p3, p1);

    }

    static void DrawTriangleFilled(Vec3 p1, Vec3 p2, Vec3 p3)
    {
        // glBegin(GL_TRIANGLES);
        // glVertex2f(p1.x, p1.y);
        // glVertex2f(p2.x, p2.y);
        // glVertex2f(p3.x, p3.y);
        // glEnd();
        DrawLine(p1, p2);
        DrawLine(p2, p3);
        DrawLine(p3, p1);
        ToScreenCoordinates(p1);
        ToScreenCoordinates(p2);
        ToScreenCoordinates(p3);
        display.drawTriangle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, hexColor);
        display.fillTriangle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, hexColor);
    }
};
bool Graphics::frustumCulling = true;
bool Graphics::backFaceCulling = true;
bool Graphics::invertNormals = false;
bool Graphics::debugNormals = false;
bool Graphics::debugVertices = false;
bool Graphics::debugAxes = false;
bool Graphics::debugBoxCollisions = false;
bool Graphics::debugSphereCollisions = false;
bool Graphics::debugPlaneCollisions = false;
bool Graphics::perspective = true;
bool Graphics::fillTriangles = true;
bool Graphics::displayWireFrames = false;
bool Graphics::lighting = true;
bool Graphics::vfx = false;
bool Graphics::matrixMode = false;
long Graphics::hexColor = 0xFFFFFF;

Vec3 lightSource = .25f*Direction::up  + Direction::back * .5f;
float nearClippingPlane = -0.1f;
float farClippingPlane = -100000.0f;
float fieldOfViewDeg = 60;
float fov = ToRad(fieldOfViewDeg);
float aspect = (float)screenHeight / (float)screenWidth;

// Perspective Projection Matrix
float persp[4][4] = {
    {aspect * 1 / tan(fov / 2), 0, 0, 0},
    {0, 1 / tan(fov / 2), 0, 0},
    {0, 0, 1, 0},
    {0, 0, -1, 0}
};

// Perspective Projection Matrix
float weakPersp[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, -1, 0}
};

//Orthographic Projection Matrix
float ortho[4][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 1}
};

float undoAspect[4][4] = {
    {1 / aspect, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 0}
};

Matrix4x4 perspectiveProjectionMatrix = persp;
Matrix4x4 weakPerspectiveProjectionMatrix = weakPersp;
Matrix4x4 orthographicProjectionMatrix = ortho;
Matrix4x4 undoAspectRatio = undoAspect;

void FOV(int deg)
{
    fieldOfViewDeg = deg;
    fov = ToRad(deg);
    float newPerspectiveProjectionMatrix[4][4] = {
        {aspect * 1 / tan(fov / 2), 0, 0, 0},
        {0, 1 / tan(fov / 2), 0, 0},
        {0, 0, 1, 0},
        {0, 0, -1, 0}
    };

    perspectiveProjectionMatrix = newPerspectiveProjectionMatrix;
}

Matrix4x4 ProjectionMatrix()
{
    if (Graphics::perspective) {
        return perspectiveProjectionMatrix;
    }
    else {
        return orthographicProjectionMatrix;
    }
}

Matrix4x4 worldToViewMatrix;
Matrix4x4 projectionMatrix;

struct Point
{
    Vec3 position;
    Color color;
    int size;

    Point(Vec3 position, Color color = Color::white, int size = 2)
    {
        this->position = position;
        this->color = color;
        this->size = size;
    }

    void Draw()
    {
        Graphics::SetPointSize(size);
        Graphics::SetDrawColor(color);
        Graphics::DrawPoint(position);
    }

    static void AddPoint(Point point)
    {
        pointBuffer->emplace_back(point);
    }

    static void AddWorldPoint(Point point)
    {
        auto matrix = ProjectionMatrix() * worldToViewMatrix;
        point.position = matrix * point.position;
        pointBuffer->emplace_back(point);
    }
};

struct Line
{
    Vec3 from;
    Vec3 to;
    Color color;
    int width;

    Line(Vec3 from, Vec3 to, Color color = Color::white, int width = 2)
    {
        this->from = from;
        this->to = to;
        this->color = color;
        this->width = width;
    }

    void Draw()
    {
        Graphics::SetLineWidth(width);
        Graphics::SetDrawColor(color.r, color.g, color.b);
        Graphics::DrawLine(from, to);
    }

    static void AddLine(Line line)
    {
        lineBuffer->emplace_back(line);
    }

    static void AddWorldLine(Line line)
    {
        auto matrix = ProjectionMatrix() * worldToViewMatrix;
        line.from = matrix * line.from;
        line.to = matrix * line.to;
        lineBuffer->emplace_back(line);
    }
};

struct Triangle : Plane
{
    Vec4 centroid = Vec4();
    Color color = Color::white;
    bool forceWireFrame = false;
    Mesh* mesh = nullptr;
    
    Triangle() : Plane()
    {
        centroid = Vec4();
        color = Color::white;
        mesh = nullptr;
    }
    Triangle(Vec3 p1, Vec3 p2, Vec3 p3, Mesh* owner = nullptr) : Plane(p1, p2, p3)
    {
        color = Color::white;
        centroid = Centroid();
        mesh = owner;
    }

    Vec4 Centroid()
    {
        centroid = Vec4(
            (verts[0].x + verts[1].x + verts[2].x) / 3.0f,
            (verts[0].y + verts[1].y + verts[2].y) / 3.0f,
            (verts[0].z + verts[1].z + verts[2].z) / 3.0f,
            centroid.w
        );

        return centroid;
    }

    void Draw()
    {
        Vec4 p1 = verts[0];
        Vec4 p2 = verts[1];
        Vec4 p3 = verts[2];

        if (Graphics::fillTriangles == false)
        {
            Graphics::displayWireFrames = true;
        }

        //glColor3ub(255, 255, 255);
        if (Graphics::matrixMode)
        {
            Graphics::SetDrawColor(0, 255, 0);
        }

        if (Graphics::fillTriangles)
        {
            Graphics::SetDrawColor(color.r, color.g, color.b);
            Graphics::DrawTriangleFilled(p1, p2, p3);
        }

        bool drawWireFrame = Graphics::displayWireFrames || forceWireFrame || (mesh != nullptr ? mesh->forceWireFrame : false);
        if (drawWireFrame)
        {
            if (Graphics::fillTriangles)
            {
                float c = Clamp(1.0 / (0.000001f + (color.r + color.g + color.b) / 3), 0, 255);
                Graphics::SetDrawColor(c, c, c);
            }
            Graphics::DrawLine(p1, p2);
            Graphics::DrawLine(p2, p3);
            Graphics::DrawLine(p3, p1);
        }

        Graphics::SetDrawColor(255, 255, 255);
    }
};

//-------------------------------TRANSFORM---------------------------------------------

Vec3 ExtractPosition(const Matrix4x4& trs)
{
    return { trs.m[0][3], trs.m[1][3], trs.m[2][3] };
}

Vec3 ExtractScale(const Matrix4x4& trs) {
    Vec3 c1 = { trs.m[0][0], trs.m[1][0], trs.m[2][0] };
    Vec3 c2 = { trs.m[0][1], trs.m[1][1], trs.m[2][1] };
    Vec3 c3 = { trs.m[0][2], trs.m[1][2], trs.m[2][2] };
    return { c1.Magnitude(), c2.Magnitude(), c3.Magnitude() };
}

// Pass scale parameter if already known since finding the rotation matrix for a TRS matrix 
// requires the scale vector, which finding can be expensive.
Matrix3x3 ExtractRotation(const Matrix4x4 trs, Vec3* scale = NULL)
{
    static Vec3 v = Vec3::zero;
    Vec3 *s = &v;
    if (scale == NULL) {
        *s = ExtractScale(trs);
    }
    else {
        s = scale;
    }

    float rot[3][3] = {
        {trs.m[0][0] / s->x, trs.m[1][0] / s->y, trs.m[2][0] / s->z},
        {trs.m[0][1] / s->x, trs.m[1][1] / s->y, trs.m[2][1] / s->z},
        {trs.m[0][2] / s->x, trs.m[1][2] / s->y, trs.m[2][2] / s->z},
    };

    return rot;
}

struct TRSInfo
{
    Vec3 scale;
    Vec3 position;
    Matrix3x3 rotation;

    TRSInfo(const Vec3& s, const Vec3& pos, const Matrix3x3& rot)
    {
        scale = s;
        position = pos;
        rotation = rot;
    }
};

TRSInfo ExtractTRS(Matrix4x4& trs)
{
    Vec3 scale = ExtractScale(trs);
    return TRSInfo(scale, ExtractPosition(trs), ExtractRotation(trs, &scale));
}

Matrix3x3 Transform::Rotation()
{
    if (parent) {//TRS already checks for parent but checks again here because cheaper to return local position than creating the matrix
        return ExtractRotation(TRS());
    }
    return localRotation;
}

Vec3 Transform::Position()
{
    if (parent) {//TRS already checks for parent but checks again here because cheaper to return local position than creating the matrix
        return ExtractPosition(TRS());
    }

    return localPosition;
}

Vec3 Transform::Scale()
{
    if (parent) {
        return ExtractScale(TRS());
    }
    return localScale;
}

void Transform::SetParent(Transform* newParent, bool changeOfBasisTransition)
{
    static Matrix4x4 T;
    if (newParent == nullptr)
    {
        if (changeOfBasisTransition)
        {
            // Results in seemless unparenting. Assigns (possibly parented) global values to the local values. 
            // This way of unparenting will maintain the previous parented position and rotation but in the new reference frame.
            // Also nothing changes if never parented.
            //if already had a parent
            
            T = this->TR();
            
           // this->scale = Scale();// Vec3(1, 1, 1);
            float rot[3][3] = {
                {T.m[0][0], T.m[0][1], T.m[0][2]},
                {T.m[1][0], T.m[1][1], T.m[1][2]},
                {T.m[2][0], T.m[2][1], T.m[2][2]}
            };
            this->localRotation = rot;
            this->localPosition = Vec3(T.m[0][3], T.m[1][3], T.m[2][3]);
        }

        this->parent = NULL;
        this->root = this;
    }
    else if (newParent != this)
    {
        if (changeOfBasisTransition)
        {
            // If you immediatley parent a transform, everything is calculated relative to its immediate parent's reference frame. 
            // This would result in a transformation if the original coordinates didn't change. 
            // Instead what we want is a change of basis.
            T = newParent->TRInverse() * this->TR();
            
            float rot[3][3] = {
                {T.m[0][0], T.m[0][1], T.m[0][2]},
                {T.m[1][0], T.m[1][1], T.m[1][2]},
                {T.m[2][0], T.m[2][1], T.m[2][2]}
            };
            this->localScale = this->Scale();
            this->localRotation = rot;
            this->localPosition = Vec3(T.m[0][3], T.m[1][3], T.m[2][3]);
            
        }

        this->parent = newParent;
        this->root = newParent->root;
    }
}

IRAM_ATTR Matrix4x4 Transform::LocalScale4x4()
{
    float matrix[4][4] =
    {
        {this->localScale.x, 0, 0, 0},
        {0, this->localScale.y, 0, 0},
        {0, 0, this->localScale.z, 0},
        {0, 0, 0, 1}
    };

    return Matrix4x4(matrix);
}

IRAM_ATTR Matrix4x4 Transform::LocalScale4x4Inverse()
{
    float inverse[4][4] =
    {
        {1.0f / this->localScale.x, 0, 0, 0},
        {0, 1.0f / this->localScale.y, 0, 0},
        {0, 0, 1.0f / this->localScale.z, 0},
        {0, 0, 0, 1}
    };

    return Matrix4x4(inverse);
}

IRAM_ATTR Matrix4x4 Transform::LocalRotation4x4()
{
    float matrix[4][4] =
    {
        {this->localRotation.m[0][0], this->localRotation.m[0][1], this->localRotation.m[0][2], 0},
        {this->localRotation.m[1][0], this->localRotation.m[1][1], this->localRotation.m[1][2], 0},
        {this->localRotation.m[2][0], this->localRotation.m[2][1], this->localRotation.m[2][2], 0},
        {0, 0, 0, 1}
    };

    return Matrix4x4(matrix);
}

IRAM_ATTR Matrix4x4 Transform::LocalTranslation4x4()
{
    float matrix[4][4] =
    {
        {1, 0, 0, this->localPosition.x},
        {0, 1, 0, this->localPosition.y},
        {0, 0, 1, this->localPosition.z},
        {0, 0, 0, 1}
    };

    return Matrix4x4(matrix);
}

IRAM_ATTR Matrix4x4 Transform::LocalTranslation4x4Inverse()
{
    float matrix[4][4] =
    {
        {1, 0, 0, -this->localPosition.x},
        {0, 1, 0, -this->localPosition.y},
        {0, 0, 1, -this->localPosition.z},
        {0, 0, 0, 1}
    };

    return Matrix4x4(matrix);
}

// 1:Scale, 2:Rotate, 3:Translate
IRAM_ATTR Matrix4x4 Transform::TRS()
{
    float trs[4][4] =
    {
        {this->localRotation.m[0][0] * localScale.x, this->localRotation.m[0][1] * localScale.y, this->localRotation.m[0][2] * localScale.z, localPosition.x},
        {this->localRotation.m[1][0] * localScale.x, this->localRotation.m[1][1] * localScale.y, this->localRotation.m[1][2] * localScale.z, localPosition.y},
        {this->localRotation.m[2][0] * localScale.x, this->localRotation.m[2][1] * localScale.y, this->localRotation.m[2][2] * localScale.z, localPosition.z},
        {0,                                         0,                                  0,                      1}
    };

    if (parent) {
        return parent->TRS() * trs;
    }

    return trs;
}

// S^-1 * R^-1 * T^-1
IRAM_ATTR Matrix4x4 Transform::TRSInverse()
{
    if (parent) {
        return LocalScale4x4Inverse() * Matrix4x4::Transpose(LocalRotation4x4()) * LocalTranslation4x4Inverse() * parent->TRSInverse();
    }

    return LocalScale4x4Inverse() * Matrix4x4::Transpose(LocalRotation4x4()) * LocalTranslation4x4Inverse();
}

// 1:Rotate, 2:Translate
IRAM_ATTR Matrix4x4 Transform::TR()
{
    float tr[4][4] =
    {
        {this->localRotation.m[0][0], this->localRotation.m[0][1], this->localRotation.m[0][2], localPosition.x},
        {this->localRotation.m[1][0], this->localRotation.m[1][1], this->localRotation.m[1][2], localPosition.y},
        {this->localRotation.m[2][0], this->localRotation.m[2][1], this->localRotation.m[2][2], localPosition.z},
        {0,                                         0,                                  0,                      1}
    };
    if (parent) {
        return parent->TR() * tr;
    }

    return tr;
}

// R^-1 * T^-1
IRAM_ATTR Matrix4x4 Transform::TRInverse()
{
    if (parent) {
        return  Matrix4x4::Transpose(LocalRotation4x4()) * LocalTranslation4x4Inverse() * parent->TRInverse();
    }

    return Matrix4x4::Transpose(LocalRotation4x4()) * LocalTranslation4x4Inverse();
}

//-----------------------------CAMERA-------------------------------------------------
struct CameraSettings
{
    static bool outsiderViewPerspective;
    static bool displayReticle;
};
bool CameraSettings::outsiderViewPerspective = false;
bool CameraSettings::displayReticle = true;

class Camera : public Transform
{
public:
    static Camera* main;
    static Camera* projector;
    static List<Camera*> cameras;
    static int cameraCount;

    std::string name;

    Camera(const Vec3& position = Vec3(0, 0, 0), const Vec3& rotationEuler = Vec3(0, 0, 0))
        : Transform(1, position, rotationEuler)
    {
        cameras.emplace(cameras.begin() + cameraCount++, this);
        name = "Camera " + cameraCount;
    }
};
List<Camera*> Camera::cameras = List<Camera*>();
int Camera::cameraCount = 0;
Camera* Camera::projector = new Camera();
Camera* camera1 = new Camera();
Camera* camera2 = new Camera(Vec3(0, 50, 0), Vec3(-90 * PI / 180, 0, 0));
Camera* Camera::main = camera1;

//---------------------------------MESH---------------------------------------------

bool Mesh::SetVisibility(bool visible)
{
    if (visible) {
        ManagedObjectPool<Mesh>::addToPool(this);
        return true;
    }
    else {
        ManagedObjectPool<Mesh>::removeFromPool(this);
        return false;
    }
}
void Mesh::SetColor(Color&& c) {
    SetColor(c);
}

void Mesh::SetColor(Color& c)
{
    if (triangles)
    {
        for (int i = 0; i < triangles->size(); i++)
        {
            (*triangles)[i].color = c;
        }
    }
}

IRAM_ATTR List<Triangle>* Mesh::MapVertsToTriangles()
{
    if (vertices && indices)
    {
        int t = 0;
        for (size_t i = 0; i < indices->size(); i++)
        {
            int p1Index = (*indices)[i++];
            int p2Index = (*indices)[i++];
            int p3Index = (*indices)[i];

            (*triangles)[t].verts[0] = (*vertices)[p1Index];
            (*triangles)[t].verts[1] = (*vertices)[p2Index];
            (*triangles)[t].verts[2] = (*vertices)[p3Index];
            
            t++;
        }
    }

    return triangles;
}

//Convert to world coordinates
List<Vec3> Mesh::WorldVertices()
{
    List<Vec3> verts = *vertices;
    Matrix4x4 matrix = TRS();
    for (size_t i = 0; i < verts.size(); i++)
    {
        verts[i] = (Vec3)(matrix * ((Vec4)verts[i]));
    }

    return verts;
}

IRAM_ATTR void Mesh::TransformTriangles()
{
    // Scale/Distance ratio culling
    /* bool tooSmallToSee = scale.SqrMagnitude() / (position - Camera::main->position).SqrMagnitude() < 0.000000125;
    if (tooSmallToSee) {
        return;
    }*/

    Matrix4x4 modelToWorldMatrix = this->TRS();

    //Transform Triangles
    List<Triangle>* tris = MapVertsToTriangles();

    for (int i = 0; i < tris->size(); i++)
    {
        Triangle tri = (*tris)[i];
        tri.mesh = this;
        Triangle worldSpaceTri = tri;
        Triangle camSpaceTri = tri;
        Triangle projectedTri = tri;
        for (int j = 0; j < 3; j++)
        {
            // Homogeneous coords (x, y, z, w=1)
            Vec4 vert = tri.verts[j];

            // =================== WORLD SPACE ===================
            // Transform local coords to world-space coords.
            Vec4 worldPoint = modelToWorldMatrix * vert;
            worldSpaceTri.verts[j] = worldPoint;

            // ================ VIEW/CAM/EYE SPACE ================
            // Transform world coordinates to view coordinates.
            Vec4 cameraSpacePoint = worldToViewMatrix * worldPoint;
            camSpaceTri.verts[j] = cameraSpacePoint;

            // ================ SCREEN SPACE ==================
            // Project to screen space (image space)
            Vec4 projectedPoint = projectionMatrix * cameraSpacePoint;
            projectedTri.verts[j] = projectedPoint;
        };

        //------------------- Normal/Frustum Culling (view space)------------------------
        Vec3 p1_c = camSpaceTri.verts[0];
        Vec3 p2_c = camSpaceTri.verts[1];
        Vec3 p3_c = camSpaceTri.verts[2];
        camSpaceTri.Centroid();

        if (Graphics::frustumCulling)
        {
            bool tooCloseToCamera = (p1_c.z >= nearClippingPlane || p2_c.z >= nearClippingPlane || p3_c.z >= nearClippingPlane || camSpaceTri.centroid.z >= nearClippingPlane);
            if (tooCloseToCamera) {
                continue;
            }

            bool tooFarFromCamera = (p1_c.z <= farClippingPlane || p2_c.z <= farClippingPlane || p3_c.z <= farClippingPlane || camSpaceTri.centroid.z <= farClippingPlane);
            if (tooFarFromCamera) {
                continue;
            }
            /*
            bool behindCamera = DotProduct((Vec3)camSpaceTri.centroid, Direction::forward) <= 0.0;
            if (behindCamera) {
                continue; // Skip triangle if it's out of cam view.
            }*/

            Range xRange = ProjectVertsOntoAxis(projectedTri.verts, 3, Direction::right);
            Range yRange = ProjectVertsOntoAxis(projectedTri.verts, 3, Direction::up);
            if ((xRange.min > 1 || yRange.min > 1) || (xRange.max < -1 || yRange.max < -1)) {
                continue;
            }
        }

        // Calculate triangle suface Normal
        camSpaceTri.Normal();//camSpaceTri.normal = worldToViewMatrix * modelToWorldMatrix * Vec4(camSpaceTri.normal, 0);

        if (Graphics::invertNormals) {
            camSpaceTri.normal = ((Vec3)camSpaceTri.normal) * -1.0f;
        }

        // Back-face Culling - Checks if the triangles backside is facing the camera.
        // Condition makes this setting optional when drawing wireframes alone, but will force culling if triangles are filled.
        if (Graphics::backFaceCulling || Graphics::fillTriangles)
        {
            Vec3 posRelativeToCam = camSpaceTri.centroid;// Since camera is (0,0,0) in view space, the displacement vector from camera to centroid IS the centroid itself.
            bool faceInvisibleToCamera = DotProduct(posRelativeToCam, (Vec3)camSpaceTri.normal) >= 0;
            if (faceInvisibleToCamera) {
                continue;// Skip triangle if it's out of cam view or it's part of the other side of the mesh.
            }
        }

        //------------------------ Lighting (world space)------------------------

        if (Graphics::lighting && Graphics::fillTriangles)
        {
            if (!ignoreLighting)
            {
                float amountFacingLight = DotProduct((Vec3)worldSpaceTri.Normal(), lightSource);
                Color colorLit = projectedTri.color * Clamp(amountFacingLight, 0.15f, 1);
                projectedTri.color = colorLit;
            }
        }

        if (Graphics::vfx)
        {
            Vec3 screenLeftSide = Vec3(-1, 0, 0);
            Vec3 screenRightSide = Vec3(1, 0, 0);
            Range range = ProjectVertsOntoAxis(projectedTri.verts, 3, screenRightSide);
            bool leftHalfScreenX = range.min > 0 && range.max > 0;

            if (leftHalfScreenX) {
                projectedTri.color = Color(0, 0, 255);// std::cout << "Inside" << std::endl;
            }
            else {
                projectedTri.color = Color::red;
            }
        }
        // ---------- Debugging -----------
        if (Graphics::debugNormals)
        {
            //---------Draw point at centroid and a line from centroid to normal (view space & projected space)-----------
            Vec2 centroidToNormal_p = projectionMatrix * ((Vec3)camSpaceTri.centroid + camSpaceTri.normal);
            Vec2 centroid_p = projectionMatrix * camSpaceTri.centroid;
            Point::AddPoint(Point(centroid_p));
            Line::AddLine(Line(centroid_p, centroidToNormal_p));
        }

        projectedTri.centroid = projectionMatrix * camSpaceTri.centroid;

        // Nested Projection or Double Projection
        if (CameraSettings::outsiderViewPerspective)
        {
            Matrix4x4 nestedProjectionMatrix = weakPerspectiveProjectionMatrix * Camera::projector->TRInverse();

            for (size_t k = 0; k < 3; k++)
            {
                projectedTri.verts[k] = nestedProjectionMatrix * projectedTri.verts[k];
            }
        }

        //Add projected tri
        triBuffer->emplace_back(projectedTri);
    }
}
//List<Mesh*> Mesh::objects = List<Mesh*>(1000);
//int Mesh::meshCount = 0;
int Mesh::worldTriangleDrawCount = 0;

//------------------------------------CUBE MESH------------------------------------------
class CubeMesh : public Mesh
{
public:
    CubeMesh(const float& scale = 1, const Vec3& position = Vec3(0, 0, 0), const Vec3& rotationEuler = Vec3(0, 0, 0))
        :Mesh(scale, position, rotationEuler)
    {
        // Local Space (Object Space)
        this->vertices = new List<Vec3>({//new Vec3[8] {
            //south
            Vec3(-0.5f, -0.5f, 0.5f),
            Vec3(-0.5f, 0.5f, 0.5f),
            Vec3(0.5f, 0.5f, 0.5f),
            Vec3(0.5f, -0.5f, 0.5f),
            //north
            Vec3(-0.5f, -0.5f, -0.5f),
            Vec3(-0.5f, 0.5f, -0.5f),
            Vec3(0.5f, 0.5f, -0.5f),
            Vec3(0.5f, -0.5f, -0.5f)
            });

        this->indices = new List<int>{
            //South
            0, 1, 2,
            0, 2, 3,
            //North
            7, 6, 5,
            7, 5, 4,
            //Right
            3, 2, 6,
            3, 6, 7,
            //Left
            4, 5, 1,
            4, 1, 0,
            //Top
            1, 5, 6,
            1, 6, 2,
            //Bottom
            3, 7, 4,
            3, 4, 0
        };

        triangles = new List<Triangle>(this->indices->size() / 3);
    }
};

//---------------------------------PLANE---------------------------------------------
class PlaneMesh : public Mesh
{
public:
    PlaneMesh(const float& scale = 1, const Vec3& position = Vec3(0, 0, 0), const Vec3& rotationEuler = Vec3(0, 0, 0))
        :Mesh(scale, position, rotationEuler)
    {
        this->vertices = new List<Vec3>{
                Vec3(-0.5f, 0, 0.5f),
                Vec3(-0.5f, 0, -0.5f),
                Vec3(0.5f, 0, -0.5f),
                Vec3(0.5f, 0, 0.5f)
        };

        this->triangles->emplace_back(Triangle((*vertices)[0], (*vertices)[1], (*vertices)[2]));
        this->triangles->emplace_back(Triangle((*vertices)[0], (*vertices)[2], (*vertices)[3]));
    }
};


//------------------------------HELPER FUNCTIONS------------------------------------------------

Mesh* LoadMeshFromOBJFile(string objFileName) 
{


//   if (!SPIFFS.begin(true))
//   {
//     Serial.println("Failed while mounting SPIFFS.");
//     return;
//   }
  
//---------------------------------

    static string filePath = "/";

    string mtlFileName = "";

    // ----------- Read object file -------------
    List<string> strings;
    string line;
    File objFile = SPIFFS.open((filePath+objFileName).c_str(), "r");
    if (objFile)//if (objFile.is_open())
    {
        while (objFile.available()) 
        {
            // 1st. Gets the next line.
            // 2nd. Seperates each word from that line then stores each word into the strings array.
            String l = objFile.readStringUntil('\n');
            line = l.c_str();
            Serial.println(l);
            string word;
            stringstream ss(line);
            while (getline(ss, word, ' ')) 
            {
                if (word == "mtllib") {
                    getline(ss, word, ' ');
                    mtlFileName = word;
                    Serial.println(String(mtlFileName.c_str()));
                }
                else {
                    strings.emplace_back(word);
                }
            }
        }
    }
    objFile.close();

    // -----------------Construct new mesh-------------------
    Mesh* mesh = new Mesh();
    List<Vec3>* verts = new List<Vec3>();
    List<int>* indices = new List<int>();
    List<Triangle>* triangles = new List<Triangle>(indices->size() / 3);
    Material material;
    for (size_t i = 0; i < strings.size(); i++)
    {
        // v = vertex
        // f = face
        string objFileSubString = strings[i];
        // if .obj encounters "usemtl" then the next string will be material id.
        // if (objFileSubString == "usemtl")
        // {
        //     // Try opening Material file to see if it exists
        //     File mtlFile = SPIFFS.open((filePath + mtlFileName).c_str());
        //     // check if using material before looking for material key words
        //     while (mtlFile) 
        //     {
        //         string mtlID = strings[++i];
        //         bool mtlIDFound = false;
        //         //materials->emplace_back(Material(materialID));
        //         // search .mtl for material properties under the the current materialID
        //         if (!mtlIDFound)// && mtlFile) 
        //         {
        //             // 1st. Gets the next line.
        //             // 2nd. Seperates each word from that line then stores each word into the strings array.
        //             String l = mtlFile.readStringUntil('\n');
        //             line = l.c_str();
        //             Serial.println(l);
        //             Serial.println("!mtlIDFound");
        //             string word;
        //             stringstream ss(line);
        //             while (!mtlIDFound && getline(ss, word, ' '))
        //             {
        //                 Serial.println("!mtlIDFound && getline(ss, word, ' ')");
        //                 if (word == "newmtl")
        //                 { 
        //                     getline(ss, word, ' ');
        //                     if (mtlID == word) {
        //                         material.name = word;
        //                     }
        //                 }
        //                 else if (mtlID == material.name && word == "Kd") {
        //                     getline(ss, word, ' ');
        //                     float r = stof(word);
        //                     getline(ss, word, ' ');
        //                     float g = stof(word);
        //                     getline(ss, word, ' ');
        //                     float b = stof(word);
        //                     material.color = Color(255*r, 255*g, 255*b);

        //                     mtlIDFound = true;
        //                 }
        //             }
        //         }
        //         mtlFile.close();
        //     }
        // }
        if (false){}
        else if (objFileSubString == "v") {
            float x = stof(strings[++i]);
            float y = stof(strings[++i]);
            float z = stof(strings[++i]);
            verts->emplace_back(Vec3(x, y, z));
        }
        //f means the next 3 strings will be the indices for mapping vertices
        else if (objFileSubString == "f") {
            int p3Index = stof(strings[++i]) - 1;
            int p2Index = stof(strings[++i]) - 1;
            int p1Index = stof(strings[++i]) - 1;

            indices->emplace_back(p1Index);
            indices->emplace_back(p2Index);
            indices->emplace_back(p3Index);

            Triangle tri = Triangle((*verts)[p1Index], (*verts)[p2Index], (*verts)[p3Index]);
            tri.color = material.color;
            triangles->emplace_back(tri);
        }
    }

    mesh->vertices = verts;
    mesh->indices = indices;
    mesh->triangles = triangles;

    return mesh;
}
/*
Mesh* LoadMeshFromOBJFile(string objFileName) 
{
    static string filePath = "./Objects/";

    string mtlFileName = "";
    std::ifstream mtlFile;

    // ----------- Read object file -------------
    List<string> strings;
    string line;
    std::ifstream objFile;
    objFile.open(filePath+objFileName);
    if (objFile.is_open())
    {
        while (objFile) {
            // 1st. Gets the next line.
            // 2nd. Seperates each word from that line then stores each word into the strings array.
            getline(objFile, line);
            string word;
            stringstream ss(line);
            while (getline(ss, word, ' ')) 
            {
                if (word == "mtllib") {
                    getline(ss, word, ' ');
                    mtlFileName = word;
                }
                else {
                    strings.emplace_back(word);
                }
            }
        }
    }
    objFile.close();

    // -----------------Construct new mesh-------------------
    Mesh* mesh = new Mesh();
    List<Vec3>* verts = new List<Vec3>();
    List<int>* indices = new List<int>();
    List<Triangle>* triangles = new List<Triangle>(indices->size() / 3);
    Material material;
    for (size_t i = 0; i < strings.size(); i++)
    {
        // v = vertex
        // f = face
        string objFileSubString = strings[i];
        
        // if .obj encounters "usemtl" then the next string will be material id.
        if (objFileSubString == "usemtl")
        {
            // Try opening Material file to see if it exists
            mtlFile.open(filePath + mtlFileName);
            // check if using material before looking for material key words
            if (mtlFile.is_open()) 
            {
                string mtlID = strings[++i];
                bool mtlIDFound = false;
                //materials->emplace_back(Material(materialID));
                // search .mtl for material properties under the the current materialID
                while (!mtlIDFound && mtlFile) 
                {
                    // 1st. Gets the next line.
                    // 2nd. Seperates each word from that line then stores each word into the strings array.
                    getline(mtlFile, line);
                    string word;
                    stringstream ss(line);
                    while (!mtlIDFound && getline(ss, word, ' '))
                    {    
                        if (word == "newmtl")
                        { 
                            getline(ss, word, ' ');
                            if (mtlID == word) {
                                material.name = word;
                            }
                        }
                        else if (mtlID == material.name && word == "Kd") {
                            getline(ss, word, ' ');
                            float r = stof(word);
                            getline(ss, word, ' ');
                            float g = stof(word);
                            getline(ss, word, ' ');
                            float b = stof(word);
                            material.color = Color(255*r, 255*g, 255*b);

                            mtlIDFound = true;
                        }
                    }
                }
                mtlFile.close();
            }
        }

        else if (objFileSubString == "v") {
            float x = stof(strings[++i]);
            float y = stof(strings[++i]);
            float z = stof(strings[++i]);
            verts->emplace_back(Vec3(x, y, z));
        }
        //f means the next 3 strings will be the indices for mapping vertices
        else if (objFileSubString == "f") {
            int p3Index = stof(strings[++i]) - 1;
            int p2Index = stof(strings[++i]) - 1;
            int p1Index = stof(strings[++i]) - 1;

            indices->emplace_back(p1Index);
            indices->emplace_back(p2Index);
            indices->emplace_back(p3Index);

            Triangle tri = Triangle((*verts)[p1Index], (*verts)[p2Index], (*verts)[p3Index]);
            tri.color = material.color;
            triangles->emplace_back(tri);
        }
    }
    //mtlFile.close();

    mesh->vertices = verts;
    mesh->indices = indices;
    mesh->triangles = triangles;

    return mesh;
}
*/

IRAM_ATTR void Draw()
{
    // Camera TRInverse = (TR)^-1 = R^-1*T^-1 = M = Mcw = World to Camera coords. 
    // This matrix isn't used on the camera itself, but we record the reverse transformations of the camera going from world space back to local camera space.
    // Every point then in world space multiplied by this matrix will end up in a position relative to the camera's point of view when it was in world space. 
    // The camera could now be considered as the origin (0,0,0) with the zero rotation (identity matrix). 
    worldToViewMatrix = Camera::main->TRInverse();
    projectionMatrix = ProjectionMatrix();
    Matrix4x4 vpMatrix = projectionMatrix * worldToViewMatrix;

    // ---------- Transform -----------
    for (int i = 0; i < Mesh::count; i++)
    {
        if (Graphics::frustumCulling)
        {/*
            // Scale/Distance ratio culling
            float sqrDist = (Mesh::objects[i]->root->localPosition - Camera::main->Position()).SqrMagnitude();
            if (sqrDist != 0.0f)
            {
                bool meshTooSmallToSee = Mesh::objects[i]->root->localScale.SqrMagnitude() / sqrDist < 0.0000000000001;
                if (meshTooSmallToSee) {
                    continue;
                }
            }
            
            bool meshBehindCamera = DotProduct((Mesh::objects[i]->Position() - Camera::main->position), Camera::main->Forward()) <= 0.0;
            if (meshBehindCamera) {
                continue;
            }
*/
            if (Mesh::objects[i]->vertices)
            {
                List<Vec3> verts = *(Mesh::objects[i]->vertices);
                for (size_t j = 0; j < verts.size(); j++)
                {
                    verts[j] = vpMatrix * Mesh::objects[i]->TRS() * verts[j];
                }
                Range xRange = ProjectVertsOntoAxis(verts.data(), verts.size(), Direction::right);
                Range yRange = ProjectVertsOntoAxis(verts.data(), verts.size(), Direction::up);
                if ( ((xRange.min > 1.0 && xRange.max > 1.0) || (xRange.min < -1.0 && xRange.max < -1.0)) || ((yRange.min > 1.0 && yRange.max > 1.0) || (yRange.min < -1.0 && yRange.max < -1.0)) )
                {
                    continue;
                }
            }

            // ---------- Debug -----------
            if (Graphics::debugAxes)
            {
                if (DotProduct(Mesh::objects[i]->Position() - Camera::main->Position(), Camera::main->Forward()) > 0)
                {
                    Matrix4x4 mvp = vpMatrix * Mesh::objects[i]->TRS();

                    Vec2 center_p = mvp * Vec4(0, 0, 0, 1);
                    Vec2 xAxis_p = mvp * Vec4(0.5, 0, 0, 1); 
                    Vec2 yAxis_p = mvp * Vec4(0, 0.5, 0, 1);
                    Vec2 zAxis_p = mvp * Vec4(0, 0, 0.5, 1);
                    Vec2 forward_p = mvp * (Direction::forward);

                    Point::AddPoint(Point(center_p, Color::red, 4));
                    Line::AddLine(Line(center_p, xAxis_p, Color::red));
                    Line::AddLine(Line(center_p, yAxis_p, Color::yellow));
                    Line::AddLine(Line(center_p, zAxis_p, Color::blue));
                    Line::AddLine(Line(center_p, mvp * (Direction::forward), Color::turquoise, 3));
                }
            }
        }

        Mesh::objects[i]->TransformTriangles();
    }

    Mesh::worldTriangleDrawCount = triBuffer->size();

    // ---------- Sort (Painter's algorithm) -----------
    sort(triBuffer->begin(), triBuffer->end(), [](const Triangle& triA, const Triangle& triB) -> bool {
        return triA.centroid.w > triB.centroid.w;
        });

    // ---------- Draw -----------
    for (int i = 0; i < triBuffer->size(); i++)
    {
        (*triBuffer)[i].Draw();
    }

    for (size_t i = 0; i < lineBuffer->size(); i++)
    {
        (*lineBuffer)[i].Draw();
    }

    for (size_t i = 0; i < pointBuffer->size(); i++)
    {
        (*pointBuffer)[i].Draw();
    }

    pointBuffer->clear();
    lineBuffer->clear();
    triBuffer->clear();
}
#endif