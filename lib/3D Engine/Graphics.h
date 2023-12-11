#pragma once
#ifndef GRAPHICS_H
#define GRAPHICS_H
#include <Matrix.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <Utility.h>
#include <U8g2lib.h>

using namespace std;

//extern U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI u8g2;
extern TFT_eSprite display;

#define Color Vector3<float>
class Mesh;
struct Plane;
struct Point;
struct Line;
struct  Triangle;
struct RGB;

List<Point>* pointBuffer = new List<Point>();
List<Line>* lineBuffer = new List<Line>();
List<Triangle>* triBuffer = new List<Triangle>();

static float worldScale = 1;
int screenWidth = 536;//128;
int screenHeight = 240;//64;

void ToScreenCoordinates(Vec3 &vec)
{
    static Vec2 origin = Vec2(0.5*(float)screenWidth, 0.5*(float)screenHeight);
    static float screenXScalar = 0.5 * (float)screenWidth;
    static float screenYScalar = -0.5 * (float)screenHeight;
    vec.x *= screenXScalar;
    vec.y *= screenYScalar;
    vec += origin;
}


class DrawAPI
{
public:
    static void SetDrawColor(Color color)
    {
        //glColor3ub(color.x, color.y, color.z);
    }

    static void SetDrawColor(float r, float g, float b)
    {
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
        display.drawPixel(point.x, point.y, TFT_GREEN);
    }

    static void DrawLine(Vec3 from, Vec3 to)
    {
        
        // glBegin(GL_LINES);
        // glVertex2f(from.x, from.y);
        // glVertex2f(to.x, to.y);
        // glEnd();
        
        ToScreenCoordinates(from);
        ToScreenCoordinates(to);
        display.drawLine(from.x, from.y, to.x, to.y, TFT_GREEN);  
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
    }
};

//-----------GRAPHICS---------------
struct GraphicSettings
{
    static bool frustumCulling;
    static bool backFaceCulling;
    static bool invertNormals;
    static bool debugNormals;
    static bool debugVertices;
    static bool debugAxes;
    static bool perspective;
    static bool fillTriangles;
    static bool displayWireFrames;
    static bool lighting;
    static bool vfx;
    static bool matrixMode;
};
bool GraphicSettings::frustumCulling = true;
bool GraphicSettings::backFaceCulling = true;
bool GraphicSettings::invertNormals = false;
bool GraphicSettings::debugNormals = false;
bool GraphicSettings::debugVertices = false;
bool GraphicSettings::debugAxes = false;
bool GraphicSettings::perspective = true;
bool GraphicSettings::fillTriangles = true;
bool GraphicSettings::displayWireFrames = false;
bool GraphicSettings::lighting = true;
bool GraphicSettings::vfx = false;
bool GraphicSettings::matrixMode = false;

struct CameraSettings
{
    static bool outsiderViewPerspective;
};
bool CameraSettings::outsiderViewPerspective = false;

#define Color Vector3<float>
struct RGB : Vector3<float>
{
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
};
Color RGB::black = Color(0, 0, 0);
Color RGB::white = Color(255, 255, 255);
Color RGB::gray = Color(128, 128, 128);
Color RGB::red = Color(255, 0, 0);
Color RGB::green = Color(0, 255, 0);
Color RGB::blue = Color(0, 0, 255);
Color RGB::pink = Color(255, 0, 255);
Color RGB::purple = Color(128, 0, 128);
Color RGB::yellow = Color(255, 255, 0);
Color RGB::turquoise = Color(0, 255, 255);
Color RGB::orange = Color(255, 158, 0);

struct Material
{
    string name = "";
    Color color = RGB::white;

    Material(string id = "", Color color = RGB::white)
    {
        this->name = id;
        this->color = color;
    }
};

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

float nearClippingPlane = -0.1;
float farClippingPlane = -100000.0;
float fieldOfViewDeg = 60;
float fov = ToRad(fieldOfViewDeg);
float aspect = (float)screenHeight / (float)screenWidth;

// Perspective Projection Matrix
float persp[4][4] = {
    {aspect * 1/tan(fov/2), 0, 0, 0},
    {0, 1/tan(fov/2), 0, 0},
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
        {aspect * 1/tan(fov/2), 0, 0, 0},
        {0, 1/tan(fov/2), 0, 0},
        {0, 0, 1, 0},
        {0, 0, -1, 0}
    };

    perspectiveProjectionMatrix = newPerspectiveProjectionMatrix;
}

Matrix4x4 ProjectionMatrix()
{
    if (GraphicSettings::perspective) {
        return perspectiveProjectionMatrix;
    }
    else {
        return orthographicProjectionMatrix;
    }
}

struct Point
{
    Vec3 point;
    Color color;
    int size;

    Point(Vec3 point, Color color = RGB::white, int size = 2)
    {
        this->point = point;
        this->color = color;
        this->size = size;
    }

    void Draw()
    {
        DrawAPI::SetPointSize(size);
        DrawAPI::SetDrawColor(color.x, color.y, color.z);
        DrawAPI::DrawPoint(point);
    }
};

struct Line
{
    Vec3 from;
    Vec3 to;
    Color color;
    int width;

    Line(Vec3 from, Vec3 to, Color color = RGB::white, int width = 2)
    {
        this->from = from;
        this->to = to;
        this->color = color;
        this->width= width;
    }

    void Draw()
    {
        DrawAPI::SetLineWidth(width);
        DrawAPI::SetDrawColor(color.x, color.y, color.z);
        DrawAPI::DrawLine(from, to);
    }
};

struct Triangle : Plane
{
    Vec4 centroid = Vec4();
    Color color = RGB::white;
    Mesh* mesh = nullptr;

    Triangle() : Plane()
    {
        centroid = Vec4();
        color = RGB::white;
        mesh = nullptr;
    }
    Triangle(Vec3 p1, Vec3 p2, Vec3 p3) : Plane(p1, p2, p3)
    {
        color = RGB::white;
        centroid = Centroid();
        mesh = nullptr;
    }

    Vec4 Centroid()
    {
        centroid = Vec4(
            (verts[0].x + verts[1].x + verts[2].x) / 3.0,
            (verts[0].y + verts[1].y + verts[2].y) / 3.0,
            (verts[0].z + verts[1].z + verts[2].z) / 3.0,
            centroid.w
        );

        return centroid;
    }

    void Draw()
    {
        Vec4 p1 = verts[0];
        Vec4 p2 = verts[1];
        Vec4 p3 = verts[2];

        if (GraphicSettings::fillTriangles == false) 
        {
            GraphicSettings::displayWireFrames = true;
        }

        //glColor3ub(255, 255, 255);
        if (GraphicSettings::matrixMode)
        {
            DrawAPI::SetDrawColor(0, 255, 0);
        }

        if (GraphicSettings::fillTriangles)
        {
            DrawAPI::SetDrawColor(color.x, color.y, color.z);
            DrawAPI::DrawTriangleFilled(p1, p2, p3);
        }

        if (GraphicSettings::displayWireFrames)
        {
            if (GraphicSettings::fillTriangles)
            {
                float c = Clamp(1.0 / (0.000001 + (color.x + color.y + color.z) / 3), 0, 255);
                DrawAPI::SetDrawColor(c, c, c);
            }
            DrawAPI::DrawLine(p1, p2);
            DrawAPI::DrawLine(p2, p3);
            DrawAPI::DrawLine(p3, p1);
        }
        
        DrawAPI::SetDrawColor(255, 255, 255);
    }
};

//-------------------------------TRANSFORM---------------------------------------------
class Transform
{
public:
    Vec3 scale = Vec3(1, 1, 1);
    Vec3 position = Vec3(0, 0, 0);
    Matrix3x3 rotation = Identity3x3;
    Transform* root = nullptr;
    Transform* parent = nullptr;
    static bool parentHierarchyDefault;

    Vec3 Forward() { return this->rotation * Direction::forward; }
    Vec3 Back() { return this->rotation * Direction::back; }
    Vec3 Right() { return this->rotation * Direction::right; }
    Vec3 Left() { return this->rotation * Direction::left; }
    Vec3 Up() { return this->rotation * Direction::up; }
    Vec3 Down() { return this->rotation * Direction::down; }

    Transform(float scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0), Transform* parent = nullptr)
    {
        this->scale.x = scale;
        this->scale.y = scale;
        this->scale.z = scale;
        this->position = position;
        this->rotation = YPR(rotationEuler.x, rotationEuler.y, rotationEuler.z);
        this->root = this;
        SetParent(parent);
    }

    void SetParent(Transform* parent)
    {
        if (parent)
        {
            this->parent = parent;
            this->root = this->parent->root;
        }
        else {
            this->root = this;
        }
    }

    Matrix4x4 ScaleMatrix4x4()
    {
        float matrix[4][4] =
{
            {this->scale.x, 0, 0, 0},
            {0, this->scale.y, 0, 0},
            {0, 0, this->scale.z, 0},
            {0, 0, 0, 1}
        };

        return Matrix4x4(matrix);
    }

    Matrix4x4 RotationMatrix4x4() 
    {
        float matrix[4][4] = 
        {
            {this->rotation.m[0][0], this->rotation.m[0][1], this->rotation.m[0][2], 0},
            {this->rotation.m[1][0], this->rotation.m[1][1], this->rotation.m[1][2], 0},
            {this->rotation.m[2][0], this->rotation.m[2][1], this->rotation.m[2][2], 0},
            {0, 0, 0, 1}
        };

        return Matrix4x4(matrix);
    }

    Matrix4x4 TranslationMatrix4x4() 
    {
        float matrix[4][4] = 
        {
            {1, 0, 0, this->position.x},
            {0, 1, 0, this->position.y},
            {0, 0, 1, this->position.z},
            {0, 0, 0, 1}
        };

        return Matrix4x4(matrix);
    }

    Matrix4x4 TranslationMatrix4x4Inverse() 
    {
        float matrix[4][4] = 
        {
            {1, 0, 0, -this->position.x},
            {0, 1, 0, -this->position.y},
            {0, 0, 1, -this->position.z},
            {0, 0, 0, 1}
        };

        return Matrix4x4(matrix);
    }

    // 1:Scale, 2:Rotate, 3:Translate
    Matrix4x4 TRS() 
    {
        float trs[4][4] =
        {
            {this->rotation.m[0][0] * scale.x, this->rotation.m[0][1] * scale.y, this->rotation.m[0][2] * scale.z, position.x},
            {this->rotation.m[1][0] * scale.x, this->rotation.m[1][1] * scale.y, this->rotation.m[1][2] * scale.z, position.y},
            {this->rotation.m[2][0] * scale.x, this->rotation.m[2][1] * scale.y, this->rotation.m[2][2] * scale.z, position.z},
            {0,                                         0,                                  0,                      1}
        };

        if (parent) {
            if (Transform::parentHierarchyDefault) {
                return parent->TRS() * trs;
            } else { 
                return parent->TRS() * ScaleMatrix4x4() * RotationMatrix4x4() * TranslationMatrix4x4();
            }
        }

        return trs;
    }

    // 1:Rotate, 2:Translate
    Matrix4x4 TR() 
    {
        if (parent) {
            if (parentHierarchyDefault) {
                return parent->TR() * TranslationMatrix4x4() * RotationMatrix4x4();
            } else {
                return parent->TR() * RotationMatrix4x4() * TranslationMatrix4x4();
            }
        }

        return TranslationMatrix4x4() * RotationMatrix4x4();
    }

    // R^-1T^-1
    Matrix4x4 TRInverse() 
    {
        if (parent) 
        {
            return  Matrix4x4::Transpose(RotationMatrix4x4()) * TranslationMatrix4x4Inverse() * parent->TRInverse();
        }

        return Matrix4x4::Transpose(RotationMatrix4x4()) * TranslationMatrix4x4Inverse();
    }
};
bool Transform::parentHierarchyDefault = true;

//-----------------------------CAMERA-------------------------------------------------
class Camera : public Transform
{
public:
    static Camera* main;
    static Camera* projector;
    static List<Camera*> cameras;
    static int cameraCount;

    std::string name;

    Camera(Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
    : Transform(1, position, rotationEuler)
    {
        cameras.emplace(cameras.begin() + cameraCount++, this);
        name = std::string("Camera " + cameraCount);
    }
};
List<Camera*> Camera::cameras = List<Camera*>();
int Camera::cameraCount = 0;
Camera* Camera::projector = new Camera();
Camera* camera1 = new Camera();
Camera* camera2 = new Camera(Vec3(0, 50, 0), Vec3(-90 * PI / 180, 0, 0));
Camera* Camera::main = camera1;

//---------------------------------MESH---------------------------------------------
Matrix4x4 worldToViewMatrix;
Matrix4x4 projectionMatrix;
class Mesh : public Transform, public ManagedObjectPool<Mesh>
{
public:
    static int worldTriangleDrawCount;
    List<Vec3>* vertices;
    List<int>* indices;
    List<Triangle>* triangles;
    Color* color;

    Mesh(float scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
    : Transform(scale, position, rotationEuler), ManagedObjectPool(this)
    {
        MapVertsToTriangles();
    }

    ~Mesh()
    {
        delete vertices;
        delete indices;
        delete triangles;
    }
    
    virtual List<Triangle>* MapVertsToTriangles() 
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
    List<Vec3> WorldVertices()
    {
        List<Vec3> verts = *vertices;
        Matrix4x4 matrix = TRS();
        for (size_t i = 0; i < verts.size(); i++)
        {
            verts[i] = (Vec3)(matrix * ((Vec4)verts[i]));
        }

        return verts;
    }

    void TransformTriangles() 
    {
        // Scale/Distance ratio culling
        bool tooSmallToSee = scale.SqrMagnitude() / (position - Camera::main->position).SqrMagnitude() < 0.000000125;
        if (tooSmallToSee) {
            //return;
        }
        
        Matrix4x4 modelToWorldMatrix = this->TRS();

        //Transform Triangles
        List<Triangle>* tris = MapVertsToTriangles();
        for (int i = 0; i < tris->size(); i++)
        {
            Triangle tri = (*tris)[i];
            tri.mesh = this;
            if (color) {
                tri.color = *color;
            }
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

            if (GraphicSettings::frustumCulling)
            {
                bool tooCloseToCamera = (p1_c.z >= nearClippingPlane || p2_c.z >= nearClippingPlane || p3_c.z >= nearClippingPlane || camSpaceTri.centroid.z >= nearClippingPlane);
                if (tooCloseToCamera) {
                    continue;
                }

                bool tooFarFromCamera = (p1_c.z <= farClippingPlane || p2_c.z <= farClippingPlane || p3_c.z <= farClippingPlane || camSpaceTri.centroid.z <= farClippingPlane);
                if (tooFarFromCamera){
                    continue;
                }

                bool behindCamera = DotProduct((Vec3)camSpaceTri.centroid, Direction::forward) <= 0.0;
                if (behindCamera) {
                    continue; // Skip triangle if it's out of cam view.
                }

                // Range range = ProjectVertsOntoAxis(projectedTri.verts, 3, Direction::left);
                // if ((range.min > 1 && range.max > 1) || (range.min < -2 && range.max < -2)) {
                //     continue;
                // }
                // range = ProjectVertsOntoAxis(projectedTri.verts, 3, Direction::down);
                // if ((range.min > 1 && range.max > 1) || (range.min < -2 && range.max < -2)) {
                //     continue;
                // }
            }

            // Calculate triangle suface Normal
            camSpaceTri.Normal();//camSpaceTri.normal = worldToViewMatrix * modelToWorldMatrix * Vec4(camSpaceTri.normal, 0);

            if (GraphicSettings::invertNormals) {
                camSpaceTri.normal = ((Vec3) camSpaceTri.normal) * -1.0;
            }

            // Back-face Culling - Checks if the triangles backside is facing the camera.
            // Condition makes this setting optional when drawing wireframes alone, but will force culling if triangles are filled.
            if (GraphicSettings::backFaceCulling || GraphicSettings::fillTriangles)
            {
                Vec3 posRelativeToCam = camSpaceTri.centroid;// Since camera is (0,0,0) in view space, the displacement vector from camera to centroid IS the centroid itself.
                bool faceInvisibleToCamera = DotProduct(posRelativeToCam, (Vec3) camSpaceTri.normal) >= 0;
                if (faceInvisibleToCamera) {
                    continue;// Skip triangle if it's out of cam view or it's part of the other side of the mesh.
                }
            }

            //------------------------ Lighting (world space)------------------------
            
            if (GraphicSettings::lighting && GraphicSettings::fillTriangles)
            {
                Vec3 lightSource = Direction::up + Direction::right + Direction::back * .3;
                float amountFacingLight = DotProduct((Vec3)worldSpaceTri.Normal(), lightSource);
                Color colorLit = (projectedTri.color* Clamp(amountFacingLight, 0.15, 1));

                projectedTri.color = colorLit;
            }

            if (GraphicSettings::vfx)
            {
                Vec3 screenLeftSide = Vec3(-1, 0, 0);
                Vec3 screenRightSide = Vec3(1, 0, 0);
                Range range = ProjectVertsOntoAxis(projectedTri.verts, 3, screenRightSide);
                bool leftHalfScreenX = range.min > 0 && range.max > 0;

                if (leftHalfScreenX) {
                    projectedTri.color = Color(0, 0, 255);// std::cout << "Inside" << std::endl;
                }
                else {
                    projectedTri.color = RGB::red;
                }
            }
            // ---------- Debugging -----------
            if (GraphicSettings::debugNormals)
            {
                //---------Draw point at centroid and a line from centroid to normal (view space & projected space)-----------
                Vec2 centroidToNormal_p = projectionMatrix * ((Vec3)camSpaceTri.centroid + camSpaceTri.normal);
                Vec2 centroid_p = projectionMatrix * camSpaceTri.centroid;
                pointBuffer->emplace_back(Point(centroid_p));
                lineBuffer->emplace_back(Line(centroid_p, centroidToNormal_p));
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
};
//List<Mesh*> Mesh::objects = List<Mesh*>(1000);
//int Mesh::meshCount = 0;
int Mesh::worldTriangleDrawCount = 0;

//------------------------------------CUBE MESH------------------------------------------
class CubeMesh : public Mesh
{
public:
    CubeMesh(int scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
        :Mesh(scale, position, rotationEuler) 
    {
        // Local Space (Object Space)
        this->vertices = new List<Vec3>({//new Vec3[8] {
            //south
            Vec3(-1, -1, 1),
            Vec3(-1, 1, 1),
            Vec3(1, 1, 1),
            Vec3(1, -1, 1),
            //north
            Vec3(-1, -1, -1),
            Vec3(-1, 1, -1),
            Vec3(1, 1, -1),
            Vec3(1, -1, -1)
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
class PlaneMesh : Mesh
{
public:
    PlaneMesh(int scale = 1, Vec3 position = Vec3(0, 0, 0), Vec3 rotationEuler = Vec3(0, 0, 0))
        :Mesh(scale, position, rotationEuler) {}
    
    List<Triangle>* MapVertsToTriangles()
    {
        static int calls = 0;
        if (calls < 1)
        {
            this->vertices = new List<Vec3> {
                Vec3(-1, -1, 0),
                Vec3(-1, 1, 0),
                Vec3(1, 1, 0),
                Vec3(1, -1, 0)
            };

            this->triangles = new List<Triangle>{
                Triangle((*vertices)[0], (*vertices)[1], (*vertices)[2]),
                Triangle((*vertices)[0], (*vertices)[2], (*vertices)[3])
            };
        }

        return triangles;
    }
};

//------------------------------HELPER FUNCTIONS------------------------------------------------

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

void Draw()
{
    // Camera TRInverse = (TR)^-1 = R^-1*T^-1 = M = Mcw = World to Camera coords. 
    // This matrix isn't used on the camera itself, but we record the reverse transformations of the camera going from world space back to local camera space.
    // Every point then in world space multiplied by this matrix will end up in a position relative to the camera's point of view when it was in world space. 
    // The camera could now be considered as the origin (0,0,0) with the zero rotation (identity matrix). 
    worldToViewMatrix = Camera::main->TRInverse();
    projectionMatrix = ProjectionMatrix();

    // ---------- Transform -----------
    for (int i = 0; i < Mesh::count; i++)
    {
        if (GraphicSettings::frustumCulling)
        {
            // Scale/Distance ratio culling
            /*bool tooSmallToSee = Mesh::objects[i]->scale.SqrMagnitude() / (Mesh::objects[i]->position - Camera::main->position).SqrMagnitude() < 0.000000125;
            if (tooSmallToSee) {
                return;
            }*/
            if (Camera::main != Mesh::objects[i]->root) {
                bool behindCamera = DotProduct((Mesh::objects[i]->root->position - Camera::main->position), Camera::main->Forward()) <= 0.0;
                if (behindCamera) {
                    continue;
                }
            }

            // if (Mesh::objects[i]->vertices)
            // {
            //     List<Vec3> verts = *(Mesh::objects[i]->vertices);
            //     for (size_t j = 0; j < verts.size(); j++)
            //     {
            //         verts[j] = ProjectionMatrix() * Camera::main->TRInverse() * Mesh::objects[i]->TRS() * verts[j];
            //     }

            //     Range range = ProjectVertsOntoAxis(verts.data(), verts.size(), Direction::left);
            //     if ((range.min > 1 && range.max > 1) || (range.min < -2 && range.max < -2)) {
            //         continue;
            //     }
            //     range = ProjectVertsOntoAxis(verts.data(), verts.size(), Direction::down);
            //     if ((range.min > 1 && range.max > 1) || (range.min < -2 && range.max < -2)) {
            //         continue;
            //     }
            // }

            // ---------- Debug -----------
            if (GraphicSettings::debugAxes)
            {
                Matrix4x4 mvp = ProjectionMatrix() * Camera::main->TRInverse() * Mesh::objects[i]->TRS();

                Vec2 center_p = mvp * Vec4(0, 0, 0, 1);
                Vec2 xAxis_p = mvp * Vec4(0.5, 0, 0, 1);
                Vec2 yAxis_p = mvp * Vec4(0, 0.5, 0, 1);
                Vec2 zAxis_p = mvp * Vec4(0, 0, 0.5, 1);
                Vec2 forward_p = mvp * (Direction::forward);

                pointBuffer->emplace_back(Point(center_p, RGB::red, 4));
                lineBuffer->emplace_back(Line(center_p, xAxis_p, RGB::red));
                lineBuffer->emplace_back(Line(center_p, yAxis_p, RGB::yellow));
                lineBuffer->emplace_back(Line(center_p, zAxis_p, RGB::blue));
                lineBuffer->emplace_back(Line(center_p, mvp * (Direction::forward), RGB::turquoise, 3));
            }
        }

        Mesh::objects[i]->TransformTriangles();
    }

    Mesh::worldTriangleDrawCount = triBuffer->size();

    // ---------- Sort (Painter's algorithm) -----------
    sort(triBuffer->begin(), triBuffer->end(), [](const Triangle& triA, const Triangle& triB) -> bool
        {
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