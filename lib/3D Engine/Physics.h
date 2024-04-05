#pragma once
#ifndef PHYSICS_H
#define PHYSICS_H

#include <Arduino.h>
#include <math.h>
#include <Matrix.h>
#include <Graphics.h>
#include <Utility.h>
using namespace std;

Vec3 gravity = Vec3(0, -9.81, 0);
const float defaultAcceleration = 50;
float accel = defaultAcceleration;
float decel = -5;
float rotateSpeed = PI/2;//PI / 2;
bool isKinematic = false;
bool dampenersActive = true;
Vec3 moveDir = Vec3(0, 0, 0);
Vec3 velocity = Vec3(0, 0, 0);

extern bool DEBUGGING;

class Physics
{
public:
    static bool collisionDetection;
    static bool dynamics;
    static bool raycasting;
    static bool raycastDebugging;
    static bool gravity;
};
bool Physics::collisionDetection = true;
bool Physics::dynamics = true;
bool Physics::raycasting = false;
bool Physics::raycastDebugging = false;
bool Physics::gravity = false;

unsigned long deltaTimeMillis = 0.0;//time difference (in milliseconds) between each loop;
unsigned long deltaTimeMicros = 0.0;//time difference (in microseconds) between each loop
float deltaTime = 0;
int fps = 0;

void Time()
{
    
  static unsigned long prevMillisTime = millis();
  static unsigned long prevMicrosTime = micros();

  deltaTimeMillis = (millis() - prevMillisTime);
  prevMillisTime = millis();

  deltaTimeMicros = (micros() - prevMicrosTime);
  prevMicrosTime = micros();

    deltaTime = deltaTimeMillis*.001;

    static float t = 0;
    static int frames = 0;

    t += deltaTime;
    frames++;
    fps = ((float)frames) / t;
    if (t >= 1.0)
    {
        t = 0;
        frames = 0;
    }
}

unsigned long deltaTimeMillis2 = 0.0;//time difference (in milliseconds) between each loop;
unsigned long deltaTimeMicros2 = 0.0;//time difference (in microseconds) between each loop
float deltaTime2 = 0;
int fps2 = 0;

void Time2()
{
    
  static unsigned long prevMillisTime = millis();
  static unsigned long prevMicrosTime = micros();

  deltaTimeMillis2 = (millis() - prevMillisTime);
  prevMillisTime = millis();

  deltaTimeMicros2 = (micros() - prevMicrosTime);
  prevMicrosTime = micros();

    deltaTime2 = deltaTimeMillis2*.001;

    static float t = 0;
    static int frames = 0;

    t += deltaTime2;
    frames++;
    fps2 = ((float)frames) / t;
    if (t >= 1.0)
    {
        t = 0;
        frames = 0;
    }
}
/*
class Component
{

};

class GameObject
{
public:
    static List<GameObject*> gameObjects;
    static int gameObjectCount;
    List<Component> components;
    Transform* transform;
    Mesh* mesh;
    RigidBody* rigidBody;

    GameObject(Mesh& mesh, RigidBody& rigidBody)
    {
        this->mesh = &mesh;
        this->rigidBody = &rigidBody;
        GameObject::gameObjects.emplace(gameObjects.begin() + gameObjectCount++, this);
    }

    ~GameObject()
    {
        if (transform)
        {
            delete transform;
        }
        if (mesh)
        {
            delete mesh;
        }
        if (rigidBody)
        {
            delete rigidBody;
        }
    }

    static void Update()
    {
        for (size_t i = 0; i < gameObjectCount; i++)
        {
            if (gameObjects[i]->rigidBody)
            {
                gameObjects[i]->Update();
            }
        }
    }
};*/

class PhysicsObject;

class Component : public Transform
{
public:
    PhysicsObject* object;
};

class RigidBody
{
public:
    Vec3 centerOfMass = Vec3::zero;
    float mass = 1;
    bool isKinematic = false;
    Vec3 velocity = Vec3::zero;
    Vec3 angularVelocity = Vec3::zero;
};

class Collider : public Component, public  ManagedObjectPool<Collider>
{
public:
    Mesh* mesh;
    bool isStatic = false;
    bool isTrigger = false;
    float coefficientRestitution = 1.0;

    Collider(bool isStatic = false, bool isTrigger = false): ManagedObjectPool<Collider>(this)
    {
        this->isStatic = isStatic;
        this->isTrigger = isTrigger;
    }

    virtual ~Collider()
    {
        delete mesh;
    }

    List<Triangle>* MapVertsToTriangles()
    {
        return mesh->MapVertsToTriangles();
    }
    
    List<Vec3> WorldBounds()
    {
        return mesh->WorldVertices();
    }
};

class BoxCollider : public Collider, public ManagedObjectPool<BoxCollider>
{
public:
    BoxCollider(bool isStatic = false, bool isTrigger = false) : Collider(isStatic, isTrigger), ManagedObjectPool<BoxCollider>(this)
    {
        mesh = new CubeMesh();
        mesh->SetColor(Color::red);
        mesh->SetParent(this);
        mesh->SetVisibility(false);
    }
};

class SphereCollider : public Collider, public ManagedObjectPool<SphereCollider>
{
    float radius = 1;
public:
    
    SphereCollider(bool isStatic = false, bool isTrigger = false) : Collider(isStatic, isTrigger), ManagedObjectPool<SphereCollider>(this)
    {
        mesh = LoadMeshFromOBJFile("Sphere.obj");
        mesh->SetColor(Color::red);
        mesh->SetParent(this);
        mesh->SetVisibility(false);
    }

    float Radius()
    {
        return Scale().x;
    }
};

class PlaneCollider : public Collider, public ManagedObjectPool<PlaneCollider>
{
public:
    Vec3 normal;
    PlaneCollider(Vec3 normal, bool isStatic = false, bool isTrigger = false) : Collider(isStatic, isTrigger), ManagedObjectPool<PlaneCollider>(this)
    {
        mesh = new PlaneMesh();
        this->normal = normal;
        mesh->SetColor(Color::red);
        mesh->SetParent(this);
        mesh->SetVisibility(false);
    }
};

class PhysicsObject : public Transform, public RigidBody, public ManagedObjectPool<PhysicsObject>
{
public:
    Collider* collider;
    Mesh* mesh;

    PhysicsObject(Mesh* mesh, Collider* collider) : ManagedObjectPool<PhysicsObject>(this)
    { 
        SetCollider(collider);
        SetMesh(mesh);
    }

    PhysicsObject(float scale, Vec3 position, Matrix3x3 rotation, Mesh* mesh, Collider* collider) : ManagedObjectPool<PhysicsObject>(this)
    {
        //this->scale = Vec3(scale, scale, scale);
        this->localPosition = position;
        this->localRotation = rotation;
        this->localScale = Vec3(scale, scale, scale);
        SetCollider(collider);
        SetMesh(mesh);
    }

    virtual ~PhysicsObject()
    {
        delete collider;
        delete mesh;
    }

    void SetCollider(Collider* collider)
    {
        if (collider)
        {
            delete this->collider;
            this->collider = collider;
            this->collider->object = this;
            this->collider->SetParent(this, false);
        }
    }

    void SetMesh(Mesh* mesh)
    {
        if (mesh)
        {
            delete this->mesh;
            this->mesh = mesh;
            this->mesh->SetParent(collider, false);
        }
    }
};

struct CollisionInfo
{
    bool colliding = false;
};

struct BoxCollisionInfo : public CollisionInfo
{
    float minOverlap = 0;
    Vec3 minOverlapAxis = Vec3::zero;
};

struct SphereCollisionInfo : public CollisionInfo
{
    Vec3 pointOfContact = Vec3::zero;
    Vec3 lineOfImpact = Vec3::zero;
};

void CalculateCollision(Vec3 lineOfImpact, float& m1, float& m2, Vec3& v1, Vec3& v2, float e = 1.0)
{
    /* Elastic collision (conserves both momentum and kinetic energy)
    Conservation Momentum: m1*v1 + m2*v2 = m1*v1' + m2*v2'
    Conservation Kinetic Energy: v1 + v1' = v2 + v2'
    Coefficient of Restitution: e = v2'-v1' / v1-v2
        e = 1 Perfectly elastic
        0 < e < 1 inelastic
        e = 0 Perfectly inelastic
    */
    lineOfImpact.Normalize();
    Vec3 v1LineOfImpact = lineOfImpact * DotProduct(v1, lineOfImpact);
    Vec3 v2LineOfImpact = lineOfImpact * DotProduct(v2, lineOfImpact);
    Vec3 v1LineOfImpactFinal = (v1LineOfImpact * m1 + v2LineOfImpact * m2 * 2.0 - v1LineOfImpact * m2) * (1.0 / (m1 + m2));
    Vec3 v2LineOfImpactFinal = ((v1LineOfImpact - v2LineOfImpact) * e) + v1LineOfImpactFinal;// e(v1-v2)+v1' = v2'
    Vec3 v1PerpendicularFinal = (v1 - v1LineOfImpact);//Perpendicular Velocity is the same before and after impact
    Vec3 v2PerpendicularFinal = (v2 - v2LineOfImpact);//Perpendicular Velocity is the same before and after impact
    Vec3 v1Final = v1LineOfImpactFinal + v1PerpendicularFinal;
    Vec3 v2Final = v2LineOfImpactFinal + v2PerpendicularFinal;

    v1 = v1Final;
    v2 = v2Final;
}

void CalculateStaticCollision(Vec3 lineOfImpact, Vec3& v1, float e = 1.0)
{
    Vec3 v1LineOfImpact = lineOfImpact * DotProduct(v1, lineOfImpact);
    Vec3 v1PerpendicularFinal = (v1 - v1LineOfImpact);//Perpendicular Velocity is the same before and after impact
    v1 = (v1LineOfImpact * -e) + v1PerpendicularFinal;
}

bool SpherePlaneColliding(SphereCollider& sphere, PlaneCollider& plane, SphereCollisionInfo& collisionInfo, bool resolve = true)
{
    float radius = sphere.Radius();
    Vec3 sphereCenter = sphere.Position();
    Vec3 v = sphereCenter - plane.Position();
    Vec3 normal = plane.normal;
    Vec3 vPerp = normal * (DotProduct(v, normal));//ProjectOnPlane(v, plane.plane.normal);
    Vec3 closestPointOnPlane = sphereCenter - vPerp;

    if ((closestPointOnPlane - sphereCenter).SqrMagnitude() < radius * radius)
    {
        collisionInfo.colliding = true;
        if (resolve)
        {
            Vec3 pointOnSphere = ClosestPointOnSphere(sphereCenter, radius, closestPointOnPlane);
            Vec3 offset = pointOnSphere - closestPointOnPlane;//overlapping
            sphere.root->localPosition -= offset;
            collisionInfo.lineOfImpact = normal * -1.0;
        }
    }

    if (Graphics::debugPlaneCollisions)
    {
        Vec3 vProj = ProjectOnPlane(v, normal);
        Line::AddWorldLine(Line(plane.Position(), plane.Position() + normal, Color::gray));
        Line::AddWorldLine(Line(sphereCenter, closestPointOnPlane, Color::red));
        Point::AddWorldPoint(Point(sphereCenter, Color::gray, 10));
        Point::AddWorldPoint(Point(closestPointOnPlane, Color::red, 10));
    }

    return collisionInfo.colliding;
}

bool SpheresColliding(SphereCollider& sphere1, SphereCollider& sphere2, SphereCollisionInfo& collisionInfo, bool resolve = true)
{
    float radius1 = sphere1.Radius();
    float radius2 = sphere2.Radius();
    Vec3 sphere1Pos = sphere1.Position();
    Vec3 sphere2Pos = sphere2.Position();
    Vec3 pointOnSphere1 = ClosestPointOnSphere(sphere1Pos, radius1, sphere2Pos);
    Vec3 pointOnSphere2 = ClosestPointOnSphere(sphere2Pos, radius2, sphere1Pos);
    float squareMagToPointOnSphere2 = (pointOnSphere2 - sphere1Pos).SqrMagnitude();
    collisionInfo.colliding = squareMagToPointOnSphere2 < radius1 * radius1;
    if (collisionInfo.colliding)
    {
        Vec3 offset = (pointOnSphere1 - pointOnSphere2);
        collisionInfo.lineOfImpact = offset;
        if (resolve) {
            offset *= 0.5;
            sphere1.root->localPosition -= offset;
            sphere2.root->localPosition += offset;
            collisionInfo.pointOfContact = (pointOnSphere1 + pointOnSphere2) * 0.5;
        }
    }

    if (Graphics::debugSphereCollisions)
    {
        Point::AddWorldPoint(Point(pointOnSphere1, Color::red, 4));
        Point::AddWorldPoint(Point(pointOnSphere2, Color::red, 4));
        Line::AddWorldLine(Line(pointOnSphere1, pointOnSphere2));
    }

    return collisionInfo.colliding;
}

// Oriented Bounding Box (OBB) with Separating Axis Theorem (SAT) algorithm
bool OBBSATColliding(BoxCollider& box1, BoxCollider& box2, BoxCollisionInfo& collisionInfo, bool resolve = true)
{
    bool gap = true;

    collisionInfo = BoxCollisionInfo();

    if (box1.isStatic && box2.isStatic)
    {
        return false;
    }

    List<Vec3> physObj1Verts = box1.WorldBounds();
    List<Vec3> physObj2Verts = box2.WorldBounds();
    List<Vec3> physObj1Normals = List<Vec3>{ box1.root->localRotation * Direction::right, box1.root->localRotation * Direction::up, box1.root->localRotation * Direction::forward };// mesh1.WorldXYZNormals();
    List<Vec3> physObj2Normals = List<Vec3>{ box2.root->localRotation * Direction::right, box2.root->localRotation * Direction::up, box2.root->localRotation * Direction::forward }; //mesh2.WorldXYZNormals();

    // Note: Collision detection stops if at any time a gap is found.
    // Note: Cache the minimum distance projection and axis for later use to resolve the collision if needed.
    // Step 1: Project both meshes onto Mesh A's normal axes.
    // Step 2: Project both meshes onto Mesh B's normal axes.
    //Initialize minimum projection distance and axis
    for (size_t i = 0; i < 2; i++)
    {
        auto normals = i == 0 ? &physObj1Normals : &physObj2Normals;
        for (size_t ii = 0; ii < normals->size(); ii++)
        {
            Vec3 axis = (*normals)[ii];
            Range rangeA = ProjectVertsOntoAxis(physObj1Verts.data(), physObj1Verts.size(), axis);
            Range rangeB = ProjectVertsOntoAxis(physObj2Verts.data(), physObj2Verts.size(), axis);
            gap = !((rangeA.max >= rangeB.min && rangeB.max >= rangeA.min));// || (mesh1Range.max < mesh2Range.min && mesh2Range.max < mesh1Range.min));
            if (gap) {
                collisionInfo.colliding = false;
                return false;
            }

            //Compare and cache minimum projection distance and axis for later use if needed for collision resolution.
            float potentialMinOverlap = 0;
            if (rangeA.max > rangeB.max) {
                potentialMinOverlap = rangeB.max - rangeA.min;
                axis *= -1.0;// Reverse push direction since object B is behind object A and we will always push A backwards and B forwards.
            }
            else {
                potentialMinOverlap = rangeA.max - rangeB.min;
            }

            if (i == 0)
            {
                collisionInfo.minOverlap = potentialMinOverlap;
                collisionInfo.minOverlapAxis = axis;
            }
            else if (potentialMinOverlap < collisionInfo.minOverlap)
            {
                collisionInfo.minOverlap = potentialMinOverlap;
                collisionInfo.minOverlapAxis = axis;
            }
        }
    }

    // Step 3: Must continue searching for possible 3D Edge-Edge collision
    if (!gap)
    {
        for (size_t i = 0; i < physObj1Normals.size(); i++)
        {
            Vec3 nA = physObj1Normals[i];
            for (size_t j = 0; j < physObj2Normals.size(); j++)
            {
                Vec3 nB = physObj2Normals[j];

                //Make sure normals are not the same before using them to calculate the cross product (otherwise the axis would be <0, 0, 0>).
                float dot = DotProduct(nA, nB);
                bool sameAxis = dot >= 1.0 || dot <= -1.0;
                if (sameAxis)
                {
                    if ((j + 1) >= physObj2Normals.size()) {
                        nB = physObj2Normals[j - 1];
                    }
                    else {
                        nB = physObj2Normals[j + 1];
                    }
                }

                // Search for possible 3D Edge-Edge collision
                Vec3 axis = CrossProduct(nA, nB);
                Range rangeA = ProjectVertsOntoAxis(physObj1Verts.data(), physObj1Verts.size(), axis);
                Range rangeB = ProjectVertsOntoAxis(physObj2Verts.data(), physObj2Verts.size(), axis);
                gap = !((rangeA.max >= rangeB.min && rangeB.max >= rangeA.min));// || (mesh1Range.max < mesh2Range.min && mesh2Range.max < mesh1Range.min));
                if (gap) {
                    collisionInfo.colliding = false;
                    return false;
                }
                /* To-Do...
                 //Compare and cache minimum projection distance and axis for later use if needed for collision resolution.
                 float potentialMinOverlap = 0;
                 if (rangeA.max > rangeB.max) {
                     potentialMinOverlap = rangeB.max - rangeA.min;
                     axis *= -1.0;// Reverse push direction since object B is behind object A and we will always push A backwards and B forwards.
                 }
                 else {
                     potentialMinOverlap = rangeA.max - rangeB.min;
                 }

                 if (potentialMinOverlap < collisionInfo.minOverlap)
                 {
                     collisionInfo.minOverlap = potentialMinOverlap;
                     collisionInfo.minOverlapAxis = axis;
                 }*/
            }
        }
    }

    collisionInfo.colliding = !gap;

    if (collisionInfo.colliding && resolve)
    {
        Vec3 offset = collisionInfo.minOverlapAxis * collisionInfo.minOverlap;

        bool neitherStatic = !box1.isStatic && !box2.isStatic;
        if (neitherStatic)
        {
            offset *= 0.5;
            box1.root->localPosition -= (offset * 1.01);
            box2.root->localPosition += (offset * 1.01);
        }
        //Only one is movable at this stage
        else if (box1.isStatic) {
            box2.root->localPosition += offset;
        }
        else {
            box1.root->localPosition -= offset;
        }
    }

    return collisionInfo.colliding;
}

void DetectCollisions()
{
    // How nested loop algorithm works: 
    // Gets colliders A, B, C, D, E...
    // Compare A:B, A:C, A:D, A:E
    // Compare B:C, B:D, B:E
    // Compare C:D, C:E
    // Compare D:E
    for (size_t i = 0; i < ManagedObjectPool<BoxCollider>::count; i++)
    {
        // exit if this is the last Collider
        if ((i + 1) >= ManagedObjectPool<BoxCollider>::count) {
            break;
        }

        // Current Collider
        BoxCollider* box1 = ManagedObjectPool<BoxCollider>::objects[i];

        for (size_t j = i + 1; j < ManagedObjectPool<BoxCollider>::count; j++)
        {
            // Next Collider
            BoxCollider* box2 = ManagedObjectPool<BoxCollider>::objects[j];

            if (box1->isStatic && box2->isStatic) {
                continue;
            }

            BoxCollisionInfo collisionInfo;
            bool resolveIfNotTrigger = !(box1->isTrigger || box2->isTrigger);
            if (OBBSATColliding(*box1, *box2, collisionInfo, resolveIfNotTrigger))
            {
                if (Physics::dynamics)
                {
                    if (box1->object->isKinematic || box2->object->isKinematic) {
                        continue;
                    }
                    /*
                    Although static objects themselves are not effected by momentum
                    transfers, their velocity variable may still be updating from new collisions.
                    Consequently, objects touching a static collider would be effected, so the
                    velocity is zeroed out to prevent this.
                    */
                    if (box1->isStatic)
                    {
                        box1->object->velocity = Vec3::zero;
                    }
                    else if (box2->isStatic)
                    {
                        box2->object->velocity = Vec3::zero;
                    }

                    CalculateCollision(
                        collisionInfo.minOverlapAxis,
                        box1->object->mass,
                        box2->object->mass,
                        box1->object->velocity,
                        box2->object->velocity,
                        1.0
                    );
                }
            }
        }
    }

    for (size_t i = 0; i < ManagedObjectPool<SphereCollider>::count; i++)
    {
        // Current Collider
        SphereCollider* sphere1 = ManagedObjectPool<SphereCollider>::objects[i];

        // SPHERE-SPHERE COLLISIONS
        for (size_t j = i + 1; j < ManagedObjectPool<SphereCollider>::count; j++)
        {
            // Next Collider
            SphereCollider* sphere2 = ManagedObjectPool<SphereCollider>::objects[j];

            if (sphere1->isStatic && sphere2->isStatic) {
                continue;
            }

            SphereCollisionInfo collisionInfo;
            bool resolveIfNotTrigger = !(sphere1->isTrigger || sphere2->isTrigger);
            if (SpheresColliding(*sphere1, *sphere2, collisionInfo, resolveIfNotTrigger))
            {
                if (Physics::dynamics)
                {
                    if (sphere1->object->isKinematic || sphere2->object->isKinematic) {
                        continue;
                    }
                    if (sphere1->isStatic)
                    {
                        CalculateStaticCollision(collisionInfo.lineOfImpact, sphere2->object->velocity, sphere2->coefficientRestitution);
                    }
                    else if (sphere2->isStatic)
                    {
                        CalculateStaticCollision(collisionInfo.lineOfImpact, sphere1->object->velocity, sphere1->coefficientRestitution);
                    }
                    else
                    {
                        CalculateCollision(
                            collisionInfo.lineOfImpact,
                            sphere1->object->mass,
                            sphere2->object->mass,
                            sphere1->object->velocity,
                            sphere2->object->velocity,
                            1.0
                        );
                    }
                }
            }
        }

        // SPHERE-PLANE COLLISIONS
        for (size_t ii = 0; ii < ManagedObjectPool<PlaneCollider>::count; ii++)
        {
            // Next Collider
            PlaneCollider* plane = ManagedObjectPool<PlaneCollider>::objects[ii];

            if (sphere1->isStatic && plane->isStatic) {
                continue;
            }

            SphereCollisionInfo collisionInfo;
            bool resolveIfNotTrigger = !(sphere1->isTrigger || plane->isTrigger);
            if (SpherePlaneColliding(*sphere1, *plane, collisionInfo, resolveIfNotTrigger))
            {
                if (Physics::dynamics)
                {
                    if (sphere1->object->isKinematic || plane->object->isKinematic) {
                        continue;
                    }
                    if (sphere1->isStatic)
                    {
                        //...
                    }
                    else if (plane->isStatic)
                    {
                        CalculateStaticCollision(collisionInfo.lineOfImpact, sphere1->object->velocity, sphere1->coefficientRestitution);
                    }
                    else
                    {
                        CalculateCollision(
                            collisionInfo.lineOfImpact,
                            sphere1->object->mass,
                            plane->object->mass,
                            sphere1->object->velocity,
                            plane->object->velocity,
                            1.0
                        );
                    }
                }
            }
        }
    }
}

class Ray
{
protected:
    Transform transform = Transform();
    Vec3 endPosition;
    Vec3 direction;
public:

    float distance = 1;

    Ray(Vec3 start, Vec3 end)
    {
        Set(start, end);
    }

    Ray(Vec3 start, Vec3 dir, float dist)
    {
        Set(start, start + dir * dist);
    }

    void Set(Vec3 start, Vec3 end)
    {
        Vec3 disp = (end - start);
        distance = disp.Magnitude();
        direction = disp.Normalized();
        transform.localRotation = OrthogonalMatrixLookAt(direction);
        transform.localPosition = start;
        endPosition = start + direction * distance;
    }

    Vec3 StartPosition()
    {
        return transform.Position();
    }

    Vec3 EndPosition()
    {
        return endPosition;
    }

    Vec3 Direction()
    {
        return direction;
    }

    Matrix4x4 WorldToRaySpaceMatrix()
    {
        return transform.TRInverse();
    }
   
    // Builds a 3x3 orthogonal matrix with its -Z axis facing the given direction (similar to a camera with no rotation.
    static Matrix3x3 OrthogonalMatrixLookAt(Vec3 direction)
    {
        // Cached staticallly since highly improbable the initial random vector will ever be exactly aligned with direction arg. 
        // Prevent recalculating a random vector every call.
        static Vec3 randomDirection = RandomDirection();

        Vec3 rayZ = direction * -1.0;
        if (rayZ == randomDirection) {
            randomDirection = RandomDirection();
        }
        Vec3 rayX = CrossProduct(rayZ, randomDirection);
        Vec3 rayY = CrossProduct(rayZ, rayX);
        rayX.Normalize();
        rayY.Normalize();
        float rotationMatrix[3][3] = {
            { rayX.x, rayY.x, rayZ.x },
            { rayX.y, rayY.y, rayZ.y },
            { rayX.z, rayY.z, rayZ.z }
        };

        return rotationMatrix;
    }
};

template <typename T>
struct RaycastInfo
{
    T* objectHit = nullptr;
    Triangle* triangleHit = nullptr;
    Triangle triangleHit_w = Triangle();
    Vec3 contactPoint = Vec3::zero;
    RaycastInfo() {};
    RaycastInfo(T* objHit, Triangle* triHit, Vec3& contactPoint, Triangle& worldSpaceTri)
    {
        this->objectHit = objHit;
        this->triangleHit = triHit;
        this->contactPoint = contactPoint;
        this->triangleHit_w = worldSpaceTri;
    }
};

template <typename T>
bool Raycast(Ray& ray, RaycastInfo<T>& raycastInfo, const std::function<void(RaycastInfo<T>&)>& callback = NULL)
{
    Vec3 from = ray.StartPosition();
    Vec3 to = ray.EndPosition();

    float closestSqrDistHit = ray.distance * ray.distance;
    for (size_t i = 0; i < ManagedObjectPool<T>::objects.size(); i++)
    {
        List<Triangle>* triangles = ManagedObjectPool<T>::objects[i]->MapVertsToTriangles();
        for (size_t j = 0; j < triangles->size(); j++)
        {
            Triangle worldSpaceTri = (*triangles)[j];
            for (size_t k = 0; k < 3; k++) {
                worldSpaceTri.verts[k] = ManagedObjectPool<T>::objects[i]->TRS() * worldSpaceTri.verts[k];
            }
            //------------------Ray casting (World & Ray Space)--------------------------
            Vec3 pointOfIntersection;
            if (LinePlaneIntersecting(from, to, worldSpaceTri, &pointOfIntersection))
            {
                if (DotProduct(pointOfIntersection - from, ray.Direction()) >= 0)
                {
                    Line::AddWorldLine(Line(from, to, Color::red));

                    Matrix4x4 worldToRaySpaceMatrix = ray.WorldToRaySpaceMatrix();
                    Vec3 pointOfIntersection_v = worldToRaySpaceMatrix * pointOfIntersection;
                    Triangle* viewSpaceTri = &((*triangles)[j]);
                    for (size_t k = 0; k < 3; k++) {
                        viewSpaceTri->verts[k] = worldToRaySpaceMatrix * worldSpaceTri.verts[k];
                    }
                    if (PointInsideTriangle(pointOfIntersection_v, viewSpaceTri->verts))
                    {
                        // Check if within range
                        float sqrDist = (pointOfIntersection - from).SqrMagnitude();
                        if (sqrDist <= closestSqrDistHit)
                        {
                            closestSqrDistHit = sqrDist;

                            raycastInfo.objectHit = ManagedObjectPool<T>::objects[i];
                            raycastInfo.contactPoint = pointOfIntersection;
                            raycastInfo.triangleHit = &(*triangles)[j];
                            raycastInfo.triangleHit_w = worldSpaceTri;

                            if (callback) {
                                callback(raycastInfo);
                            }
                         
                            // ---------- Debugging -----------
                            if (Physics::raycastDebugging)
                            {
                                Point::AddWorldPoint(Point(pointOfIntersection_v, Color::red, 5));
                                // Reflect
                                Vec3 n = worldSpaceTri.Normal();
                                Vec3 v = (pointOfIntersection - from);
                                Vec3 reflection = Reflect(v, n);
                                Line::AddWorldLine(Line(pointOfIntersection, pointOfIntersection + reflection, Color::red));

                                // Project
                                Vec3 vecPlane = ProjectOnPlane(v, n);
                                Line::AddWorldLine(Line(pointOfIntersection, pointOfIntersection + vecPlane, Color::pink));
                            }
                        }
                    }
                }
            }
        }
    }
    return raycastInfo.objectHit != nullptr;
}

template <typename T>
bool Raycast(Vec3 from, Vec3 to, RaycastInfo<T>& raycastInfo, const std::function<void(RaycastInfo<T>&)>& callback = NULL)
{
    Ray ray = Ray(from, to);
    return Raycast(ray, raycastInfo);
}

static void Physics()
{
    Camera* cam;
    if (CameraSettings::outsiderViewPerspective)
    {
        cam = Camera::projector;
    }
    else {
        cam = Camera::main;
    }
    //---------- Physics Update ------------
    if (isKinematic)
    {
        cam->localPosition += moveDir * accel * deltaTime;
    }
    else
    {
        velocity += moveDir * accel * deltaTime;
        if (Physics::gravity) {
            velocity += gravity * deltaTime;
        }
        else if (dampenersActive) {
            velocity += velocity * decel * deltaTime;
        }
        if (velocity.SqrMagnitude() < 0.0001) {
            velocity = Vec3::zero;
        }
        cam->localPosition += velocity * deltaTime;
    }

    if (Physics::dynamics)
    {
        for (size_t i = 0; i < ManagedObjectPool<PhysicsObject>::count; i++)
        {
            PhysicsObject* obj = ManagedObjectPool<PhysicsObject>::objects[i];
            if (!obj->collider->isStatic && !obj->isKinematic)
            {
                if (Physics::gravity) {
                    obj->velocity += gravity * deltaTime;
                }
                obj->localPosition += obj->velocity * deltaTime;
            }
        }
    }

    if (Physics::collisionDetection)
    {
        DetectCollisions();
    }

    if (Physics::raycasting)
    {
        Ray ray1 = Ray(Camera::cameras[1]->Position(), Camera::cameras[1]->Position() + Camera::cameras[1]->Forward() * 50);
        RaycastInfo<Mesh> info;
        if (Raycast(ray1, info))
        {
            //cout << "RAYCAST HIT" << '\n';
            Line::AddWorldLine(Line(ray1.StartPosition(), ray1.EndPosition(), Color::green, 3));
            Point::AddWorldPoint(Point(info.contactPoint, Color::green, 7));
            //info.objectHit->SetColor(Color::purple);
            info.triangleHit->color = Color::purple;//Color::Random();
        }
        
        Ray ray2 = Ray(Camera::cameras[2]->Position(), Camera::cameras[2]->Forward(), 50);
        RaycastInfo<Collider> info2;
        if (Raycast<Collider>(ray2, info2))
        {
            //cout << "RAYCAST HIT" << '\n';
            Line::AddWorldLine(Line(ray2.StartPosition(), ray2.EndPosition(), Color::green, 3));
            Point::AddWorldPoint(Point(info2.contactPoint, Color::green, 7));
            info2.objectHit->object->mesh->SetColor(Color::red);
        }
    }

    if (DEBUGGING)
    {
        std::cout << "--------PHYSICS-------" << endl;

        string onoff = Physics::collisionDetection ? "On" : "Off";
        std::cout << "Collisions: " << onoff << " (press P)" << endl;

        onoff = Physics::gravity ? "On" : "Off";
        std::cout << "Gravity: " << onoff << " (press G)" << endl;

        std::cout << "Sphere Colliders: " << ManagedObjectPool<SphereCollider>::count << endl;
        std::cout << "Box Colliders: " << ManagedObjectPool <BoxCollider>::count << endl;
        std::cout << "Plane Colliders: " << ManagedObjectPool<PlaneCollider>::count << endl;
        std::cout << "Colliders: " << Collider::count << endl;

        std::cout << "--------PLAYER-------" << endl;

        onoff = isKinematic ? "On" : "Off";
        std::cout << "Kinematic: " << onoff << " (press X)" << endl;

        onoff = dampenersActive ? "On" : "Off";
        std::cout << "Inertial Dampeners: " << onoff << " (press Z)" << endl;

        std::cout << "Position: (" << Camera::main->Position().x << ", " << Camera::main->Position().y << ", " << Camera::main->Position().z << ")" << endl;
        std::cout << "Velocity: <" << velocity.x << ", " << velocity.y << ", " << velocity.z << ">" << endl;
    }
}
#endif