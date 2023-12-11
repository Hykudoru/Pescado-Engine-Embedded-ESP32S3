#pragma once
#ifndef ENGINE_H
#define ENGINE_H
// #include <iostream>
// #include <fstream>
// #include <sstream>
// #include <string>
// #include <math.h>
// #include <Matrix.h>
// #include <Graphics.h>
// #include <Physics.h>
// #include <Input.h>
// using namespace std;

// // Checking if DEBUGGING true in other scripts before using cout also ensures readable slow incremental output.
// bool DEBUGGING = false;
// void Debug()
// {
//     //debugging
//     DEBUGGING = false;
//     static double coutTimer = 0;
//     coutTimer += deltaTime;
//     if (coutTimer > 0.25 && coutTimer < 0.25 + deltaTime)
//     {
//         DEBUGGING = true;
//         coutTimer = 0;
//     }

//     if (DEBUGGING)
//     {
//         std::cout << "--------GRAPHICS-------" << endl;
//         std::cout << "FPS:" << fps << std::endl;
//         std::cout << "Frame Time:" << 1.0 / (double)fps << std::endl;
//         std::cout << "Meshes:" << Mesh::count << std::endl;
//         std::cout << "Triangles Drawn:" << Mesh::worldTriangleDrawCount << std::endl;
//     }
// }

// Mesh* textHelloWorld;
// Mesh* planet;
// Mesh* spaceShip;
// Mesh* spaceShip2;
// Mesh* spaceShip3;
// CubeMesh* obj1;
// CubeMesh* obj2;
// CubeMesh* obj3;
// CubeMesh* obj4;
// void Init()
// {
//     GraphicSettings::matrixMode = true;
//     //GraphicSettings::debugAxes = true;

//     //Mesh* cube0 = new CubeMesh(1, Vec3(0, 0, 0));
//     //Mesh* cube1 = new CubeMesh(1, Vec3(0, 0, -10));
//     CubeMesh* cube2 = new CubeMesh(15, Vec3(-5, 5, -20));
//     //CubeMesh* cube3 = new CubeMesh(1, Vec3(5, 5, -30));
//     // CubeMesh* cube5 = new CubeMesh(1, Vec3(-5, -5, 10));
//     // CubeMesh* cube6 = new CubeMesh(1, Vec3(-5, 5, 20));
//     // CubeMesh* cube7 = new CubeMesh(1, Vec3(5, 5, 30));
//     // CubeMesh* cube8 = new CubeMesh(10, Vec3(5, -5, 40));
// /*
//     planet = LoadMeshFromOBJFile("Sphere.obj");
//     planet->scale = Vec3(500, 500, 500);
//     planet->position += Direction::forward * 1000;
//     planet->color = &RGB::white;
    
//     textHelloWorld = LoadMeshFromOBJFile("Hello3DWorldText.obj");
//     textHelloWorld->scale = Vec3(2, 2, 2);
//     textHelloWorld->position = Vec3(0, 0, -490);
//     textHelloWorld->color = &RGB::green;
    
//     spaceShip = LoadMeshFromOBJFile("SpaceShip_2.2.obj");
//     spaceShip->position = Direction::left * 30 + Direction::forward * 10;

//     spaceShip2 = LoadMeshFromOBJFile("SpaceShip_3.obj");
//     spaceShip2->position = Direction::right * 40 + Direction::forward * 100;
//     spaceShip2->rotation = Matrix3x3::RotY(PI);

//     spaceShip3 = LoadMeshFromOBJFile("SpaceShip_5.obj");
//     spaceShip3->position = Direction::right * 20 + Direction::up * 10;

//     Mesh* parent = new CubeMesh();
//     Mesh* child = new CubeMesh();
//     Mesh* grandchild = new CubeMesh();
//     child->parent = parent;
//     grandchild->parent = child;

//     obj1 = new CubeMesh(1, Vec3(0, 10, 50), Vec3(0, 90, 0));
//     obj2 = new CubeMesh(2, Vec3(0, 0, -2), Vec3(0, 90, 0));
//     obj3 = new CubeMesh(2, Vec3(0, 0, -2), Vec3(0, 90, 0));
//     obj4 = new CubeMesh(2, Vec3(0, 0, -2), Vec3(0, 90, 0));
//     obj2->SetParent(obj1);
//     obj3->SetParent(obj2);
//     obj4->SetParent(obj3);
//     obj1->color = &RGB::red;
//     obj2->color = &RGB::orange;
//     obj3->color = &RGB::yellow;
//     obj4->color = &RGB::green;

// */
//     //Mesh* guitar = LoadMeshFromOBJFile("Objects/Guitar.obj");
//     //guitar->position += (Camera::main->Forward() * 10) + Camera::main->Right();
//     //Mesh* chair = LoadMeshFromOBJFile("Objects/Chair.obj");
//     //chair->position += (Camera::main->Forward() * 10) + Camera::main->Left();

//     //Plane* plane = new Plane(1, Vec3(0, 0, 0), Vec3(0, 0, 0));
//     /*for (size_t i = 1; i < Camera::cameras.size(); i++)//starts at 1 to avoid projector camera
//     {
//        Mesh* cameraMesh = LoadMeshFromOBJFile("Objects/Camera.obj");
//        cameraMesh->SetParent(Camera::cameras[i]);
//     }*/
// }

// void Update()
// {
   
// }
// Vec3 origin(128/2, 64/2, 0);

// //Vec3 origin(128/2, 64/2, 0);

// void Main()
// {
//     Time();
//     Input();
//     Physics();
//     Update();
//     //Draw();
//     Debug();
// }

#endif