//glColor3f(1,0.1,0.2);
#define SFML_STATIC
#include <iostream>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include "btBulletDynamicsCommon.h"
#define width 700
#define height 700
#define PI 3.1415

void tiklanma(sf::Event& event);
bool tiklandimi = false;
bool basildimi = false;
bool bom = false;
static float mouse_pos_x = 0;
static float mouse_pos_y = 0;
float x = 0;
float x_a = 0;
float z = 0;
float z_a = 0;
float y = 0;
float y_a = 0;
float g_x = 0;
float g_y = 0;
float g_z = 0;
float wx = 0;
float wy = 0;
float wz = 0;
float mesx = 0;
int rats = 0;
float yaw = 0;
float pitch = 0;
float sensivity = 4;
float r_g = 1;
float r_r = 0;
float r_b = 0.5;
bool t_rank = false;

float timeass = 0.0;
static btScalar matrix[16];
static btTransform trans;
static btTransform b_trans;
static btDiscreteDynamicsWorld *dynamicsWorld;

static btRigidBody *box1, *box2, *h_body;

sf::Texture tt;

static void sekiller(void);
sf::Clock sure;
static void timer(void)
{
    float dtime = timeass;
    timeass = float(sure.getElapsedTime().asSeconds()) / 5.0f;
    dtime = timeass - dtime;
    if(dynamicsWorld)
        {dynamicsWorld->stepSimulation(dtime, 10);}
    rats += 1;
    if(rats%4==0)
    {
        rats = 0;
        if(t_rank)
            t_rank = false;
        else if(!t_rank)
            t_rank = true;
    }
    r_g -= 0.1;
    r_r += 0.1;
    r_b += 0.1;
    if(r_g<0)
        r_g=1;
    if(r_r>1)
        r_r=1;
    if(r_b>1)
        r_b=1;
}

void celsitli()
{
    mesx = sqrt((trans.getOrigin().x()-wx)*(trans.getOrigin().x()-wx)+(trans.getOrigin().y()-wy)*(trans.getOrigin().y()-wy)+(trans.getOrigin().z()-wz)*(trans.getOrigin().z()-wz));
}

void perspektif()
{
    yaw = mouse_pos_x/10*sensivity;
    pitch = mouse_pos_y/10*sensivity;
    if(pitch>90.0f)
        pitch=90.0f;
    if(pitch<-90.0f)
        pitch=-90.0f;
    glViewport(0.0f, 0.0f,(GLsizei)width,(GLsizei)height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glm::mat4 projectionMatrix = glm::perspective(45.0f,(float)width/(float)height,0.001f,10000.0f);
    glm::vec3 rot;
    glm::vec3 poss = glm::vec3(0.0f,1.0f,0.0f);
    glm::vec3 position = glm::normalize(poss);
    rot.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    rot.y = -sin(glm::radians(pitch));
    rot.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    glm::vec3 rott = glm::normalize(rot);
    glm::vec3 right = glm::normalize(glm::cross(rott,position));
    glm::vec3 up = glm::normalize(glm::cross(right,rott));
    glm::mat4 look = glm::lookAt(position,position+rott,up);
    glm::mat4 transleted = glm::translate(glm::mat4(1.0f),glm::vec3(-x_a,-y_a,-z_a));
    glm::mat4 fullTransform = projectionMatrix * look * transleted;
    glLoadMatrixf(&fullTransform[0][0]);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if(tiklandimi)
    {
        x += rot.x*1.2;
        y += rot.y*1.2;
        z += rot.z*1.2;
    }
    else
    {
        if(x<0)
            x+=0.01;
        else if(x>0)
            x-=0.01;
        if(y<0)
            y+=0.01;
        else if(y>0)
            y-=0.01;
        if(z<0)
            z+=0.01;
        else if(z>0)
            z-=0.01;
        if(x == 0.01 || x == -0.01)
            x = 0;
        if(y == 0.01 || y == -0.01)
            y = 0;
        if(z == 0.01 || z == -0.01)
            z = 0;
    }
    if(basildimi)
    {
        x = 0;
        z = 0;
        y = 0;
        h_body->setLinearVelocity(btVector3(0,0,0));
    }
    if(tiklandimi)
    {
        h_body->setLinearVelocity(btVector3(x,y,z));
    }
    b_trans.setRotation(btQuaternion(rot.x,rot.y,rot.z));

    wx = rot.x*10+x_a;
    wy = rot.y*10+y_a;
    wz = rot.z*10+z_a;

    x_a = b_trans.getOrigin().x();
    z_a = b_trans.getOrigin().z();
    y_a = b_trans.getOrigin().y();
}

int main()
{
    sf::ContextSettings contextSettings;
    contextSettings.depthBits = 24;
    contextSettings.stencilBits = 8;
    contextSettings.antialiasingLevel = 0;
    contextSettings.majorVersion = 3;
    contextSettings.minorVersion = 3;

    sf::Window window(sf::VideoMode(width, height), "HELLO WORLD !!", sf::Style::Default, contextSettings);
    window.setFramerateLimit(30);
    glViewport(0, 0, width, height);
    perspektif();
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    tt.loadFromFile("surreal.png");

    btQuaternion qtn;

    btCollisionShape *shape;
    btDefaultMotionState *motionState;

    btDefaultCollisionConfiguration *collisionCfg
    = new btDefaultCollisionConfiguration();

    btAxisSweep3 *axisSweep
    = new btAxisSweep3(btVector3(-100,-100,-100), btVector3(100,100,100), 128);

    dynamicsWorld = new btDiscreteDynamicsWorld(new btCollisionDispatcher(collisionCfg),
    axisSweep, new btSequentialImpulseConstraintSolver, collisionCfg);

    dynamicsWorld->setGravity(btVector3(0, 0, 0));

    // box1 - STATIC / mass=btScalar(0.0)
    shape = new btBoxShape(btVector3(10,10,10));
    trans.setIdentity();
    qtn.setEuler(0, 0, 0);
    trans.setRotation(qtn);
    trans.setOrigin(btVector3(0, 0, 0));
    motionState = new btDefaultMotionState(trans);
    box1 = new btRigidBody(btScalar(0.0), motionState, shape, btVector3(0,0,0));
    dynamicsWorld->addRigidBody(box1);
    //** box2 - DYNAMIC / mass=btScalar(1.0)
    shape = new btBoxShape(btVector3(1,1,1));
    trans.setIdentity();
    qtn.setEuler(45, 0.7, 0.4);
    trans.setRotation(qtn);
    trans.setOrigin(btVector3(0, 20, 0));
    motionState = new btDefaultMotionState(trans);
    box2 = new btRigidBody(btScalar(1.0), motionState, shape, btVector3(1,1,1));
    dynamicsWorld->addRigidBody(box2);
    //** body - DYNAMIC / mass=btScalar(1.0)
    shape = new btBoxShape(btVector3(0.1,0.1,0.1));
    b_trans.setIdentity();
    b_trans.setRotation(btQuaternion(0,0,0));
    b_trans.setOrigin(btVector3(0, 10, 5));
    motionState = new btDefaultMotionState(b_trans);
    h_body = new btRigidBody(btScalar(1.0), motionState, shape, btVector3(1,1,1));
    dynamicsWorld->addRigidBody(h_body);
    h_body->setGravity(btVector3(0,0,0));

while (window.isOpen())
{
    if(t_rank)
        glClearColor(0, 0, 0, 1.0);
    if(!t_rank)
        glClearColor(1, 1, 1, 1.0);
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    perspektif();
    celsitli();
    timer();
    mouse_pos_x = (float(sf::Mouse::getPosition().x-600));
    mouse_pos_y = (float(sf::Mouse::getPosition().y-300));
    sf::Event event;
    while (window.pollEvent(event))
    {
        if (event.type == sf::Event::Closed)
            window.close();
            tiklanma(event);
    }

    if(basildimi)
    {
        if((wx) < (trans.getOrigin().x()))
            g_x = -9.81*PI*fabs(trans.getOrigin().x()-wx);
        else if((wx) > (trans.getOrigin().x()))
            g_x = 9.81*PI*fabs(trans.getOrigin().x()-wx);
        if((wy) < (trans.getOrigin().y()))
            g_y = -9.81*PI*fabs(trans.getOrigin().y()-wy);
        else if((wy) > (trans.getOrigin().y()))
            g_y = 9.81*PI*fabs(trans.getOrigin().y()-wy);
        if((wz) < (trans.getOrigin().z()))
            g_z = -9.81*PI*fabs(trans.getOrigin().z()-wz);
        else if((wz) > (trans.getOrigin().z()))
            g_z = 9.81*PI*fabs(trans.getOrigin().z()-wz);
        if(ceil(wy) == ceil(trans.getOrigin().y()) && ceil(wx) == ceil(trans.getOrigin().x()) && ceil(wz) == ceil(trans.getOrigin().z()))
        {
            box2->setLinearVelocity(btVector3(0,0,0));
            trans.setOrigin(btVector3(wx,wy,wz));
            box2->translate(btVector3(0,0,0));
        }
        else
        {
            box2->setLinearVelocity(btVector3(g_x/2,g_y/2,g_z/2));
        }
    }
    glEnable(GL_TEXTURE_2D);
    glLoadIdentity();
    sf::Texture::bind(&tt);
    glLoadIdentity();
    glPushMatrix();
    box2->getMotionState()->getWorldTransform(trans);
    trans.getOpenGLMatrix(matrix);
    glMultMatrixf(matrix);
    glBegin(GL_QUADS);
    glColor3f(r_r-0.4,r_g-0.1,r_b+0.2);
    glTexCoord2d(0,0);
    glVertex3f(1,-1,1);
    glTexCoord2d(0,1);
    glVertex3f(1,-1,-1);
    glTexCoord2d(1,1);
    glVertex3f(-1,-1,-1);
    glTexCoord2d(1,0);
    glVertex3f(-1,-1,1);
    glTexCoord2d(0,0);
    glVertex3f(1,1,1);
    glTexCoord2d(0,1);
    glVertex3f(1,1,-1);
    glTexCoord2d(1,1);
    glVertex3f(-1,1,-1);
    glTexCoord2d(1,0);
    glVertex3f(-1,1,1);
    glTexCoord2d(0,0);
    glVertex3f(1,1,1);
    glTexCoord2d(0,1);
    glVertex3f(1,-1,1);
    glTexCoord2d(1,1);
    glVertex3f(-1,-1,1);
    glTexCoord2d(1,0);
    glVertex3f(-1,1,1);
    glTexCoord2d(0,0);
    glVertex3f(1,1,-1);
    glTexCoord2d(0,1);
    glVertex3f(1,-1,-1);
    glTexCoord2d(1,1);
    glVertex3f(-1,-1,-1);
    glTexCoord2d(1,0);
    glVertex3f(-1,1,-1);
    glTexCoord2d(0,0);
    glVertex3f(-1,1,-1);
    glTexCoord2d(0,1);
    glVertex3f(-1,1,1);
    glTexCoord2d(1,1);
    glVertex3f(-1,-1,1);
    glTexCoord2d(1,0);
    glVertex3f(-1,-1,-1);
    glTexCoord2d(0,0);
    glVertex3f(1,1,-1);
    glTexCoord2d(0,1);
    glVertex3f(1,1,1);
    glTexCoord2d(1,1);
    glVertex3f(1,-1,1);
    glTexCoord2d(1,0);
    glVertex3f(1,-1,-1);
    glEnd();
    glFlush();
    glPopMatrix();

    glLoadIdentity();
    sf::Texture::bind(&tt);
    glBegin(GL_QUADS);
    glColor3f(r_r,r_g,r_b);
    glVertex3f(10,-10,10);
    glTexCoord2d(0,0);
    glVertex3f(10,-10,-10);
    glTexCoord2d(0,1);
    glVertex3f(-10,-10,-10);
    glTexCoord2d(1,1);
    glVertex3f(-10,-10,10);
    glTexCoord2d(1,0);
    glVertex3f(10,10,10);
    glTexCoord2d(0,0);
    glVertex3f(10,10,-10);
    glTexCoord2d(0,1);
    glVertex3f(-10,10,-10);
    glTexCoord2d(1,1);
    glVertex3f(-10,10,10);
    glTexCoord2d(1,0);
    glVertex3f(10,10,10);
    glTexCoord2d(0,0);
    glVertex3f(10,-10,10);
    glTexCoord2d(0,1);
    glVertex3f(-10,-10,10);
    glTexCoord2d(1,1);
    glVertex3f(-10,10,10);
    glTexCoord2d(1,0);
    glVertex3f(10,10,-10);
    glTexCoord2d(0,0);
    glVertex3f(10,-10,-10);
    glTexCoord2d(0,1);
    glVertex3f(-10,-10,-10);
    glTexCoord2d(1,1);
    glVertex3f(-10,10,-10);
    glTexCoord2d(1,0);
    glVertex3f(-10,10,-10);
    glTexCoord2d(0,0);
    glVertex3f(-10,10,10);
    glTexCoord2d(0,1);
    glVertex3f(-10,-10,10);
    glTexCoord2d(1,1);
    glVertex3f(-10,-10,-10);
    glTexCoord2d(1,0);
    glVertex3f(10,10,-10);
    glTexCoord2d(0,0);
    glVertex3f(10,10,10);
    glTexCoord2d(0,1);
    glVertex3f(10,-10,10);
    glTexCoord2d(1,1);
    glVertex3f(10,-10,-10);
    glTexCoord2d(1,0);
    glEnd();
    glFlush();
    glDisable(GL_TEXTURE_2D);
    sekiller();
    glPushMatrix();
    h_body->getMotionState()->getWorldTransform(b_trans);
    b_trans.getOpenGLMatrix(matrix);
    glMultMatrixf(matrix);
    glPopMatrix();
    window.display();
}
delete shape;
delete motionState;
delete collisionCfg;
delete axisSweep;
return 0;
}

static void sekiller(void)
{

    glLoadIdentity();
    glTranslatef(wx,wy,wz);
    glBegin(GL_QUADS);
    glColor3f(0,0,0);
    glVertex3f(0.1,-0.10,0.10);
    glVertex3f(0.1,-0.10,-0.10);
    glVertex3f(-0.10,-0.10,-0.10);
    glVertex3f(-0.10,-0.10,0.10);
    glVertex3f(0.10,0.10,0.10);
    glVertex3f(0.10,0.10,-0.10);
    glVertex3f(-0.10,0.10,-0.10);
    glVertex3f(-0.10,0.10,0.10);
    glVertex3f(0.10,0.10,0.10);
    glVertex3f(0.10,-0.10,0.10);
    glVertex3f(-0.10,-0.10,0.10);
    glVertex3f(-0.10,0.10,0.10);
    glVertex3f(0.10,0.10,-0.10);
    glVertex3f(0.10,-0.10,-0.10);
    glVertex3f(-0.10,-0.10,-0.10);
    glVertex3f(-0.10,0.10,-0.10);
    glVertex3f(-0.10,0.10,-0.10);
    glVertex3f(-0.10,0.10,0.10);
    glVertex3f(-0.10,-0.10,0.10);
    glVertex3f(-0.10,-0.10,-0.10);
    glVertex3f(0.10,0.10,-0.10);
    glVertex3f(0.10,0.10,0.10);
    glVertex3f(0.10,-0.10,0.10);
    glVertex3f(0.10,-0.10,-0.10);
    glEnd();
    glFlush();
}

void tiklanma(sf::Event& event)
{
    if(event.type == sf::Event::MouseButtonPressed)
        if(event.key.code == sf::Mouse::Left)
            tiklandimi = true;
    if(event.type == sf::Event::MouseButtonReleased)
    {
        basildimi = false;
        tiklandimi = false;
    }
    if(event.type == sf::Event::MouseButtonPressed){
        if(event.key.code == sf::Mouse::Right)
            {
                basildimi = true;
            }
    }
}
