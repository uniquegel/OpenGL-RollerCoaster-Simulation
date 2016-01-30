/*
 CSCI 480
 Assignment 2
 */

#include <stdio.h>
#include <stdlib.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include "pic.h"
#include "Eigen/Eigen/Core"
#include "Eigen/Eigen/Dense"
#include <iostream>


using Eigen::MatrixXd;
using Eigen::Vector3d;
using namespace std;

double uValue;
double trackVertex[100000][3];
double trackVertexTangentNormal[100000][3];
double trackVertexTangentBiNormal[100000][3];
double trackLookAtCenterVertex[100000][3];
double pointLeftVector[100000][3];
double pointRightVector[100000][3];
double scaleMultiplier = 20;
double unscaleMultiplier;

bool enableScreenshot = false;
bool debugEnabled = false;
bool birdsViewEnabled = false;
bool translatedOnce = birdsViewEnabled;

int carSpeed = 10;
int trackPolygonMode = 1;
int arraySizeMarker = 0;
int animationCounter = 0;
int ssCounter = 0;

MatrixXd m(4,4);
MatrixXd u(1,4);
MatrixXd c(4,3);

GLuint texture[1];
GLuint skyTexture[1];

char skyTexName[] = "skytex.jpg";
char groundTexName[] = "gtex.jpg";


//Custom Structs
struct Vector3D
{
    int x;
    int y;
    int z;
};

struct BoundingBox
{
    struct Vector3D max;
    struct Vector3D min;
};
BoundingBox *box_mo;
BoundingBox *static_box[400];


//Old Defines from Assign1

int g_iMenuId;
int count;

int g_vMousePos[2] = {0, 0};
int g_iLeftMouseButton = 0;    /* 1 if pressed, 0 if not */
int g_iMiddleMouseButton = 0;
int g_iRightMouseButton = 0;

typedef enum { ROTATE, TRANSLATE, SCALE } CONTROLSTATE;

CONTROLSTATE g_ControlState = TRANSLATE;

/* state of the world */
float g_vLandRotate[3] = {0.0, 0.0, 0.0};
float g_vLandTranslate[3] = {0.0, 0.0, 0.0};
float g_vLandScale[3] = {1.0, 1.0, 1.0};

/* see <your pic directory>/pic.h for type Pic */
Pic * g_pHeightData;

/* represents one control point along the spline */
struct point {
    double x;
    double y;
    double z;
};

/* spline struct which contains how many control points, and an array of control points */
struct spline {
    int numControlPoints;
    struct point *points;
};

/* the spline array */
struct spline *g_Splines;

/* total number of splines */
int g_iNumOfSplines;





void texload(int i,char *filename) {
    Pic* img;
    img = jpeg_read(filename, NULL);
    glBindTexture(GL_TEXTURE_2D, texture[i]);
    
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGB,
                 img->nx,
                 img->ny,
                 0,
                 GL_RGB,
                 GL_UNSIGNED_BYTE,
                 &img->pix[0]);
    
    pic_free(img);
}


/************************************************/
/* implementation of old functions from ASSIGN1 */
void drawHeightField() {
    glTranslatef(-100,-100,-100);
    glScalef(0.78125,0.78125,0.78125);
    glEnable(GL_TEXTURE_2D);
    glColor4f(1.0,1.0,1.0,0.0);
    texload(0, groundTexName);
    
    glBindTexture(GL_TEXTURE_2D, texture[0]);
    glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    
    for(int i=0;i<g_pHeightData->ny;i++){
        //initalize drawing
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glBegin(GL_TRIANGLE_STRIP);
        //second loop, the row loop
        for(int j=0;j<g_pHeightData->nx;j++) {
            glTexCoord2f(j/255.0, i/255.0);
            glVertex3f(j, i, PIC_PIXEL(g_pHeightData, j, i, 0)/5);//draw first vertex, the bottom vertext
            glTexCoord2f(j/255.0, (i+1)/255.0);
            glVertex3f(j, i+1, PIC_PIXEL(g_pHeightData, j, i+1, 0)/5);//second vertex, the top vertex
        }
        glEnd();
    }
    glDisable(GL_TEXTURE_2D);
    glScalef(1.28,1.28,1.28);
    glTranslatef(100,100,100);
    
    //when either lefr or middle mouse button is pressed
    if (g_iLeftMouseButton || g_iMiddleMouseButton) {
        //determine transformation based on the control state
        switch (g_ControlState)
        {
            case TRANSLATE:{
                printf ("Translating! z:%f  \n", g_vLandTranslate[2]);
                glTranslatef(g_vLandTranslate[0]*20, g_vLandTranslate[1]*20, g_vLandTranslate[2]*20 );
                break;
            }
            case ROTATE:{
                printf ("Rotating! z:%f  \n", g_vLandRotate[2]);
                glTranslatef(g_pHeightData->nx/2,g_pHeightData->ny/2,0);
                glRotatef(g_vLandRotate[0], 1.0, 0.0, 0.0);
                glRotatef(g_vLandRotate[1], 0.0, 1.0, 0.0);
                glRotatef(g_vLandRotate[2], 0.0, 0.0, 1.0);
                glTranslatef(-g_pHeightData->nx/2,-g_pHeightData->ny/2,0);
                break;
            }
            case SCALE:{
                printf ("Scaling! z:%f  \n", g_vLandScale[2]);
                glTranslatef(g_pHeightData->nx/2,g_pHeightData->ny/2,0);
                glScalef(g_vLandScale[0], g_vLandScale[1], g_vLandScale[2]);
                glTranslatef(-g_pHeightData->nx/2,-g_pHeightData->ny/2,0);
                break;
            }
        }
        //after each transformation, clear the arrays
        for (int i=0; i<4; i++) {
            g_vLandRotate[i] = 0.0;
            g_vLandTranslate[i] = 0.0;
            g_vLandScale[i] = 1.0;
        }
    }
}

void saveScreenshot (char *filename)
{
    int i, j;
    Pic *in = NULL;
    
    if (filename == NULL)
        return;
    
    /* Allocate a picture buffer */
    in = pic_alloc(500, 500, 3, NULL);
    
    printf("File to save to: %s\n", filename);
    
    for (i=499; i>=0; i--) {
        glReadPixels(0, 500-i, 500, 1, GL_RGB, GL_UNSIGNED_BYTE,
                     &in->pix[i*in->nx*in->bpp]);
    }
    
    if (jpeg_write(filename, in))
        printf("File saved Successfully\n");
    else
        printf("Error in Saving\n");
    
    pic_free(in);
}

void menufunc(int value)
{
    switch (value)
    {
        case 0:
            exit(0);
            break;
    }
}

void mousedrag(int x, int y)
{
    int vMouseDelta[2] = {x-g_vMousePos[0], y-g_vMousePos[1]};
    switch (g_ControlState)
    {
        case TRANSLATE:{
            if (g_iLeftMouseButton)
            {
                g_vLandTranslate[0] += vMouseDelta[0]*0.01;
                g_vLandTranslate[1] -= vMouseDelta[1]*0.01;
            }
            if (g_iMiddleMouseButton)
            {
                g_vLandTranslate[2] += vMouseDelta[1]*0.01;
            }
            
            break;
        }
        case ROTATE:
            if (g_iLeftMouseButton)
            {
                g_vLandRotate[0] += vMouseDelta[1];
                g_vLandRotate[1] += vMouseDelta[0];
            }
            if (g_iMiddleMouseButton)
            {
                g_vLandRotate[2] += vMouseDelta[1];
            }
            
            break;
        case SCALE:
            if (g_iLeftMouseButton)
            {
                g_vLandScale[0] *= 1.0+vMouseDelta[0]*0.01;
                g_vLandScale[1] *= 1.0-vMouseDelta[1]*0.01;
            }
            if (g_iMiddleMouseButton)
            {
                g_vLandScale[2] *= 1.0-vMouseDelta[1]*0.01;
            }
            
            break;
    }
    g_vMousePos[0] = x;
    g_vMousePos[1] = y;
}


void mouseidle(int x, int y)
{
    g_vMousePos[0] = x;
    g_vMousePos[1] = y;
}

void mousebutton(int button, int state, int x, int y)
{
    switch (button)
    {
        case GLUT_LEFT_BUTTON:
            g_iLeftMouseButton = (state==GLUT_DOWN);
            break;
        case GLUT_MIDDLE_BUTTON:
            g_iMiddleMouseButton = (state==GLUT_DOWN);
            break;
        case GLUT_RIGHT_BUTTON:
            g_iRightMouseButton = (state==GLUT_DOWN);
            break;
    }
    
    switch(glutGetModifiers())
    {
        case GLUT_ACTIVE_CTRL:
            g_ControlState = TRANSLATE;
            break;
        case GLUT_ACTIVE_SHIFT:
            g_ControlState = SCALE;
            break;
        default:
            g_ControlState = ROTATE;
            break;
    }
    g_vMousePos[0] = x;
    g_vMousePos[1] = y;
}

void MyKeyboardFunc(unsigned char Key, int x, int y)
{
    switch(Key)
    {
        case 'z':  trackPolygonMode = 0; break;
        case 'x':  trackPolygonMode = 1; break;
        case 'c':  trackPolygonMode = 2; break;
        case 'b':  birdsViewEnabled = true; break;
        case 'r': {
            translatedOnce = false;
            birdsViewEnabled = false;
            break;
        }
        default: break;
    };
}

/************************************************/

int loadSplines(char *argv) {
    char *cName = (char *)malloc(128 * sizeof(char));
    FILE *fileList;
    FILE *fileSpline;
    int iType, i = 0, j, iLength;
    
    
    /* load the track file */
    fileList = fopen(argv, "r");
    if (fileList == NULL) {
        printf ("can't open file\n");
        exit(1);
    }
    
    /* stores the number of splines in a global variable */
    fscanf(fileList, "%d", &g_iNumOfSplines);
    
    g_Splines = (struct spline *)malloc(g_iNumOfSplines * sizeof(struct spline));
    
    /* reads through the spline files */
    for (j = 0; j < g_iNumOfSplines; j++) {
        i = 0;
        fscanf(fileList, "%s", cName);
        fileSpline = fopen(cName, "r");
        
        if (fileSpline == NULL) {
            printf ("can't open file\n");
            exit(1);
        }
        
        /* gets length for spline file */
        fscanf(fileSpline, "%d %d", &iLength, &iType);
        
        /* allocate memory for all the points */
        g_Splines[j].points = (struct point *)malloc(iLength * sizeof(struct point));
        g_Splines[j].numControlPoints = iLength;
        
        /* saves the data to the struct */
        while (fscanf(fileSpline, "%lf %lf %lf",
                      &g_Splines[j].points[i].x,
                      &g_Splines[j].points[i].y,
                      &g_Splines[j].points[i].z) != EOF) {
            i++;
        }
    }
    
    free(cName);
    
    return 0;
}


void drawXYZ() {
    glScaled(1000,1000,1000);
    
    glBegin(GL_LINES);
    // draw line for x axis
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(1.0, 0.0, 0.0);
    // draw line for y axis
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 1.0, 0.0);
    // draw line for Z axis
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 1.0);
    glEnd();
    glScaled(0.001,0.001,0.001);
}

void reshape(int w,int h)
{
    glViewport(0,0,(GLsizei)w,(GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    if(w<=h)
        glOrtho(-50.0,50.0,-50.0*(GLfloat)h/(GLfloat)w,50.0*(GLfloat)h/(GLfloat)w,-50.0,50.0);
    else
        glOrtho(-50.0,5.0,-50.0*(GLfloat)w/(GLfloat)h,50.0*(GLfloat)w/(GLfloat)h,-50.0,50.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
}

void drawBox(BoundingBox *b){
    glColor3f(1,1,1);
    int max = 100;
    int min = -100;
    glBegin(GL_LINE_LOOP);
    glVertex3f(100,100,-100);
    glVertex3f(-100,100,-100);
    glVertex3f(-100,-100,-100);
    glVertex3f(100,-100,-100);
    glEnd();
    
    glBegin(GL_LINE_LOOP);
    glVertex3f(100,-100,100);
    glVertex3f(100,100,100);
    glVertex3f(-100,100,100);
    glVertex3f(-100,-100,100);
    glEnd();
    
    glBegin(GL_LINE_LOOP);
    glVertex3f(100,100,-100);
    glVertex3f(100,100,100);
    glVertex3f(-100,100,100);
    glVertex3f(-100,100,-100);
    glEnd();
    
    glBegin(GL_LINE_LOOP);
    glVertex3f(100,-100,100);
    glVertex3f(-100,-100,100);
    glVertex3f(-100,-100,-100);
    glVertex3f(100,-100,-100);
    glEnd();
}



void skyTexload(int i,char *filename) {
    Pic* img;
    img = jpeg_read(filename, NULL);
    glBindTexture(GL_TEXTURE_2D, skyTexture[0]);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGB,
                 img->nx,
                 img->ny,
                 0,
                 GL_RGB,
                 GL_UNSIGNED_BYTE,
                 &img->pix[0]);
    
    pic_free(img);
}


void clipSkyTexture() {
    
    skyTexload(0, skyTexName);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, skyTexture[0]);
    glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glScaled(100,100,100);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_POLYGON);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(1.0, -1.0, -1.0);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1.0, -1.0, -1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-1.0, -1.0, 1.0);
    
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0, -1.0, 1.0);
    glEnd();
    
    glBegin(GL_POLYGON);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(1.0, 1.0, -1.0);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(1.0, -1.0, -1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(1.0, -1.0, 1.0);
    
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0, 1.0, 1.0);
    glEnd();
    
    glBegin(GL_POLYGON);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(-1.0, 1.0, -1.0);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(1.0, 1.0, -1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(1.0, 1.0, 1.0);
    
    glTexCoord2f(1.0, 1.0);
    glVertex3f(-1.0, 1.0, 1.0);
    glEnd();
    
    glBegin(GL_POLYGON);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(-1.0, -1.0, -1.0);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1.0, 1.0, -1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-1.0, 1.0, 1.0);
    
    glTexCoord2f(1.0, 1.0);
    glVertex3f(-1.0, -1.0, 1.0);
    glEnd();
    
    glBegin(GL_POLYGON);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(-1.0, -1.0, 1.0);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-1.0, 1.0, 1.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(1.0, 1.0, 1.0);
    
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0, -1.0, 1.0);
    glEnd();
    
    glScaled(0.01,0.01,0.01);
    glDisable(GL_TEXTURE_2D);
    
    
}

void getPoints(){
    int splineNumber = 0;
    // glLineWidth(100);
    // printf("no Control POints: %d \n",g_Splines[splineNumber].numControlPoints );
    
    // glScaled(5,5,5);
    for (int i = 0; i < g_Splines[splineNumber].numControlPoints; ++i)
    {
        if (i>=3) {
            c(0,0) = g_Splines[splineNumber].points[i-3].x;
            c(0,1) = g_Splines[splineNumber].points[i-3].y;
            c(0,2) = g_Splines[splineNumber].points[i-3].z;
            c(1,0) = g_Splines[splineNumber].points[i-2].x;
            c(1,1) = g_Splines[splineNumber].points[i-2].y;
            c(1,2) = g_Splines[splineNumber].points[i-2].z;
            c(2,0) = g_Splines[splineNumber].points[i-1].x;
            c(2,1) = g_Splines[splineNumber].points[i-1].y;
            c(2,2) = g_Splines[splineNumber].points[i-1].z;
            c(3,0) = g_Splines[splineNumber].points[i].x;
            c(3,1) = g_Splines[splineNumber].points[i].y;
            c(3,2) = g_Splines[splineNumber].points[i].z;
        }
        else {
            continue;
        }
        
        for (uValue = 0; uValue < 1; uValue+=0.001)
        {
            //DRAW SPLINE P1-P2 connect
            u(0,0) = uValue*uValue*uValue;
            u(0,1) = uValue*uValue;
            u(0,2) = uValue;
            u(0,3) = 1;
            
            MatrixXd newPoint = u*m*c;
            
            trackVertex[(int)((i-3)*1000+1000*uValue)][0] = newPoint(0,0);
            trackVertex[(int)((i-3)*1000+1000*uValue)][1] = newPoint(0,1);
            trackVertex[(int)((i-3)*1000+1000*uValue)][2] = newPoint(0,2);
            
            MatrixXd tangentU(1,4);
            tangentU(0,0) = 3*uValue*uValue;
            tangentU(0,1) = 2*uValue;
            tangentU(0,2) = 1;
            tangentU(0,3) = 0;
            
            MatrixXd tangent(1,3);
            tangent= (tangentU*m*c)/(tangentU*m*c).norm();
            
            MatrixXd centerVec  = newPoint + 2 * tangent;
            trackLookAtCenterVertex[(int)((i-3)*1000+1000*uValue)][0] = centerVec(0,0);
            trackLookAtCenterVertex[(int)((i-3)*1000+1000*uValue)][1]= centerVec(0,1);
            trackLookAtCenterVertex[(int)((i-3)*1000+1000*uValue)][2] = centerVec(0,2);
            
            Vector3d tangentBiNormalPrev;
            Vector3d tangentV = tangent.transpose();
            Vector3d tangentNormal;
            
            Vector3d tangentBiNormal;
            if ((int)(((i-3)*1000+1000*uValue)==0))
            {
                tangentBiNormalPrev = Vector3d(1,1,1);
                tangentNormal = tangentBiNormalPrev.cross(tangentV).normalized();
            }
            else {
                tangentBiNormalPrev(0,0) = trackVertexTangentBiNormal[(int)((i-3)*1000+1000*uValue)-1][0];
                tangentBiNormalPrev(1,0) = trackVertexTangentBiNormal[(int)((i-3)*1000+1000*uValue)-1][1];
                tangentBiNormalPrev(2,0) = trackVertexTangentBiNormal[(int)((i-3)*1000+1000*uValue)-1][2];
                tangentNormal = tangentV.cross(tangentBiNormalPrev).normalized();
            }
            tangentBiNormal = tangentNormal.cross(tangentV).normalized();
            
            trackVertexTangentNormal[(int)((i-3)*1000+1000*uValue)][0] = tangentNormal(0,0);
            trackVertexTangentNormal[(int)((i-3)*1000+1000*uValue)][1] = tangentNormal(1,0);
            trackVertexTangentNormal[(int)((i-3)*1000+1000*uValue)][2] = tangentNormal(2,0);
            trackVertexTangentBiNormal[(int)((i-3)*1000+1000*uValue)][0] = tangentBiNormal(0,0);
            trackVertexTangentBiNormal[(int)((i-3)*1000+1000*uValue)][1] = tangentBiNormal(1,0);
            trackVertexTangentBiNormal[(int)((i-3)*1000+1000*uValue)][2] = tangentBiNormal(2,0);
            
            Vector3d vLeft;
            Vector3d vRight;
            Vector3d newPointVector = newPoint.transpose();
            
            vLeft = newPointVector + .5*(tangentNormal-tangentBiNormal);
            vRight = newPointVector + .5*(-tangentNormal-tangentBiNormal);
            
            pointLeftVector[(int)((i-3)*1000+1000*uValue)][0] = vLeft.x();
            pointLeftVector[(int)((i-3)*1000+1000*uValue)][1] = vLeft.y();
            pointLeftVector[(int)((i-3)*1000+1000*uValue)][2] = vLeft.z();
            pointRightVector[(int)((i-3)*1000+1000*uValue)][0] = vRight.x();
            pointRightVector[(int)((i-3)*1000+1000*uValue)][1] = vRight.y();
            pointRightVector[(int)((i-3)*1000+1000*uValue)][2] = vRight.z();
            
            arraySizeMarker = (int)((i-3)*1000+1000*uValue);
            
            
            if (debugEnabled)
            {
                printf("*************************A NEW POINT************************\n");
                std::cout << "uPoint:  " <<newPoint << "\n"
                << "vLeft:  " <<vLeft.transpose()<< "\n"
                << "vRIght: " <<vRight.transpose() << "\n"
                << "prevTangentBiNormal: " << tangentBiNormalPrev.transpose()    << "\n"
                << "tangent: " << tangent << "\n"
                << "tangentNormal: " << tangentNormal.transpose() << "\n"
                << "tangentBiNormal: " << tangentBiNormal.transpose() << "\n"
                << "At control point:   i=" << i << "  c= " << c(1,0) << ","<<c(1,1)<< "," <<c(1,2)<< " - " << c(2,0)<< ","<<c(2,1)<< "," <<c(2,2) << std::endl;
            }
        }
    }
}


void drawSpline(bool isMiddle) {
    int splineNumber = 0;
    
    for (int i = 0; i < g_Splines[splineNumber].numControlPoints; ++i)
    {
        
        for (uValue = 0; uValue < .999; uValue+=0.001)
        {
            // if ((int)(uValue*1000)%10!=0)
            // {
            //     continue;
            // }
            
            switch(trackPolygonMode)
            {
                case 0:  glPolygonMode(GL_FRONT_AND_BACK, GL_POINT); break;
                case 1:  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); break;
                case 2:  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); break;
                default: break;
            };
            glEnable(GL_POLYGON_SMOOTH);
            glBegin(GL_POLYGON);
            
            glColor3f(1,0,0);
            glVertex3f(pointLeftVector[(int)((i-3)*1000+1000*uValue)][0],
                       pointLeftVector[(int)((i-3)*1000+1000*uValue)][1],
                       pointLeftVector[(int)((i-3)*1000+1000*uValue)][2]);
            
            glColor3f(0,0,1);
            glVertex3f(pointRightVector[(int)((i-3)*1000+1000*uValue)][0],
                       pointRightVector[(int)((i-3)*1000+1000*uValue)][1],
                       pointRightVector[(int)((i-3)*1000+1000*uValue)][2]);
            glColor3f(1,0,0);
            glVertex3f(pointRightVector[(int)((i-3)*1000+1000*uValue)+1][0],
                       pointRightVector[(int)((i-3)*1000+1000*uValue)+1][1],
                       pointRightVector[(int)((i-3)*1000+1000*uValue)+1][2]);
            glColor3f(0,0,1);
            glVertex3f(pointLeftVector[(int)((i-3)*1000+1000*uValue)+1][0],
                       pointLeftVector[(int)((i-3)*1000+1000*uValue)+1][1],
                       pointLeftVector[(int)((i-3)*1000+1000*uValue)+1][2]);
            
            glEnd();
        }
    }
}



void startCamMove(){
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    gluLookAt(trackVertex[animationCounter][0],
              trackVertex[animationCounter][1],
              trackVertex[animationCounter][2],
              trackLookAtCenterVertex[animationCounter][0],
              trackLookAtCenterVertex[animationCounter][1],
              trackLookAtCenterVertex[animationCounter][2],
              trackVertexTangentBiNormal[animationCounter][0],
              trackVertexTangentBiNormal[animationCounter][1],
              trackVertexTangentBiNormal[animationCounter][2]);
}

void doIdle()
{
    /* do some stuff... */
    
    
    
    /* make the screen update */
    glutPostRedisplay();
}


void initScene()
{
    
    glEnable(GL_DEPTH_TEST);
    /* setup gl view here */
    glClearColor(0.0, 0.0, 0.0, 0.0);
    
    //setup view port
    glViewport(0, 0, 500, 500); // use a screen size of WIDTH x HEIGHT
    
    //set matrixmode to be projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //set perspective
    gluPerspective(45, 1.333, 0.01, 10000.0);
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity(); 
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    if (!birdsViewEnabled)
    {
        glTranslatef(0,0,-50);
        startCamMove();
        if (animationCounter!=arraySizeMarker)
        {
            animationCounter+=carSpeed;
        }
        glTranslatef(0,0,50);
    }
    else if (!translatedOnce)
    {
        glTranslatef(0,0,-50);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(0., 0., 1., 0., 0., 0., 0., 1., 0.);
        translatedOnce= true;
    }
    
    
    glTranslatef(0.0,0,-50);
    drawSpline(true);
    glTranslatef(0.0,0,50);
    
    drawHeightField();
    clipSkyTexture();
    
    glutSwapBuffers();
    
    //takes screenshot at every display call
    if (enableScreenshot)
    {
        
        if (ssCounter<1000)
        {
            char myFilenm[2048];
            sprintf(myFilenm, "%04d.jpg", ssCounter);
            saveScreenshot(myFilenm);
            ssCounter++;
        }
    }
}

int main (int argc, char ** argv)
{
    double s = 0.5 ;
    m(0,0) = -s;
    m(0,1) = 2-s;
    m(0,2) = s-2;
    m(0,3) = s;
    m(1,0) = 2*s;
    m(1,1) = s-3;
    m(1,2) = 3-2*s;
    m(1,3) = -s;
    m(2,0) = -s;
    m(2,1) = 0;
    m(2,2) = s;
    m(2,3) = 0 ;
    m(3,0) = 0;
    m(3,1) = 1;
    m(3,2) = 0;
    m(3,3) = 0 ;
    
    if (argc<2)
    {
        printf ("usage: %s <trackfile> <heightmapfile> \n", argv[0]);
        exit(0);
    }
    
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA |  GLUT_DEPTH);
    glutInitWindowSize(500,500);
    glutInitWindowPosition(100,100);
    glutIdleFunc(doIdle);
    glutCreateWindow("ROLLER COASTER");
    initScene();
    
    glGenTextures(1, texture);
    glGenTextures(1, skyTexture);
    loadSplines(argv[1]);
    
    /************************************/
    //from assign1
    g_pHeightData = jpeg_read(argv[2], NULL);
    if (!g_pHeightData)
    {
        printf ("error reading %s.\n", argv[4]);
        exit(1);
    }
    /* callback for mouse drags */
    glutMotionFunc(mousedrag);
    /* callback for idle mouse movement */
    glutPassiveMotionFunc(mouseidle);
    /* callback for mouse button changes */
    glutMouseFunc(mousebutton);
    
    //setup keyboard functions
    glutKeyboardFunc(MyKeyboardFunc);
    /************************************/
    
    getPoints();
    
    glutDisplayFunc(display); 
    // glutReshapeFunc(reshape);
    glutMainLoop();
    
    return 0;
}
