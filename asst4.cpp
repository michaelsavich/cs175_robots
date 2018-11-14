////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
//   Solution by Tom Kelliher
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#if __GNUG__
#   include <tr1/memory>
#endif

#include <GL/glew.h>
#ifdef __APPLE__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "cvec.h"
#include "matrix4.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"
#include "rigtform.h"
#include "arcball.h"
#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"


using namespace std;      // for string, vector, iostream, and other standard C++ stuff
using namespace tr1; // for shared_ptr

// G L O B A L S ///////////////////////////////////////////////////

static const int G_SKY_CAM = 2;
static const int G_ROBOT1 = 0;
static const int G_ROBOT2 = 1;

static const int g_numObjects = 2;

static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;
static int g_activeView = G_SKY_CAM;
//static int g_activeObject = G_ROBOT1;
static bool g_picking = false;

static bool g_skySkyFrame = false; // Sky-sky frame is active when modifying the sky camera

double g_arcballRadius = 0.25 * min(g_windowHeight, g_windowWidth);
Cvec3 g_arcballClick;
double g_arcballScale = 1.0;

static const int PICKING_SHADER = 2; // index of the picking shader is g_shaerFiles
static const int g_numShaders = 3;
static const char * const g_shaderFilesGl2[g_numShaders][2] = {
   {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
   {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"},
   {"./shaders/basic-gl2.vshader", "./shaders/pick-gl2.fshader"}
};
static vector<shared_ptr<ShaderState> > g_shaderStates; // our global shader states

// --------- Geometry

// Macro used to obtain relative offset of a field within a struct
#define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

// A vertex with floating point position and normal
struct VertexPN {
   Cvec3f p, n;

   VertexPN() {}
   VertexPN(float x, float y, float z,
            float nx, float ny, float nz)
      : p(x,y,z), n(nx, ny, nz)
   {}

   // Define copy constructor and assignment operator from GenericVertex so we can
   // use make* functions from geometrymaker.h
   VertexPN(const GenericVertex& v) {
      *this = v;
   }

   VertexPN& operator = (const GenericVertex& v) {
      p = v.pos;
      n = v.normal;
      return *this;
   }
};

struct Geometry {
   GlBufferObject vbo, ibo;
   int vboLen, iboLen;

   Geometry(VertexPN *vtx, unsigned short *idx, int vboLen, int iboLen) {
      this->vboLen = vboLen;
      this->iboLen = iboLen;

      // Now create the VBO and IBO
      glBindBuffer(GL_ARRAY_BUFFER, vbo);
      glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPN) * vboLen, vtx, GL_STATIC_DRAW);

      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * iboLen, idx, GL_STATIC_DRAW);
   }

   void draw(const ShaderState& curSS) {
      // Enable the attributes used by our shader
      safe_glEnableVertexAttribArray(curSS.h_aPosition);
      safe_glEnableVertexAttribArray(curSS.h_aNormal);

      // bind vbo
      glBindBuffer(GL_ARRAY_BUFFER, vbo);
      safe_glVertexAttribPointer(curSS.h_aPosition, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, p));
      safe_glVertexAttribPointer(curSS.h_aNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, n));

      // bind ibo
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

      // draw!
      glDrawElements(GL_TRIANGLES, iboLen, GL_UNSIGNED_SHORT, 0);

      // Disable the attributes used by our shader
      safe_glDisableVertexAttribArray(curSS.h_aPosition);
      safe_glDisableVertexAttribArray(curSS.h_aNormal);
   }
};

typedef SgGeometryShapeNode<Geometry> MyShapeNode;

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode; // used later when you do picking

// Vertex buffer and index buffer associated with the ground and cubes geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere;

// --------- Scene

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space

///////////////// END OF G L O B A L S //////////////////////////////////////////////////




static void initGround() {
   // A x-z plane at y = g_groundY of dimension [-g_groundSize, g_groundSize]^2
   VertexPN vtx[4] = {
      VertexPN(-g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
      VertexPN(-g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
      VertexPN( g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
      VertexPN( g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
   };
   unsigned short idx[] = {0, 1, 2, 0, 2, 3};
   g_ground.reset(new Geometry(&vtx[0], &idx[0], 4, 6));
}

static void initCubes() {
   int ibLen, vbLen;
   getCubeVbIbLen(vbLen, ibLen);

   // Temporary storage for cube geometry
   vector<VertexPN> vtx(vbLen);
   vector<unsigned short> idx(ibLen);

   makeCube(1, vtx.begin(), idx.begin());
   g_cube.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
   float radius = 1.0;
   const int slices = 20;
   const int stacks = 10;
   int ibLen, vbLen;
   getSphereVbIbLen(slices, stacks, vbLen, ibLen);
   vector<VertexPN> vtx(vbLen);
   vector<unsigned short> idx(ibLen);
   makeSphere(radius, slices, stacks, vtx.begin(), idx.begin());
   g_sphere.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState& curSS, const Matrix4& projMatrix) {
   GLfloat glmatrix[16];
   projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
   safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
   if (g_windowWidth >= g_windowHeight)
      g_frustFovY = g_frustMinFov;
   else {
      const double RAD_PER_DEG = 0.5 * CS175_PI/180;
      g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
   }
}

static Matrix4 makeProjectionMatrix() {
   return Matrix4::makeProjection(
                                  g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
                                  g_frustNear, g_frustFar);
}

inline RigTForm getEyeRbt()
{
	return g_skyNode->getRbt();

   // TODO: Add code to implement view switching, using getPathAccumRbt().
}

inline RigTForm getArcballRbt() {
	if (g_currentPickedRbtNode == g_skyNode && !g_skySkyFrame)
		return RigTForm::identity();
	else
		return getPathAccumRbt(g_world, g_currentPickedRbtNode, 0);
}

// return the normalized vector corresponding to the virtual click point
// on the arcball.  the vector is from the arcball's origin to the click point.
static Cvec3 findAbClick(int x, int y) {
   // Find arcball origin and then the origin's coordinates in screen space
   Cvec4 p = rigTFormToMatrix(inv(getEyeRbt()) * getArcballRbt()) * Cvec4(0,0,0,1);
   Cvec2 ss = getScreenSpaceCoord(Cvec3(p[0], p[1], p[2]), makeProjectionMatrix(), g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);
   // determine x and y components of vector from origin to click point
   double abx = g_mouseClickX - ss[0];
   double aby = g_mouseClickY - ss[1];
   // determine z component; clamp to 0, if necessary
   double abz = g_arcballRadius * g_arcballRadius - abx * abx - aby * aby;
   if (abz <= 0.0)
      abz = 0.0;
   else
      abz = std::sqrt(abz);

   return normalize(Cvec3(abx, aby, abz));
}

static bool activeObject(int objectID)
{
   switch (objectID) {
   case G_SKY_CAM:
      return g_currentPickedRbtNode == g_skyNode;
      break;
   case G_ROBOT1:
      return checkForDescendant(g_robot1Node, g_currentPickedRbtNode);
      /*return g_currentPickedRbtNode == g_robot1Node || 
        getPathAccumRbt(g_robot1Node, g_currentPickedRbtNode, 0) == RigTForm::identity();*/
      break;
   case G_ROBOT2:
	   return checkForDescendant(g_robot2Node, g_currentPickedRbtNode);
   default:
	   cout << "Whoops --- shouldn't have reached this code" << endl;
	   assert(0);
   }
   return false;  // Make the code checker happy.
}

static bool useArcball() {
   if (activeObject(G_SKY_CAM))
      return g_activeView == G_SKY_CAM && !g_skySkyFrame;
   else if (activeObject(G_ROBOT1))
      return g_activeView != G_ROBOT1;
   else if (activeObject(G_ROBOT2))
      return g_activeView != G_ROBOT2;
   else
      return true;
}

static bool applyM() {
   return useArcball() || 
      (activeObject(g_activeView) &&
       (g_currentPickedRbtNode == g_robot1Node
        || g_currentPickedRbtNode == g_robot2Node)) ||
      (activeObject(G_SKY_CAM) && g_activeView == G_SKY_CAM);
}

static void drawStuff(const ShaderState& curSS, bool picking) {

   // build & send proj. matrix to vshader
   const Matrix4 projmat = makeProjectionMatrix();
   sendProjectionMatrix(curSS, projmat);


   // use the skyRbt as the eyeRbt
   const RigTForm eyeRbt = getEyeRbt();
   const RigTForm invEyeRbt = inv(eyeRbt);

   const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
   const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
   safe_glUniform3f(curSS.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
   safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);

   if (!picking) {
      Drawer drawer(invEyeRbt, curSS);
      g_world->accept(drawer);

      // draw green wireframe arcball
      if (useArcball()) {
         safe_glUniform3f(curSS.h_uColor, 0.0, 1.0, 0.0);
         // arcball origin in eye coordinates
         Cvec4 p = rigTFormToMatrix(invEyeRbt * getArcballRbt()) * Cvec4(0,0,0,1);
         Matrix4 MVM = rigTFormToMatrix(invEyeRbt * getArcballRbt());
         // Dynamically scale the arcball, unless we're translating in Z
         if (!g_mouseMClickButton && !(g_mouseLClickButton && g_mouseRClickButton))
            g_arcballScale = getScreenToEyeScale(p[2], g_frustFovY, g_windowHeight);
         double scale = g_arcballScale * g_arcballRadius;
         MVM = MVM * Matrix4::makeScale(Cvec3(scale, scale, scale));
         Matrix4 NMVM = normalMatrix(MVM);
         sendModelViewNormalMatrix(curSS, MVM, NMVM);
         glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
         g_sphere->draw(curSS);
         glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      }
   }
   else {
      Picker picker(invEyeRbt, curSS);
      g_world->accept(picker);
      glFlush();
      g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
      if (g_currentPickedRbtNode == g_groundNode || g_currentPickedRbtNode == shared_ptr<SgRbtNode>())
         g_currentPickedRbtNode = g_skyNode; //shared_ptr<SgRbtNode>();   // set to NULL
   }
}

static void display() {
   glUseProgram(g_shaderStates[g_activeShader]->program);
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

   drawStuff(*g_shaderStates[g_activeShader], false);

   glutSwapBuffers();                                    // show the back buffer (where we rendered stuff)

   checkGlErrors();
}

static void reshape(const int w, const int h) {
   g_windowWidth = w;
   g_windowHeight = h;
   g_arcballRadius = 0.25 * min(g_windowHeight, g_windowWidth);
   glViewport(0, 0, w, h);
   cerr << "Size of window is now " << w << "x" << h << endl;
   updateFrustFovY();
   glutPostRedisplay();
}

// scaling factor for translations
static inline double getTranslationScale() {
   if (useArcball())
      return g_arcballScale;
   else
      return 0.01;
}

static void motion(const int x, const int y) {
   double dx = x - g_mouseClickX;
   double dy = g_windowHeight - y - 1 - g_mouseClickY;

   RigTForm m;
   if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
      if (g_currentPickedRbtNode == g_skyNode ||
          (g_currentPickedRbtNode == g_robot1Node && g_activeView == G_ROBOT1) ||
          (g_currentPickedRbtNode == g_robot2Node && g_activeView == G_ROBOT2)
          )
      {
         dx = -dx;
         dy = -dy;
      }
      if (useArcball()) {  // compute rotation quaternion for arcball rotation
         // second arcball normalized click coordinate vector.  the first vector
         // was stored in g_arcballClick by the mouse callback, or this callback
         // if motion is called multiple times.
         Cvec3 abClick = findAbClick(x, g_windowHeight - y - 1);
         Quat v1 = Quat(0.0, -g_arcballClick[0], -g_arcballClick[1], -g_arcballClick[2]);
         Quat v2 = Quat(0.0, abClick[0], abClick[1], abClick[2]);
         g_arcballClick = abClick;  // update first arcball click vector
         m = RigTForm(v2 * v1);
         if (g_currentPickedRbtNode == g_skyNode ||
             (g_currentPickedRbtNode == g_robot1Node && g_activeView == G_ROBOT1) ||
             (g_currentPickedRbtNode == g_robot2Node && g_activeView == G_ROBOT2)
             )  // Reverse rotation if modifying sky camera or one of the robots
            m = inv(m);
      }
      else
         m = RigTForm::makeXRotation(-dy) * RigTForm::makeYRotation(dx);
   }
   else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
      if (g_currentPickedRbtNode == g_skyNode && !g_skySkyFrame)
      {
         dx = -dx;
         dy = -dy;
      }
      m = RigTForm::makeTranslation(Cvec3(dx, dy, 0) * getTranslationScale());
   }
   else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
      if (g_currentPickedRbtNode == g_skyNode && !g_skySkyFrame)
      {
         dy = -dy;
      }
      m = RigTForm::makeTranslation(Cvec3(0, 0, -dy) * getTranslationScale());
   }

   if (g_mouseClickDown && applyM()) {

	  // TODO
	  // Working with g_currentPickedRbtNode, add code here to apply transformation m.

      glutPostRedisplay(); // we always redraw if we changed the scene
   }

   g_mouseClickX = x;
   g_mouseClickY = g_windowHeight - y - 1;
}


static void pick() {
  // We need to set the clear color to black, for pick rendering.
  // so let's save the clear color
  GLdouble clearColor[4];
  glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

  glClearColor(0, 0, 0, 0);

  // using PICKING_SHADER as the shader
  glUseProgram(g_shaderStates[PICKING_SHADER]->program);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  drawStuff(*g_shaderStates[PICKING_SHADER], true);

  // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
  // to see result of the pick rendering pass
  //glutSwapBuffers();

  //Now set back the clear color
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

  checkGlErrors();
}


static void mouse(const int button, const int state, const int x, const int y) {
   g_mouseClickX = x;
   g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

   g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
   g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
   g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

   g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
   g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
   g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

   g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;

   if (g_picking && g_mouseLClickButton)
      pick();

   g_picking = false;

   if (g_mouseLClickButton && useArcball())  // store arcball click coordinates
      g_arcballClick = findAbClick(g_mouseClickX, g_mouseClickY);

   glutPostRedisplay();
}


static void keyboard(const unsigned char key, const int x, const int y) {
   switch (key) {
   case 27:
      exit(0);                                  // ESC
   case 'h':
      cout << " ============== H E L P ==============\n\n"
           << "h\t\thelp menu\n"
           << "s\t\tsave screenshot\n"
           << "f\t\tToggle flat shading on/off.\n"
           << "o\t\tCycle object to edit\n"
           << "v\t\tCycle view\n"
           << "drag left mouse to rotate\n" << endl;
      break;
   case 's':
      glFlush();
      writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
      break;
   case 'f':
      g_activeShader ^= 1;
      break;
   case 'v':  // Cycle through the views, one per object, including the sky cam
      g_activeView = (g_activeView + 1) % (g_numObjects + 1);
      switch (g_activeView) {
      case G_SKY_CAM:
         cout << "Active view set to sky camera" << endl;
         break;
      case G_ROBOT1:
         cout << "Active view set to robot 1" << endl;
         break;
      case G_ROBOT2:
         cout << "Active view set to robot 2" << endl;
         break;
      }
      break;
   case 'm':
      if (g_currentPickedRbtNode == g_skyNode && g_activeView == G_SKY_CAM) {
         g_skySkyFrame ^= true;

         switch (g_skySkyFrame) {
         case false:
            cout << "Active sky camera frame is world-sky frame" << endl;
            break;
         case true:
            cout << "Active sky camera frame is sky-sky frame" << endl;
            break;
         }
      }
      break;
   case 'p':
      g_picking = true;
      break;
   }
   glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
   glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
   glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
   glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
   glutCreateWindow("Assignment 2");                       // title the window

   glutDisplayFunc(display);                               // display rendering callback
   glutReshapeFunc(reshape);                               // window reshape callback
   glutMotionFunc(motion);                                 // mouse movement callback
   glutMouseFunc(mouse);                                   // mouse click callback
   glutKeyboardFunc(keyboard);
}

static void initGLState() {
   glClearColor(128./255., 200./255., 255./255., 0.);
   glClearDepth(0.);
   glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
   glPixelStorei(GL_PACK_ALIGNMENT, 1);
   glCullFace(GL_BACK);
   glEnable(GL_CULL_FACE);
   glEnable(GL_DEPTH_TEST);
   glDepthFunc(GL_GREATER);
   glReadBuffer(GL_BACK);
}

static void initShaders() {
   g_shaderStates.resize(g_numShaders);
   for (int i = 0; i < g_numShaders; ++i)
      g_shaderStates[i].reset(new ShaderState(g_shaderFilesGl2[i][0], g_shaderFilesGl2[i][1]));
}

static void initGeometry() {
   initGround();
   initCubes();
   initSphere();
}

static void constructRobot(shared_ptr<SgTransformNode> base, const Cvec3& color) {

   const double ARM_LEN = 0.7,
      ARM_THICK = 0.25,
      TORSO_LEN = 1.5,
      TORSO_THICK = 0.25,
      TORSO_WIDTH = 1;
   const int NUM_JOINTS = 3,
      NUM_SHAPES = 3;

   struct JointDesc {
      int parent;
      float x, y, z;
   };

   JointDesc jointDesc[NUM_JOINTS] = {
      {-1}, // torso
      {0,  TORSO_WIDTH/2, TORSO_LEN/2, 0}, // upper right arm
      {1,  ARM_LEN, 0, 0} // lower right arm
   };

   struct ShapeDesc {
      int parentJointId;
      float x, y, z, sx, sy, sz;
      shared_ptr<Geometry> geometry;
   };

   ShapeDesc shapeDesc[NUM_SHAPES] = {
      {0, 0,         0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube}, // torso
      {1, ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // upper right arm
      {2, ARM_LEN/2, 0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK/2, g_sphere} // lower right arm
   };

   shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

   for (int i = 0; i < NUM_JOINTS; ++i) {
      if (jointDesc[i].parent == -1)
         jointNodes[i] = base;
      else {
         jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
         jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
      }
   }
   for (int i = 0; i < NUM_SHAPES; ++i) {
      shared_ptr<MyShapeNode> shape(
                                    new MyShapeNode(shapeDesc[i].geometry,
                                                    color,
                                                    Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
                                                    Cvec3(0, 0, 0),
                                                    Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
      jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
   }
}

static void initScene() {
   g_world.reset(new SgRootNode());

   g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 8.0))));

   g_groundNode.reset(new SgRbtNode());
   g_groundNode->addChild(shared_ptr<MyShapeNode>(
                                                  new MyShapeNode(g_ground, Cvec3(0.1, 0.95, 0.1))));

   g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
   g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

   constructRobot(g_robot1Node, Cvec3(1, 0, 0)); // a Red robot
   constructRobot(g_robot2Node, Cvec3(0, 0, 1)); // a Blue robot

   g_world->addChild(g_skyNode);
   g_world->addChild(g_groundNode);
   g_world->addChild(g_robot1Node);
   g_world->addChild(g_robot2Node);
   g_currentPickedRbtNode = g_skyNode;  // set to NULL
}

int main(int argc, char * argv[]) {
   try {
      initGlutState(argc,argv);

      glewInit(); // load the OpenGL extensions

      initGLState();
      initShaders();
      initGeometry();
      initScene();

      glutMainLoop();
      return 0;
   }
   catch (const runtime_error& e) {
      cout << "Exception caught: " << e.what() << endl;
      return -1;
   }
}
