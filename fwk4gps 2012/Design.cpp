/* Design Implementation - Application Layer
 *
 * Design.cpp
 * fwk4gps version 3.0
 * gam666/dps901/gam670/dps905
 * January 14 2012
 * copyright (c) 2012 Chris Szalwinski 
 * distributed under TPL - see ../Licenses.txt
 */

#include "Design.h"          // for the Design class definition
#include "iText.h"           // for the Text Interface
#include "iHUD.h"            // for the HUD Interface
#include "iSound.h"          // for the Sound Interface
#include "iLight.h"          // for the Light Interface
#include "iObject.h"         // for the Object Interface
#include "iTexture.h"        // for the Texture Interface
#include "Camera.h"          // for the Camera Interface
#include "iGraphic.h"        // for the Graphic Interface
#include "iUtilities.h"      // for strcpy()
#include "MathDefinitions.h" // for MODEL_Z_AXIS
#include "ModellingLayer.h"  // for MOUSE_BUTTON_SCALE, ROLL_SPEED
#include "Common_Symbols.h"  // for Action and Sound enumerations
#include "Physics.h"
#include "CSphere.h"
#include "iSimpleCollisionSpace.h"
#include "iAPIWindow.h"

#include <strsafe.h>
const wchar_t* orient(wchar_t*, const iFrame*, char, unsigned = 1u);
const wchar_t* position(wchar_t*, const iFrame*, char = ' ', unsigned = 1u);
const wchar_t* onOff(wchar_t*, const iSwitch*);
//const Colour red(1,0,0);
//-------------------------------- Design -------------------------------------
//
// The Design class implements the game design within the Modelling Layer
//
// constructor initializes the engine and the instance pointers
//
Design::Design(void* h, int s) : Coordinator(h, s) {

   cs_=CreateSimpleCollisionSpace(60);
   CollisionGeometry::setGlobalCollisionSpace(cs_);
}

// initialize initializes the general display design coordinator, creates the 
// primitive sets, textures, objects, lights, sounds, cameras, and text items
//
void Design::initialize() {

       // general display design
    //
   Reflectivity redish = Reflectivity(red);
   Reflectivity greenish = Reflectivity(green);
   Reflectivity bluish = Reflectivity(blue);
   Reflectivity whitish = Reflectivity(white);
   setProjection(0.9f, 1.0f, 1000.0f);
   setAmbientLight(0.9f, 0.9f, 0.9f);
   // camera at a distance - in lhs coordinates
    // camera at a distance - in lhs coordinates
   iCamera* camera = CreateCamera();
   camera->translate(0, 190,-500);
   camera->setRadius(17.8f);
   //camera->rotatex(3.1459/4.0f);
   
    lastUpdate = now;	

    hud = CreateHUD(0.72f, 0.01f, 0.27f, 0.99f, CreateTexture(HUD_IMAGE));
    // cameras ----------------------------------------------------------------

   velocitytxt_=CreateText(Rectf(0.05f,0.27f,0.95f,0.37f),hud,L"",TEXT_HEIGHT,TEXT_TYPEFACE,TEXT_LEFT);
   deltatxt_=CreateText(Rectf(0.05f,0.17f,0.95f,0.27f),hud,L"",TEXT_HEIGHT,TEXT_TYPEFACE,TEXT_LEFT);
   positiontxt_=CreateText(Rectf(0.05f,0.38f,0.95f,0.48f),hud,L"",TEXT_HEIGHT,TEXT_TYPEFACE,TEXT_LEFT);

   lasttextupdate=now;

   // game ----------------------------------------------------------------------
   stretcher = CreatePhysicsBox(-25, -2.5, 0, 25, 2.5, 0, &bluish, 1, PHYS_Floating, true);
   iAPIWindow* win = getWindow();
   stretcher->translate(0, 0, 0);

      iPhysics* fallingBox = CreatePhysicsBox(-5, -5, -5, 5, 5, 5, &whitish, 1, PHYS_Falling, true);
      fallingBox->translate(-150, 350, 0);
      fallingBox->setVelocity(Vector(5, 20, 0));
      fallingBox->addBodyForce(Vector(0, -10, 0));
      fallingBox->setCollision(CreateCSphere(fallingBox, 5));
      objects.insert(objects.end(), fallingBox);
}

// update updates the position and orientation of each object according to the 
// actions initiated by the user
//
void Design::update() 
{
   bool translate = false;
   int 
      delta = now - lastUpdate,
      dX = 0,
      dY = 0;
   static int accum = delta;
   wchar_t str[MAX_DESC + 1];
   Vector position = stretcher->position();
   StringCbPrintfW(str, MAX_DESC, L"Stretcher position X: %.2f, Y: %.2f", position.x, position.y); 
   deltatxt_->set(str);

   if (pressed(MOVE_LEFT))
   {
      translate = true;
      dX = - 10;
   }
   
   if (pressed(ROTATE_LEFT))
   {
      translate = true;
   }

   if (pressed(MOVE_RIGHT))
   {
      translate = true;
      dX = +10;
   }

   if (pressed(ROTATE_RIGHT))
   {
      translate = true;
   }

   if (translate)
      stretcher->translate(dX, dY, position.z);
  // stretcher->update(delta);

   const CollisionContact* cc;
   cs_->populateContactList(delta/UNITS_PER_SEC);
   int nc=cs_->getNumContacts();

   if(nc!=0)
   {
      float vrn, J;
      Vector force,n,relativeVelocity;
      Vector g1deltap, g2deltap;

      for(int i = 1; i < nc ;i++)
      {
         cc = cs_->getContactList(i);
         
         StringCbPrintfW(str, MAX_DESC, L"address g1: %x", cc->g1->getPhysics());
         velocitytxt_->set(str);

         if(cc->g1->getPhysics() == stretcher && cc->g2->getPhysics() != stretcher)
         {
            n = cc->normal;
            Vector v1 = cc->g2->getPhysics()->velocity();
            relativeVelocity =  - cc->g2->getPhysics()->velocity();

            // normal component of the relative velocity
            vrn = dot(relativeVelocity, n);

            // magnitude of the impulse at collision
            J = - vrn * 2.0f / (1.0f / cc->g1->getPhysics()->mass() + 1.0f / cc->g2->getPhysics()->mass());


            // force generated by the impulse
            force = J * n / (float(delta)/float(UNITS_PER_SEC));

            // apply the force to both objects

            cc->g2->getPhysics()->addimpulseForce(-1 * force);

            //push objects apart so that it doesn't keep colliding
            
            Vector v2 = v1 + (-J * n)/ cc->g2->getPhysics()->mass();
            cc->g2->getPhysics()->setVelocity(v2);

            //push objects apart so that it doesn't keep colliding
            float massTotal = (cc->g1->getPhysics()->mass() + cc->g2->getPhysics()->mass()) * 0.85f;
            float rbd1mf = cc->g1->getPhysics()->mass() / massTotal;
            float rbd2mf = cc->g2->getPhysics()->mass() / massTotal;
            Vector g1deltap (-1*(rbd1mf * cc->depth * n));
            Vector g2deltap (rbd2mf * cc->depth * n);

            cc->g2->getPhysics()->translate(g2deltap.x,g2deltap.y,g2deltap.z);
         }
      }
   }
   accum += delta;
   if (accum > 1500)
   {
      /*Reflectivity whitish = Reflectivity(white);
      iPhysics* fallingBox = CreatePhysicsBox(-5, -5, -5, 5, 5, 5, &whitish, 1, PHYS_Falling, true);
      fallingBox->translate(-150, 350, 0);
      fallingBox->setVelocity(Vector(5, 20, 0));
      fallingBox->addBodyForce(Vector(0, -10, 0));
      fallingBox->setCollision(CreateCSphere(fallingBox, 5));
      objects.insert(objects.end(), fallingBox);*/
      accum = 0;
   }

   for (LIST_iPHYSICS::iterator itr = objects.begin(); itr != objects.end(); ++itr)
   {
      (*itr)->update(delta);
   }
}
