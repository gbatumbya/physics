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
#include "iPhysics.h"
#include "CSphere.h"
#include "iSimpleCollisionSpace.h"
#include "iAPIWindow.h"
#include <vector>
#include <ctime>
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
   setAmbientLight(1, 1, 1);
   // camera at a distance - in lhs coordinates
    // camera at a distance - in lhs coordinates
   iCamera* camera = CreateCamera();
   camera->translate(0, 190,-500);
   camera->setRadius(17.8f);
   
    lastUpdate = now;	

    hud = CreateHUD(0.72f, 0.01f, 0.27f, 0.99f, CreateTexture(HUD_IMAGE));
    // cameras ----------------------------------------------------------------

   velocitytxt_=CreateText(Rectf(0.05f,0.27f,0.95f,0.37f),hud,L"",TEXT_HEIGHT,TEXT_TYPEFACE,TEXT_LEFT);
   deltatxt_=CreateText(Rectf(0.05f,0.17f,0.95f,0.27f),hud,L"",TEXT_HEIGHT,TEXT_TYPEFACE,TEXT_LEFT);
   positiontxt_=CreateText(Rectf(0.05f,0.38f,0.95f,0.48f),hud,L"",TEXT_HEIGHT,TEXT_TYPEFACE,TEXT_LEFT);

   lasttextupdate=now;

   // game ----------------------------------------------------------------------
   stretcher = CreatePhysicsBox(-40, -5, 0, 40, 5, 0, &bluish, 1, PHYS_Floating, true);
   iAPIWindow* win = getWindow();
   stretcher->translate(0, 0, 0);

   building = CreateObject(CreateBox(0, 0, 0, 100, 550, 0), &redish);
   building->translate(-400, 200, 0);

   ambulance = CreatePhysicsBox(-100, -20, 0, 100, 20, 0, &whitish, 1, PHYS_Floating, true);
   ambulance->translate(400, -20, 0);

   iPhysics* fallingBox = CreatePhysicsBox(-5, -5, -5, 5, 5, 5, &whitish, 1, PHYS_Falling, true);
   fallingBox->translate(-350, 350, 0);
   fallingBox->setVelocity(Vector(5, 20, 0));
   fallingBox->addBodyForce(Vector(0, -10, 0));
   fallingBox->setCollision(CreateCSphere(fallingBox, 5));
   objects.insert(objects.end(), fallingBox);

   wchar_t str[MAX_DESC + 1];
   StringCbPrintfW(str, MAX_DESC, L"Score: 0");
   velocitytxt_->set(str);

   StringCbPrintfW(str, MAX_DESC, L"Life left: 5"); 
   deltatxt_->set(str);
}

// update updates the position and orientation of each object according to the 
// actions initiated by the user
//
void Design::update() 
{
   bool
      translate = false,
      rotate    = false;

   int 
      delta = now - lastUpdate,
      dX = 0;
   float wZ = 0;
   static int 
      accum = delta,
      next  = 3000,
      score = 0,
      life = 5,
      update = true;
   wchar_t str[MAX_DESC + 1];

   if (!update)
   {
      StringCbPrintfW(str, MAX_DESC, L"GAME OVER"); 
      deltatxt_->set(str);

      StringCbPrintfW(str, MAX_DESC, L"Score: %d", score);
      velocitytxt_->set(str);
      return;
   }

   if (pressed(MOVE_LEFT))
   {
      translate = true;
      dX = -10;
   }
   
   if (pressed(ROTATE_LEFT))
   {
      rotate = true;
      wZ = -0.1f;
   }

   if (pressed(MOVE_RIGHT))
   {
      translate = true;
      dX = 10;
   }

   if (pressed(ROTATE_RIGHT))
   {
      rotate = true;
      wZ = 0.1f;
   }

   if (translate)
      stretcher->translate(dX, 0, 0);
   if (rotate)
      stretcher->rotatez(wZ);


   const CollisionContact* cc;
   cs_->populateContactList(delta/UNITS_PER_SEC);
   int nc=cs_->getNumContacts();
   std::vector<iPhysics*> toRemove; 

   if(nc!=0)
   {
      float vrn, J;
      Vector force,n,relativeVelocity;
      Vector g1deltap, g2deltap;

      for(int i = 1; i < nc ;i++)
      {
         cc = cs_->getContactList(i);
         
         
         iPhysics* g1 = cc->g1->getPhysics();
         iPhysics* g2 = cc->g2->getPhysics();

         if(g1 == stretcher && g2 != stretcher)
         {
            if (g2 == ambulance)
            {
               stretcher->translate(-dX, 0, 0);
               continue;
            }

            n = cc->normal;
            relativeVelocity = -g2->velocity();

            // normal component of the relative velocity
            vrn = dot(relativeVelocity, n);

            // magnitude of the impulse at collision
            J = - vrn * 2.0f / (1.0f / cc->g1->getPhysics()->mass() + 1.0f / cc->g2->getPhysics()->mass());

            // force generated by the impulse
            force = J * n / (float(delta)/float(UNITS_PER_SEC));

            // apply the force to both objects
            cc->g2->getPhysics()->addimpulseForce(-1 * force);
            cc->g2->getPhysics()->setVelocity(g2->velocity() + (-J * n)/ cc->g2->getPhysics()->mass());

            //push objects apart so that it doesn't keep colliding
            float massTotal = (cc->g1->getPhysics()->mass() + cc->g2->getPhysics()->mass()) * 0.85f;
            float rbd1mf = cc->g1->getPhysics()->mass() / massTotal;
            float rbd2mf = cc->g2->getPhysics()->mass() / massTotal;
            Vector g1deltap (-1*(rbd1mf * cc->depth * n));
            Vector g2deltap (rbd2mf * cc->depth * n);

            cc->g2->getPhysics()->translate(g2deltap.x,g2deltap.y,g2deltap.z);
         }

         if (g1 == ambulance && g2 != ambulance)
         {
            if (g2 == stretcher)
            {
               stretcher->translate(-dX, 0, 0);
            }

            toRemove.insert(toRemove.begin(), g2);

            StringCbPrintfW(str, MAX_DESC, L"Score: %d", ++score);
            velocitytxt_->set(str);
         }
      }
   }

   for (int i = toRemove.size() - 1; i >= 0; --i)
   {
      iPhysics* object = toRemove.at(i);
      cs_->remove(object->collisionGeometry());
      remove(object->bv());
      objects.remove(object);
   }

   accum += delta;

   if (accum > next)
   {
      Reflectivity whitish = Reflectivity(white);
      iPhysics* fallingBox = CreatePhysicsBox(-5, -5, -5, 5, 5, 5, &whitish, 1, PHYS_Falling, true);

      fallingBox->translate(-350, 350, 0);
      srand(time(0));
      fallingBox->setVelocity(Vector(rand() % 40, rand() % 30, 0));
      fallingBox->addBodyForce(Vector(0, -10, 0));
      fallingBox->setCollision(CreateCSphere(fallingBox, 5));
      objects.insert(objects.end(), fallingBox);

      accum = 0;
      next = 2000;

      if (next < 0)
      {
         next = 100;
      }
   }

   Vector stretchPos = stretcher->position();
   LIST_iPHYSICS::iterator itr = objects.begin();

   while (itr != objects.end())
   {
      iPhysics* object = *itr;
      Vector position = object->position();

      if (position.y < stretchPos.y)
      {
         if (--life < 0)
         {
            StringCbPrintfW(str, MAX_DESC, L"Your lost "); 
            deltatxt_->set(str);

            update = false;
         }

         cs_->remove(object->collisionGeometry());
         remove(object->bv());

         itr = objects.erase(itr);
         
         StringCbPrintfW(str, MAX_DESC, L"Life left: %d ", life); 
         deltatxt_->set(str);

         continue;
      }

      (*itr)->update(delta);
      ++itr;
   }
}
