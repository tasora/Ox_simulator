//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

// A very simple example that can be used as template project for 
// a Chrono::Engine simulator with 3D view.
 

#include "physics/ChSystem.h"
#include "physics/ChBodyEasy.h"
#include "physics/ChLinkMate.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "unit_IRRLICHT/ChIrrApp.h"
#include "physics/ChBody.h"


// Use the namespace of Chrono

using namespace chrono;


// Use the main namespaces of Irrlicht

using namespace irr;
using namespace irr::core;
using namespace irr::scene; 
using namespace irr::video;
using namespace irr::io; 
using namespace irr::gui; 



int main(int argc, char* argv[])
{
	// Create a ChronoENGINE physical system
	ChSystem mphysicalSystem;

	// Create the Irrlicht visualization (open the Irrlicht device, 
	// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"Ox_simulator",core::dimension2d<u32>(800,600),false); //screen dimensions

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(2,2,-5), core::vector3df(0,0,0));		//to change the position of camera
    application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));
 

	//======================================================================

	// HERE YOU CAN POPULATE THE PHYSICAL SYSTEM WITH BODIES AND LINKS.
	//

	//    Consistent unit system:
	//    - Force [uN];
	//    - Mass [gr];
	//    - Time [s];
	//    - Length [mm];
	//    - Inertia [gr*mm^2];
	//    - Torsional stiffness [uN*mm/rad];
	//    - Torsional damping [uN*mm*s/rad];
	//    - Torque [uN*mm];
	//    - Angular speed [rad/s].

	double dist_rocker_center = 2.17;
	double rad_escapement = 1.8;
	double thickness = 0.15;
	double ang_speed = (CH_C_PI)* 2;

	// 1-Create the truss  

	ChSharedPtr<ChBodyEasyBox> trussBody(new ChBodyEasyBox(6,7,0.01, 1, false, true));		//to create the floor, false -> doesn't represent a collide's surface
	trussBody->SetPos(ChVector<>(0, 0, thickness));
	trussBody->SetBodyFixed(true);

	mphysicalSystem.Add(trussBody);


	// 2-Create the escapement wheel 

	ChSharedPtr<ChBody> escapementBody(new ChBody());		//to create the floor, false -> doesn't represent a collide's surface
	escapementBody->SetPos(ChVector<>(0, 0, 0));
	//escapementBody->SetWvel_loc(ChVector<>(0, 0, -ang_speed)); // for example
	escapementBody->Set_Scr_torque(ChVector<>(0, 0, -37.91)); // constant torque to eascape wheel [uN*mm]

	escapementBody->SetMass(1); // to set
	escapementBody->SetInertiaXX( ChVector<>(1, 1, 0.00185) ); // to set: Jzz in [gr*mm^2]

	// Collision shape of the escapementBody

	escapementBody->GetCollisionModel()->ClearModel();
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> > 
	{
	{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 }, 
	{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(CH_C_2PI/30, ChVector<>(0,0,1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(2*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(3*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(4*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(5*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(6*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(7*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(8*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(9*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(10*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(11*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(12*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(13*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(14*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(15*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(16*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(17*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(18*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(19*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(20*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(21*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(22*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(23*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(24*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(25*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(26*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(27*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(28*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{-1.7012, -0.0374, -thickness / 2}, { -1.8998, 0.0296, -thickness / 2 }, { -1.8994, 0.0486, -thickness / 2 }, { -1.8990, 0.0439, -thickness / 2 }, { -1.8983, 0.0495, -thickness / 2 }, { -1.7077, 0.0273, -thickness / 2 },
		{ -1.7012, -0.0374, thickness / 2 }, { -1.8998, 0.0296, thickness / 2 }, { -1.8994, 0.0486, thickness / 2 }, { -1.8990, 0.0439, thickness / 2 }, { -1.8983, 0.0495, thickness / 2 }, { -1.7077, 0.0273, thickness / 2 }
	}, ChVector<>(0, 0, 0), (ChMatrix33<>(29*CH_C_2PI / 30, ChVector<>(0, 0, 1))));
	escapementBody->GetCollisionModel()->BuildModel();
	escapementBody->SetCollide(true);

	// optional visualization
	ChSharedPtr<ChCylinderShape> myvisual_cylinder( new ChCylinderShape);
	myvisual_cylinder->GetCylinderGeometry().rad = rad_escapement;
	myvisual_cylinder->GetCylinderGeometry().p1 = ChVector<>(0,0,-thickness/2);
	myvisual_cylinder->GetCylinderGeometry().p2 = ChVector<>(0,0, thickness/2);
	escapementBody->AddAsset(myvisual_cylinder); 

	mphysicalSystem.Add(escapementBody);


	// 3-Create a rocker 

	ChSharedPtr<ChBody> rockerBody(new ChBody());		//to create the floor, false -> doesn't represent a collide's surface
	rockerBody->SetPos(ChVector<>(0, dist_rocker_center, 0));
	rockerBody->SetWvel_loc(ChVector<>(0, 0, -50)); // for example

	rockerBody->SetMass(1); // to set
	rockerBody->SetInertiaXX(ChVector<>(1, 1, 1.9837)); // to set: Jzz in [gr*mm^2]

	// Collision shape of the rockerBody

	rockerBody->GetCollisionModel()->ClearModel();
	rockerBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> >
	{
		{ -0.9853, -0.5631, -thickness / 2}, { -1.0787, -0.5997, -thickness / 2 }, { -1.0885, -0.5816, -thickness / 2 }, { -1.0986, -0.5623, -thickness / 2 }, { -1.1079, -0.5438, -thickness / 2 },
		{ -1.1169, -0.5250, -thickness / 2 }, { -1.1257, -0.5060, -thickness / 2 }, { -1.1342, -0.4867, -thickness / 2 }, { -1.1423, -0.4671, -thickness / 2 }, { -1.1502, -0.4473, -thickness / 2 },
		{ -1.1579, -0.4272, -thickness / 2 }, { -1.1652, -0.4069, -thickness / 2 }, { -1.1722, -0.3863, -thickness / 2 }, { -1.1788, -0.3654, -thickness / 2 }, { -1.1863, -0.3402, -thickness / 2 },
		{ -0.9853, -0.5631, thickness / 2 }, { -1.0787, -0.5997, thickness / 2 }, { -1.0885, -0.5816, thickness / 2 }, { -1.0986, -0.5623, thickness / 2 }, { -1.1079, -0.5438, thickness / 2 },
		{ -1.1169, -0.5250, thickness / 2 }, { -1.1257, -0.5060, thickness / 2 }, { -1.1342, -0.4867, thickness / 2 }, { -1.1423, -0.4671, thickness / 2 }, { -1.1502, -0.4473, thickness / 2 },
		{ -1.1579, -0.4272, thickness / 2 }, { -1.1652, -0.4069, thickness / 2 }, { -1.1722, -0.3863, thickness / 2 }, { -1.1788, -0.3654, thickness / 2 }, { -1.1863, -0.3402, thickness / 2 },
	}, ChVector<>(0, 0, 0));
		rockerBody->GetCollisionModel()->BuildModel();
		rockerBody->SetCollide(true);

	//rockerBody->GetCollisionModel()->ClearModel();
	//rockerBody->GetCollisionModel()->AddBox(1, 0.2, thickness, ChVector<>(0, 0, 0));
	//rockerBody->GetCollisionModel()->BuildModel();
	//rockerBody->SetCollide(true);

	// optional visualization
	ChSharedPtr<ChBoxShape> myvisual_box(new ChBoxShape);
	myvisual_box->GetBoxGeometry().Size = ChVector<>(1, 0.2, thickness);
	rockerBody->AddAsset(myvisual_box);

	//GetLog() << "Mass of the rocker:" << rockerBody->GetMass() << " \n";

	mphysicalSystem.Add(rockerBody);


	// 4- create constraint

	ChSharedPtr<ChLinkLockRevolute> escapementLink(new ChLinkLockRevolute()); 

	ChCoordsys<> link_position_abs1(ChVector<>(0,0,0)); 

	escapementLink->Initialize(trussBody, escapementBody, link_position_abs1);		// the link reference attached to 1st body

	mphysicalSystem.Add(escapementLink);


	// 5- create constraint

	ChSharedPtr<ChLinkLockRevolute> rockerLink(new ChLinkLockRevolute()); 

	ChCoordsys<> link_position_abs2(ChVector<>(0, dist_rocker_center, 0));

	rockerLink->Initialize(trussBody, rockerBody, link_position_abs2);		// the link reference attached to 2nd body

	rockerLink->GetForce_Rz()->Set_active(true);
	rockerLink->GetForce_Rz()->Set_K(73600); // Torsional stiffness, to set, in [uN*mm/rad]
	rockerLink->GetForce_Rz()->Set_active(true);
	rockerLink->GetForce_Rz()->Set_R(0); // Torsional damping, to set, in [uN*mm*s/rad]

	mphysicalSystem.Add(rockerLink);


	// optional, attach a RGB color asset to the trussBody, for better visualization	
	ChSharedPtr<ChColorAsset> acolor(new ChColorAsset());
	acolor->SetColor(ChColor(0.55, 0.1, 0));
	trussBody->AddAsset(acolor);

	// optional, attach a RGB color asset to the rockerBody, for better visualization	
	ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset());
	mcolor->SetColor(ChColor(0.22, 0.25, 0.25));
	rockerBody->AddAsset(mcolor);

	// optional, attach a texture to the escapementBody, for better visualization
	ChSharedPtr<ChTexture> mtexture(new ChTexture());
    mtexture->SetTextureFilename(GetChronoDataFile("escape_wheel.png"));		//texture in ../data
	escapementBody->AddAsset(mtexture);		


	//======================================================================
	

	// Use this function for adding a ChIrrNodeAsset to all items
	// Otherwise use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();

	// Use this function for 'converting' assets into Irrlicht meshes 
	application.AssetUpdateAll();


	// Adjust some settings:
	application.SetTimestep(0.001);
	application.SetTryRealtime(false);

	mphysicalSystem.SetIterLCPmaxItersSpeed(100);
	mphysicalSystem.SetLcpSolverType(ChSystem::eCh_lcpSolver::LCP_ITERATIVE_BARZILAIBORWEIN); // or: LCP_ITERATIVE_APGD or: LCP_ITERATIVE_SOR (for speed)
	
	//mphysicalSystem.SetIntegrationType(ChSystem::eCh_integrationType::INT_ANITESCU); // in future: INT_HHT or other


	// 
	// THE SOFT-REAL-TIME CYCLE
	//

	ChStreamOutAsciiFile result_rocker("output_rocker.txt");

	while (application.GetDevice()->run())
	{
		application.BeginScene();

		application.DrawAll();

		// This performs the integration timestep!
		application.DoStep();
		
		// Save results:
		result_rocker << rockerBody->GetWvel_loc().z << " " << rockerBody->GetWacc_loc().z << "\n";
		
		application.EndScene();

		if (mphysicalSystem.GetChTime() > 20) 
			break;

	}


	return 0;
}
  
