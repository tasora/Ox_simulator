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
	ChIrrApp application(&mphysicalSystem, L"A simple project template",core::dimension2d<u32>(800,600),false); //screen dimensions

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(2,2,-5), core::vector3df(0,1,0));		//to change the position of camera
	//application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));
 

	//======================================================================

	// HERE YOU CAN POPULATE THE PHYSICAL SYSTEM WITH BODIES AND LINKS.
	//

	double dist_rocker_center = 2.1;
	double rad_escapement = 1.8;
	double thickness = 0.15;

	// 1-Create the truss  

	ChSharedPtr<ChBody> trussBody(new ChBody());		//to create the floor, false -> doesn't represent a collide's surface
	trussBody->SetBodyFixed(true);

	mphysicalSystem.Add(trussBody);


	// 2-Create the escapement wheel 

	ChSharedPtr<ChBody> escapementBody(new ChBody());		//to create the floor, false -> doesn't represent a collide's surface
	escapementBody->SetPos( ChVector<>(0,0,0) );	
	escapementBody->SetWvel_loc( ChVector<>(0,0,1) ); // for example

	escapementBody->SetMass(0.1); // to set
	escapementBody->SetInertiaXX( ChVector<>(1,1,1) ); // to set

	  // optional visualization
	ChSharedPtr<ChCylinderShape> myvisual_cylinder( new ChCylinderShape);
	myvisual_cylinder->GetCylinderGeometry().rad = rad_escapement;
	myvisual_cylinder->GetCylinderGeometry().p1 = ChVector<>(0,0,-thickness/2);
	myvisual_cylinder->GetCylinderGeometry().p2 = ChVector<>(0,0, thickness/2);
	escapementBody->AddAsset(myvisual_cylinder);

	mphysicalSystem.Add(escapementBody);


	// 3-Create a rocker 

	ChSharedPtr<ChBodyEasyBox> rockerBody(new ChBodyEasyBox( 0.5, 1.3, thickness, 2330, false, true));		//to create the floor, false -> doesn't represent a collide's surface
	rockerBody->SetPos( ChVector<>(dist_rocker_center,0,0) );	
	rockerBody->SetWvel_loc( ChVector<>(0,0,1.2) ); // for example

	GetLog() << "Mass of the rocker:" << rockerBody->GetMass() << " \n";

	mphysicalSystem.Add(rockerBody);


	// 4- create constraint

	ChSharedPtr<ChLinkLockRevolute> escapementLink(new ChLinkLockRevolute()); 

	ChCoordsys<> link_position_abs1(ChVector<>(0,0,0)); 

	escapementLink->Initialize(trussBody, escapementBody, link_position_abs1);		// the link reference attached to 2nd body

	mphysicalSystem.Add(escapementLink);


	// 5- create constraint

	ChSharedPtr<ChLinkLockRevolute> rockerLink(new ChLinkLockRevolute()); 

	ChCoordsys<> link_position_abs2(ChVector<>(dist_rocker_center,0,0)); 

	rockerLink->Initialize(trussBody, rockerBody, link_position_abs2);		// the link reference attached to 2nd body

	rockerLink->GetForce_Rz()->Set_active(true);
	rockerLink->GetForce_Rz()->Set_K(1002);

	mphysicalSystem.Add(rockerLink);



	// optional, attach a RGB color asset to the floor, for better visualization
	ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset());
    mcolor->SetColor(ChColor(0.22, 0.25, 0.25));
	rockerBody->AddAsset(mcolor);		

	// optional, attach a texture to the pendulum, for better visualization
	ChSharedPtr<ChTexture> mtexture(new ChTexture());
    mtexture->SetTextureFilename(GetChronoDataFile("cubetexture_bluwhite.png"));		//texture in ../data
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

		if (mphysicalSystem.GetChTime() > 2) 
			break;

	}


	return 0;
}
  
