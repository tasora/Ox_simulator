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
#include "physics/ChContactContainer.h"

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

	double dist_rocker_center = 2.3365;
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
	escapementBody->Set_Scr_torque(ChVector<>(0, 0, 25*-37.91)); // constant torque to eascape wheel [uN*mm]

	escapementBody->GetMaterialSurface()->SetFriction(0.0);

	escapementBody->SetMass(1); // to set
	escapementBody->SetInertiaXX( ChVector<>(1, 1, 0.00152) ); // to set: Jzz in [gr*mm^2]

	// Collision shape of the escapementBody

	escapementBody->GetCollisionModel()->SetSafeMargin(0.001); 
	escapementBody->GetCollisionModel()->SetEnvelope(0.002);

	escapementBody->GetCollisionModel()->ClearModel();
	int num_escapements = 30;
	for (int i=0; i<num_escapements; ++i)
	{
		escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> > 
		{
			{-0.85503, 1.20112, -thickness / 2}, { -0.74738, 1.28277, -thickness / 2 }, { -0.86842, 1.51932, -thickness / 2 }, { -0.85723, 1.52567, -thickness / 2 }, { -0.85645, 1.52591, -thickness / 2 }, { -0.85561, 1.52582, -thickness / 2 }, { -0.8549, 1.52541, -thickness / 2 }, { -0.85442, 1.52473, -thickness / 2 },
			{ -0.85503, 1.20112, thickness / 2 }, { -0.74738, 1.28277, thickness / 2 }, { -0.86842, 1.51932, thickness / 2 }, { -0.85723, 1.52567, thickness / 2 }, { -0.85645, 1.52591, thickness / 2 }, { -0.85561, 1.52582, thickness / 2 }, { -0.8549, 1.52541, thickness / 2 }, { -0.85442, 1.52473, thickness / 2 }
		}, ChVector<>(0, 0, 0), (ChMatrix33<>(i * CH_C_2PI / num_escapements, ChVector<>(0, 0, 1)) ));
	}
	escapementBody->GetCollisionModel()->BuildModel();
	escapementBody->SetCollide(true);

	// optional visualization
	ChSharedPtr<ChCylinderShape> myvisual_cylinder( new ChCylinderShape);
	myvisual_cylinder->GetCylinderGeometry().rad = rad_escapement;
	myvisual_cylinder->GetCylinderGeometry().p1 = ChVector<>(0,0,-thickness/2);
	myvisual_cylinder->GetCylinderGeometry().p2 = ChVector<>(0,0, thickness/2);
	//escapementBody->AddAsset(myvisual_cylinder); 

	// c.hulls visualization
	for (int i=0; i<num_escapements; ++i)
	{
		std::vector < ChVector<> >   points = 
		{
			{ -0.85503, 1.20112, -thickness / 2 }, { -0.74738, 1.28277, -thickness / 2 }, { -0.86842, 1.51932, -thickness / 2 }, { -0.85723, 1.52567, -thickness / 2 }, { -0.85645, 1.52591, -thickness / 2 }, { -0.85561, 1.52582, -thickness / 2 }, { -0.8549, 1.52541, -thickness / 2 }, { -0.85442, 1.52473, -thickness / 2 },
			{ -0.85503, 1.20112, thickness / 2 }, { -0.74738, 1.28277, thickness / 2 }, { -0.86842, 1.51932, thickness / 2 }, { -0.85723, 1.52567, thickness / 2 }, { -0.85645, 1.52591, thickness / 2 }, { -0.85561, 1.52582, thickness / 2 }, { -0.8549, 1.52541, thickness / 2 }, { -0.85442, 1.52473, thickness / 2 }
		};
		ChSharedPtr<ChAssetLevel> vlevel (new  ChAssetLevel() );
		ChSharedPtr<ChTriangleMeshShape> vshape (new ChTriangleMeshShape() );
		collision::ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(points, vshape->GetMesh());
		vlevel->GetFrame().SetRot (ChMatrix33<>(i * CH_C_2PI / num_escapements, ChVector<>(0, 0, 1)) ) ;
		vlevel->AddAsset(vshape);
		escapementBody->AddAsset( vlevel );
	}



	mphysicalSystem.Add(escapementBody);


	// 3-Create a rocker 

	ChSharedPtr<ChBody> rockerBody(new ChBody());		//to create the floor, false -> doesn't represent a collide's surface
	rockerBody->SetPos(ChVector<>(0, dist_rocker_center, 0));
	//rockerBody->SetWvel_loc(ChVector<>(0, 0, -25)); // for example

	rockerBody->SetMass(1); // to set
	rockerBody->SetInertiaXX(ChVector<>(1, 1, 1.9837)); // to set: Jzz in [gr*mm^2]

	rockerBody->GetMaterialSurface()->SetFriction(0.0);

	// Collision shape of the rockerBody

	// modifica ultimo punto rockerL: (originale, spigolo paletta interno sx) { -1.1779, -1.0623, -thickness }

	std::vector < ChVector<> > points_rockerL = {
			{ -1.2886, -1.1450, -thickness}, { -1.3018, -1.1300, -thickness}, { -1.3148, -1.1149, -thickness}, { -1.3276, -1.0996, -thickness}, { -1.3402, -1.0842, -thickness},
			{ -1.3526, -1.0686, -thickness}, { -1.3649, -1.0529, -thickness}, { -1.3770, -1.0371, -thickness}, { -1.3889, -1.0211, -thickness}, { -1.4006, -1.0050, -thickness},
			{ -1.4121, -0.9887, -thickness}, { -1.4234, -0.9723, -thickness}, { -1.4346, -0.9558, -thickness}, { -1.4455, -0.9391, -thickness}, { -1.4563, -0.9223, -thickness},
			{ -1.4669, -0.9054, -thickness}, { -1.4773, -0.8883, -thickness}, { -1.4875, -0.8711, -thickness}, { -1.4976, -0.8537, -thickness}, { -1.5074, -0.8362, -thickness},
			{ -1.5171, -0.8185, -thickness }, { -1.4458, -0.7653, -thickness }, { -1.1779, -1.0623, -thickness }, { -1.2886, -1.1450, thickness }, { -1.3018, -1.1300, thickness },
			{ -1.3148, -1.1149, thickness }, { -1.3276, -1.0996, thickness }, { -1.3402, -1.0842, thickness },
			{ -1.3526, -1.0686, thickness }, { -1.3649, -1.0529, thickness }, { -1.3770, -1.0371, thickness }, { -1.3889, -1.0211, thickness }, { -1.4006, -1.0050, thickness },
			{ -1.4121, -0.9887, thickness }, { -1.4234, -0.9723, thickness }, { -1.4346, -0.9558, thickness }, { -1.4455, -0.9391, thickness }, { -1.4563, -0.9223, thickness },
			{ -1.4669, -0.9054, thickness }, { -1.4773, -0.8883, thickness }, { -1.4875, -0.8711, thickness }, { -1.4976, -0.8537, thickness }, { -1.5074, -0.8362, thickness },
			{ -1.5171, -0.8185, thickness }, { -1.4458, -0.7653, thickness }, { -1.1779, -1.0623, thickness }
	};
	std::vector < ChVector<> > points_rockerR1 = {
		    { 1.2747, -1.1603, -thickness}, { 1.1909, -1.0474, -thickness}, { 1.2050, -1.0311, -thickness}, { 1.3073, -1.0667, -thickness}, 
			{ 1.2747, -1.1603, thickness }, { 1.1909, -1.0474, thickness }, { 1.2050, -1.0311, thickness }, { 1.3073, -1.0667, thickness }
	};
	std::vector < ChVector<> > points_rockerR2 = {
		{ 1.3070, -1.0676, -thickness }, { 1.2043, -1.0319, -thickness }, { 1.2181, -1.0155, -thickness }, { 1.3136, -1.0488, -thickness },
		{ 1.3070, -1.0676, thickness }, { 1.2043, -1.0319, thickness }, { 1.2181, -1.0155, thickness }, { 1.3136, -1.0488, thickness }
	};
	std::vector < ChVector<> > points_rockerR3 = {
		{ 1.3132, -1.0497, -thickness }, { 1.2174, -1.0164, -thickness }, { 1.2310, -0.9999, -thickness }, { 1.3198, -1.0308, -thickness },
		{ 1.3132, -1.0497, thickness }, { 1.2174, -1.0164, thickness }, { 1.2310, -0.9999, thickness }, { 1.3198, -1.0308, thickness }
	};
	std::vector < ChVector<> > points_rockerR4 = {
		{ 1.3195, -1.0318, -thickness }, { 1.2303, -1.0007, -thickness }, { 1.2436, -0.9842, -thickness }, { 1.3260, -1.0129, -thickness },
		{ 1.3195, -1.0318, thickness }, { 1.2303, -1.0007, thickness }, { 1.2436, -0.9842, thickness }, { 1.3260, -1.0129, thickness }
	};
	std::vector < ChVector<> > points_rockerR5 = {
		{ 1.3257, -1.0138, -thickness }, { 1.2429, -0.9850, -thickness }, { 1.2559, -0.9684, -thickness }, { 1.3323, -0.9949, -thickness },
		{ 1.3257, -1.0138, thickness }, { 1.2429, -0.9850, thickness }, { 1.2559, -0.9684, thickness }, { 1.3323, -0.9949, thickness }
	};
	std::vector < ChVector<> > points_rockerR6 = {
		{ 1.3320, -0.9959, -thickness }, { 1.2553, -0.9692, -thickness }, { 1.2680, -0.9524, -thickness }, { 1.3385, -0.9770, -thickness },
		{ 1.3320, -0.9959, thickness }, { 1.2553, -0.9692, thickness }, { 1.2680, -0.9524, thickness }, { 1.3385, -0.9770, thickness }
	};
	std::vector < ChVector<> > points_rockerR7 = {
		{ 1.3382, -0.9779, -thickness }, { 1.2674, -0.9533, -thickness }, { 1.2799, -0.9365, -thickness }, { 1.3448, -0.9590, -thickness },
		{ 1.3382, -0.9779, thickness }, { 1.2674, -0.9533, thickness }, { 1.2799, -0.9365, thickness }, { 1.3448, -0.9590, thickness }
	};
	std::vector < ChVector<> > points_rockerR8 = {
		{ 1.3444, -0.9600, -thickness }, { 1.2793, -0.9373, -thickness }, { 1.2915, -0.9204, -thickness }, { 1.3510, -0.9411, -thickness },
		{ 1.3444, -0.9600, thickness }, { 1.2793, -0.9373, thickness }, { 1.2915, -0.9204, thickness }, { 1.3510, -0.9411, thickness }
	};
	std::vector < ChVector<> > points_rockerR9 = {
		{ 1.3507,-0.9420, -thickness }, { 1.2909, -0.9212, -thickness }, { 1.3029, -0.9042, -thickness }, { 1.3573, -0.9231, -thickness },
		{ 1.3507,-0.9420, thickness }, { 1.2909, -0.9212, thickness }, { 1.3029, -0.9042, thickness }, { 1.3573, -0.9231, thickness }
	};
	std::vector < ChVector<> > points_rockerR10 = {
		{ 1.3569, -0.9241, -thickness }, { 1.3023, -0.9051, -thickness }, { 1.3140, -0.8880, -thickness }, { 1.3635, -0.9052, -thickness },
		{ 1.3569, -0.9241, thickness }, { 1.3023, -0.9051, thickness }, { 1.3140, -0.8880, thickness }, { 1.3635, -0.9052, thickness }
	};
	std::vector < ChVector<> > points_rockerR11 = {
		{ 1.3632, -0.9061, -thickness }, { 1.3134, -0.8888, -thickness }, { 1.3249, -0.8716, -thickness }, { 1.3697, -0.8872, -thickness },
		{ 1.3632, -0.9061, thickness }, { 1.3134, -0.8888, thickness }, { 1.3249, -0.8716, thickness }, { 1.3697, -0.8872, thickness }
	};
	std::vector < ChVector<> > points_rockerR12 = {
		{ 1.3694, -0.8882, -thickness }, { 1.3243, -0.8725, -thickness }, { 1.3355, -0.8552, -thickness }, { 1.3760, -0.8693, -thickness },
		{ 1.3694, -0.8882, thickness }, { 1.3243, -0.8725, thickness }, { 1.3355, -0.8552, thickness }, { 1.3760, -0.8693, thickness }
	};
	std::vector < ChVector<> > points_rockerR13 = {
		{ 1.3757, -0.8702, -thickness }, { 1.3350, -0.8561, -thickness }, { 1.3460, -0.8387, -thickness }, { 1.3822, -0.8514, -thickness },
		{ 1.3757, -0.8702, thickness }, { 1.3350, -0.8561, thickness }, { 1.3460, -0.8387, thickness }, { 1.3822, -0.8514, thickness }
	};
	std::vector < ChVector<> > points_rockerR14 = {
		{ 1.3819, -0.8523, -thickness }, { 1.3454, -0.8396, -thickness }, { 1.3561, -0.8222, -thickness }, { 1.3885, -0.8334, -thickness },
		{ 1.3819, -0.8523, thickness }, { 1.3454, -0.8396, thickness }, { 1.3561, -0.8222, thickness }, { 1.3885, -0.8334, thickness }
	};
	std::vector < ChVector<> > points_rockerR15 = {
		{ 1.3881, -0.8344, -thickness }, { 1.3556, -0.8230, -thickness }, { 1.3661, -0.8055, -thickness }, { 1.3947, -0.8155, -thickness },
		{ 1.3881, -0.8344, thickness }, { 1.3556, -0.8230, thickness }, { 1.3661, -0.8055, thickness }, { 1.3947, -0.8155, thickness }
	};

	rockerBody->GetCollisionModel()->SetSafeMargin(0.001); 
	rockerBody->GetCollisionModel()->SetEnvelope(0.002);

	rockerBody->GetCollisionModel()->ClearModel();
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerL, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR1, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR2, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR3, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR4, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR5, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR6, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR7, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR8, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR9, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR10, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR11, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR12, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR13, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR14, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->AddConvexHull(points_rockerR15, ChVector<>(0, 0, 0));
	rockerBody->GetCollisionModel()->BuildModel();
	rockerBody->SetCollide(true);


	// Visualization of the collision shapes in rocker

	ChSharedPtr<ChTriangleMeshShape> vshapeL (new ChTriangleMeshShape() );
	collision::ChConvexHullLibraryWrapper lhL;
	lhL.ComputeHull(points_rockerL, vshapeL->GetMesh());
	rockerBody->AddAsset( vshapeL );

	ChSharedPtr<ChTriangleMeshShape> vshapeR1 (new ChTriangleMeshShape() );
	collision::ChConvexHullLibraryWrapper lhR1;
	lhR1.ComputeHull(points_rockerR1, vshapeR1->GetMesh());
	rockerBody->AddAsset( vshapeR1 );
	ChSharedPtr<ChTriangleMeshShape> vshapeR2(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR2;
	lhR2.ComputeHull(points_rockerR2, vshapeR2->GetMesh());
	rockerBody->AddAsset(vshapeR2);
	ChSharedPtr<ChTriangleMeshShape> vshapeR3(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR3;
	lhR3.ComputeHull(points_rockerR3, vshapeR3->GetMesh());
	rockerBody->AddAsset(vshapeR3);
	ChSharedPtr<ChTriangleMeshShape> vshapeR4(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR4;
	lhR4.ComputeHull(points_rockerR4, vshapeR4->GetMesh());
	rockerBody->AddAsset(vshapeR4);
	ChSharedPtr<ChTriangleMeshShape> vshapeR5(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR5;
	lhR5.ComputeHull(points_rockerR5, vshapeR5->GetMesh());
	rockerBody->AddAsset(vshapeR5);
	ChSharedPtr<ChTriangleMeshShape> vshapeR6(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR6;
	lhR6.ComputeHull(points_rockerR6, vshapeR6->GetMesh());
	rockerBody->AddAsset(vshapeR6);
	ChSharedPtr<ChTriangleMeshShape> vshapeR7(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR7;
	lhR7.ComputeHull(points_rockerR7, vshapeR7->GetMesh());
	rockerBody->AddAsset(vshapeR7);
	ChSharedPtr<ChTriangleMeshShape> vshapeR8(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR8;
	lhR8.ComputeHull(points_rockerR8, vshapeR8->GetMesh());
	rockerBody->AddAsset(vshapeR8);
	ChSharedPtr<ChTriangleMeshShape> vshapeR9(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR9;
	lhR9.ComputeHull(points_rockerR9, vshapeR9->GetMesh());
    rockerBody->AddAsset(vshapeR9);
	ChSharedPtr<ChTriangleMeshShape> vshapeR10(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR10;
	lhR10.ComputeHull(points_rockerR10, vshapeR10->GetMesh());
	rockerBody->AddAsset(vshapeR10);
	ChSharedPtr<ChTriangleMeshShape> vshapeR11(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR11;
	lhR11.ComputeHull(points_rockerR11, vshapeR11->GetMesh());
	rockerBody->AddAsset(vshapeR11);
	ChSharedPtr<ChTriangleMeshShape> vshapeR12(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR12;
	lhR12.ComputeHull(points_rockerR12, vshapeR12->GetMesh());
	rockerBody->AddAsset(vshapeR12);
	ChSharedPtr<ChTriangleMeshShape> vshapeR13(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR13;
	lhR13.ComputeHull(points_rockerR13, vshapeR13->GetMesh());
	rockerBody->AddAsset(vshapeR13);
	ChSharedPtr<ChTriangleMeshShape> vshapeR14(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR14;
	lhR14.ComputeHull(points_rockerR14, vshapeR14->GetMesh());
	rockerBody->AddAsset(vshapeR14);
	ChSharedPtr<ChTriangleMeshShape> vshapeR15(new ChTriangleMeshShape());
	collision::ChConvexHullLibraryWrapper lhR15;
	lhR15.ComputeHull(points_rockerR15, vshapeR15->GetMesh());
	rockerBody->AddAsset(vshapeR15);


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
	application.SetTimestep(0.00001);
	application.SetTryRealtime(false);
	application.SetVideoframeSaveInterval(20);

	mphysicalSystem.SetIterLCPmaxItersSpeed(100);
	mphysicalSystem.SetLcpSolverType(ChSystem::eCh_lcpSolver::LCP_ITERATIVE_BARZILAIBORWEIN); // or: LCP_ITERATIVE_APGD or: LCP_ITERATIVE_SOR (for speed)
	
	//mphysicalSystem.SetIntegrationType(ChSystem::eCh_integrationType::INT_ANITESCU); // in future: INT_HHT or other


    // This is the contact reporter class
    class _contact_reporter_class : public  chrono::ChReportContactCallback 
    {
        public:
        ChStreamOutAsciiFile* mfile; // the file to save data into
        double mtime;
        int mstep;

        virtual bool ReportContactCallback(const chrono::ChVector<>& pA,
                                           const chrono::ChVector<>& pB,
                                           const chrono::ChMatrix33<>& plane_coord,
                                           const double& distance,
                                           const float& mfriction,
                                           const chrono::ChVector<>& react_forces,
                                           const chrono::ChVector<>& react_torques,
                                           chrono::collision::ChCollisionModel* modA,
                                           chrono::collision::ChCollisionModel* modB) 
        {
            // For each contact, this function is executed. 
            // In this example, saves on ascii file:
            //  step, time,  position xyz, direction xyz, normal impulse, modelA ID, modelB ID information is saved. 
            (*mfile) <<  mstep << " "
                     <<  mtime << " " 
                     << pA.x << " " 
                     << pA.y << " " 
                     << pA.z << " " 
                     << plane_coord.Get_A_Xaxis().x << " "
                     << plane_coord.Get_A_Xaxis().y << " "
                     << plane_coord.Get_A_Xaxis().z << " "
                     << react_forces.x << " "
                     << modA->GetPhysicsItem()->GetIdentifier() << " "
                     << modB->GetPhysicsItem()->GetIdentifier() << "\n";

            return true;  // to continue scanning contacts
        }
    };


    // The following will enable the 'contact reporting' feature, that is the ReportContactCallback() above 
    // will be executed for each contact. 

    _contact_reporter_class my_contact_rep;

    ChStreamOutAsciiFile result_contacts("contacts.txt");
    my_contact_rep.mfile = &result_contacts;
    my_contact_rep.mtime = 0;
    my_contact_rep.mstep = 0;

	// 
	// THE SOFT-REAL-TIME CYCLE
	//

	ChStreamOutAsciiFile result_rocker_Theta("output_rocker_Theta.txt");
	ChStreamOutAsciiFile result_rocker_ThetaP("output_rocker_ThetaP.txt");
	ChStreamOutAsciiFile result_rocker_ThetaPP("output_rocker_ThetaPP.txt");
	ChStreamOutAsciiFile result_wheel_Theta("output_wheel_Theta.txt");
	ChStreamOutAsciiFile result_wheel_ThetaP("output_wheel_ThetaP.txt");
	ChStreamOutAsciiFile result_wheel_ThetaPP("output_wheel_ThetaPP.txt");

	while (application.GetDevice()->run())
	{
		application.BeginScene();

		application.DrawAll();

		// This performs the integration timestep!
		//for (int i=0;i<100; ++i)
		application.DoStep();
		
		// Save results:
		result_rocker_Theta << rockerBody->GetRotAngle() << "\n";
		result_rocker_ThetaP << rockerBody->GetWvel_loc().z << "\n";
		result_rocker_ThetaPP << rockerBody->GetWacc_loc().z << "\n";

		result_wheel_Theta << escapementBody->GetRotAngle() << "\n";
		result_wheel_ThetaP << escapementBody->GetWvel_loc().z << "\n";
		result_wheel_ThetaPP << escapementBody->GetWacc_loc().z << "\n";
		
        // Save contacts:
        my_contact_rep.mtime = mphysicalSystem.GetChTime();
        my_contact_rep.mstep++;
        mphysicalSystem.GetContactContainer()->ReportAllContacts(&my_contact_rep);

		application.EndScene();

		if (mphysicalSystem.GetChTime() > 0.1) 
			break;

	}


	return 0;
}
  
