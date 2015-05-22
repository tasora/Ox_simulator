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
#include "physics/ChBodyAuxRef.h"
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

	// SCAPPAMENTO 2 NEW

	double dist_rocker_center = 2.2662;
	double thickness = 0.15;
	double ang_speed = (CH_C_PI)* 2;
	// braccio piano di impulso - COG rocker
	double braccio = 0.274;

	// 1-Create the truss  

	ChSharedPtr<ChBodyEasyBox> trussBody(new ChBodyEasyBox(6,7,0.01, 1, false, true));		//to create the floor, false -> doesn't represent a collide's surface
	trussBody->SetPos(ChVector<>(0, 0, thickness));
	trussBody->SetBodyFixed(true);

	mphysicalSystem.Add(trussBody);


	// 2-Create the escapement wheel 

	ChSharedPtr<ChBody> escapementBody(new ChBody());		//to create the floor, false -> doesn't represent a collide's surface
	escapementBody->SetPos(ChVector<>(0, 0, 0));
	//escapementBody->SetWvel_loc(ChVector<>(0, 0, -ang_speed)); // for example
	escapementBody->Set_Scr_torque(ChVector<>(0, 0, 2*-37.91)); // constant torque to eascape wheel [uN*mm]

	escapementBody->GetMaterialSurface()->SetFriction(0.1);

	escapementBody->SetMass(1); // to set
	escapementBody->SetInertiaXX( ChVector<>(1, 1, 0.00123) ); // to set: Jzz in [gr*mm^2]

	// Collision shape of the escapementBody

	escapementBody->GetCollisionModel()->SetSafeMargin(0.001); 
	escapementBody->GetCollisionModel()->SetEnvelope(0.002);

	escapementBody->GetCollisionModel()->ClearModel();
	int num_escapements = 25;
	for (int i=0; i<num_escapements; ++i)
	{
		escapementBody->GetCollisionModel()->AddConvexHull(std::vector < ChVector<> > 
		{
			{-0.79705, 1.12201, -thickness / 2}, { -0.78825, 1.44954, -thickness / 2 }, { -0.77692, 1.45564, -thickness / 2 }, { -0.77605, 1.45588, -thickness / 2 }, { -0.77529, 1.45576, -thickness / 2 }, { -0.77457, 1.4553, -thickness / 2 }, { -0.77412, 1.45463, -thickness / 2 }, { -0.67397, 1.20773, -thickness / 2 },
			{ -0.79705, 1.12201, thickness / 2 }, { -0.78825, 1.44954, thickness / 2 }, { -0.77692, 1.45564, thickness / 2 }, { -0.77605, 1.45588, thickness / 2 }, { -0.77529, 1.45576, thickness / 2 }, { -0.77457, 1.4553, thickness / 2 }, { -0.77412, 1.45463, thickness / 2 }, { -0.67397, 1.20773, thickness / 2 }
		}, ChVector<>(0, 0, 0), (ChMatrix33<>(i * CH_C_2PI / num_escapements, ChVector<>(0, 0, 1)) ));
	}
	escapementBody->GetCollisionModel()->BuildModel();
	escapementBody->SetCollide(true);


	// c.hulls visualization
	for (int i=0; i<num_escapements; ++i)
	{
		std::vector < ChVector<> >   points = 
		{
			{ -0.79705, 1.12201, -thickness / 2 }, { -0.78825, 1.44954, -thickness / 2 }, { -0.77692, 1.45564, -thickness / 2 }, { -0.77605, 1.45588, -thickness / 2 }, { -0.77529, 1.45576, -thickness / 2 }, { -0.77457, 1.4553, -thickness / 2 }, { -0.77412, 1.45463, -thickness / 2 }, { -0.67397, 1.20773, -thickness / 2 },
			{ -0.79705, 1.12201, thickness / 2 }, { -0.78825, 1.44954, thickness / 2 }, { -0.77692, 1.45564, thickness / 2 }, { -0.77605, 1.45588, thickness / 2 }, { -0.77529, 1.45576, thickness / 2 }, { -0.77457, 1.4553, thickness / 2 }, { -0.77412, 1.45463, thickness / 2 }, { -0.67397, 1.20773, thickness / 2 }
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


	// RockerBody creato con funzione ChBody (sistema di riferimento coincidente con COG del corpo rigido, utilizzabile con vincolo cerniera, NO vincolo sferico)

	//ChSharedPtr<ChBody> rockerBody(new ChBody());		//to create the floor, false -> doesn't represent a collide's surface
	//rockerBody->SetPos(ChVector<>(0, dist_rocker_center, 0)); // COG's position of the rocker
	//rockerBody->SetWvel_loc(ChVector<>(0, 0, -25)); // initial angular velocity of the rocker


	// RockerBody creato con funzione ChBodyAuxRef (sistema di riferimento NON coincidente con COG del corpo rigido, utilizzabile con vincolo sferico)

	ChSharedPtr<ChBodyAuxRef> rockerBody(new ChBodyAuxRef());
	// Set position of COG respect to reference
	rockerBody->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0, 0, braccio), ChQuaternion<>(1, 0, 0, 0)));
	// Set position of reference in absolute space
	rockerBody->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(0, dist_rocker_center, 0), ChQuaternion<>(1, 0, 0, 0)));
	//rockerBody->SetPos(ChVector<>(0, dist_rocker_center, braccio)); // COG's position of the rocker

	rockerBody->SetMass(1); // to set
	rockerBody->SetInertiaXX(ChVector<>(0.154239195, 1.826093039, 1.807630708)); // to set: Jzz in [gr*mm^2]

	rockerBody->GetMaterialSurface()->SetFriction(0.1);

	// Collision shape of the rockerBody

	// ultimo punto rockerL: (originale, spigolo paletta interno sx) { -1.1358, -1.0897, -thickness }

	std::vector < ChVector<> > points_rockerL = {
			{ -1.2602, -1.1934, -thickness}, { -1.2752, -1.1774, -thickness}, { -1.2900, -1.1612, -thickness}, { -1.3047, -1.1446, -thickness}, { -1.3193, -1.1278, -thickness},
			{ -1.3337, -1.1107, -thickness}, { -1.3480, -1.0933, -thickness}, { -1.3622, -1.0756, -thickness}, { -1.3762, -1.0575, -thickness}, { -1.3902, -1.0392, -thickness},
			{ -1.4039, -1.0205, -thickness}, { -1.4176, -1.0014, -thickness}, { -1.4311, -0.9821, -thickness}, { -1.4444, -0.9624, -thickness}, { -1.4428, -0.9588, -thickness},
			{ -1.1358, -1.0897, -thickness }, { -1.2602, -1.1934, thickness }, { -1.2752, -1.1774, thickness }, { -1.2900, -1.1612, thickness }, { -1.3047, -1.1446, thickness }, 
			{ -1.3193, -1.1278, thickness },
			{ -1.3337, -1.1107, thickness }, { -1.3480, -1.0933, thickness }, { -1.3622, -1.0756, thickness }, { -1.3762, -1.0575, thickness }, { -1.3902, -1.0392, thickness },
			{ -1.4039, -1.0205, thickness }, { -1.4176, -1.0014, thickness }, { -1.4311, -0.9821, thickness }, { -1.4444, -0.9624, thickness }, { -1.4428, -0.9588, thickness },
			{ -1.1358, -1.0897, thickness }
	};
	std::vector < ChVector<> > points_rockerR1 = {
		    { 1.1492, -1.0752, -thickness}, { 1.1653, -1.0577, -thickness}, { 1.2690, -1.0758, -thickness}, { 1.2459, -1.2083, -thickness}, 
			{ 1.1492, -1.0752, thickness }, { 1.1653, -1.0577, thickness }, { 1.2690, -1.0758, thickness }, { 1.2459, -1.2083, thickness }
	};
	std::vector < ChVector<> > points_rockerR2 = {
		{ 1.1645, -1.0586, -thickness }, { 1.1802, -1.0410, -thickness }, { 1.2723, -1.0571, -thickness }, { 1.2689, -1.0768, -thickness },
		{ 1.1645, -1.0586, thickness }, { 1.1802, -1.0410, thickness }, { 1.2723, -1.0571, thickness }, { 1.2689, -1.0768, thickness }
	};
	std::vector < ChVector<> > points_rockerR3 = {
		{ 1.1794, -1.0419, -thickness }, { 1.1948, -1.0243, -thickness }, { 1.2756, -1.0384, -thickness }, { 1.2721, -1.0581, -thickness },
		{ 1.1794, -1.0419, thickness }, { 1.1948, -1.0243, thickness }, { 1.2756, -1.0384, thickness }, { 1.2721, -1.0581, thickness }
	};
	std::vector < ChVector<> > points_rockerR4 = {
		{ 1.1940, -1.0251, -thickness }, { 1.2090, -1.0075, -thickness }, { 1.2788, -1.0197, -thickness }, { 1.2754, -1.0394, -thickness },
		{ 1.1940, -1.0251, thickness }, { 1.2090, -1.0075, thickness }, { 1.2788, -1.0197, thickness }, { 1.2754, -1.0394, thickness }
	};
	std::vector < ChVector<> > points_rockerR5 = {
		{ 1.2082, -1.0083, -thickness }, { 1.2228, -0.9906, -thickness }, { 1.2821, -1.0010, -thickness }, { 1.2787, -1.0207, -thickness },
		{ 1.2082, -1.0083, thickness }, { 1.2228, -0.9906, thickness }, { 1.2821, -1.0010, thickness }, { 1.2787, -1.0207, thickness }
	};
	std::vector < ChVector<> > points_rockerR6 = {
		{ 1.2221, -0.9915, -thickness }, { 1.2363, -0.9737, -thickness }, { 1.2854, -0.9822, -thickness }, { 1.2819, -1.0019, -thickness },
		{ 1.2221, -0.9915, thickness }, { 1.2363, -0.9737, thickness }, { 1.2854, -0.9822, thickness }, { 1.2819, -1.0019, thickness }
	};
	std::vector < ChVector<> > points_rockerR7 = {
		{ 1.2356, -0.9746, -thickness }, { 1.2495, -0.9567, -thickness }, { 1.2887, -0.9635, -thickness }, { 1.2852, -0.9832, -thickness },
		{ 1.2356, -0.9746, thickness }, { 1.2495, -0.9567, thickness }, { 1.2887, -0.9635, thickness }, { 1.2852, -0.9832, thickness }
	};
	std::vector < ChVector<> > points_rockerR8 = {
		{ 1.2488, -0.9576, -thickness }, { 1.2624, -0.9396, -thickness }, { 1.2919, -0.9448, -thickness }, { 1.2885, -0.9645, -thickness },
		{ 1.2488, -0.9576, thickness }, { 1.2624, -0.9396, thickness }, { 1.2919, -0.9448, thickness }, { 1.2885, -0.9645, thickness }
	};
	std::vector < ChVector<> > points_rockerR9 = {
		{ 1.2617,-0.9405, -thickness }, { 1.2749, -0.9225, -thickness }, { 1.2952, -0.9261, -thickness }, { 1.2918, -0.9458, -thickness },
		{ 1.2617,-0.9405, thickness }, { 1.2749, -0.9225, thickness }, { 1.2952, -0.9261, thickness }, { 1.2918, -0.9458, thickness }
	};
	std::vector < ChVector<> > points_rockerR10 = {
		{ 1.2743, -0.9234, -thickness }, { 1.2872, -0.9054, -thickness }, { 1.2985, -0.9074, -thickness }, { 1.2950, -0.9271, -thickness },
		{ 1.2743, -0.9234, thickness }, { 1.2872, -0.9054, thickness }, { 1.2985, -0.9074, thickness }, { 1.2950, -0.9271, thickness }
	};
	std::vector < ChVector<> > points_rockerR11 = {
		{ 1.2865, -0.9063, -thickness }, { 1.2991, -0.8882, -thickness }, { 1.3018, -0.8887, -thickness }, { 1.2983, -0.9084, -thickness },
		{ 1.2865, -0.9063, thickness }, { 1.2991, -0.8882, thickness }, { 1.3018, -0.8887, thickness }, { 1.2983, -0.9084, thickness }
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

	ChSharedPtr<ChLinkLockSpherical> rockerLink(new ChLinkLockSpherical()); 

	ChCoordsys<> link_position_abs2(ChVector<>(0, dist_rocker_center, braccio));

	rockerLink->Initialize(trussBody, rockerBody, link_position_abs2);		// the link reference attached to 2nd body

	rockerLink->GetForce_Rz()->Set_active(true);
	rockerLink->GetForce_Rz()->Set_K(44601.5); // Torsional stiffness Z, to set, in [uN*mm/rad]
	rockerLink->GetForce_Ry()->Set_active(true);
	rockerLink->GetForce_Ry()->Set_K(6706289.796); // Torsional stiffness Y, to set, in [uN*mm/rad]
	rockerLink->GetForce_Rx()->Set_active(true);
	rockerLink->GetForce_Rx()->Set_K(611889.515); // Torsional stiffness X, to set, in [uN*mm/rad]
	rockerLink->GetForce_Rz()->Set_active(true);
	rockerLink->GetForce_Rz()->Set_R(0); // Torsional damping Z, to set, in [uN*mm*s/rad]

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

	ChStreamOutAsciiFile result_rocker_Theta("output_rocker_Theta_ThetaP_ThetaPP.txt");
	ChStreamOutAsciiFile result_wheel_Theta("output_wheel_Theta_ThetaP_ThetaPP.txt");


	while (application.GetDevice()->run())
	{
		application.BeginScene();

		application.DrawAll();

		// This performs the integration timestep!
		//for (int i=0;i<100; ++i)
		application.DoStep();
		
		// Save results:

		double angle_z = 0; 
		if (rockerBody->GetRotAxis().z > 0)
			angle_z = rockerBody->GetRotAngle();
		else
			angle_z = -rockerBody->GetRotAngle();

		result_rocker_Theta 
		<< angle_z << " "
		<< rockerBody->GetWvel_loc().x << " "
		<< rockerBody->GetWvel_loc().y << " "
		<< rockerBody->GetWvel_loc().z << " "
		<< rockerBody->GetWacc_loc().z << "\n";

		result_wheel_Theta 
		<< escapementBody->GetRotAngle() << " "
		<< escapementBody->GetWvel_loc().z << " "
		<< escapementBody->GetWacc_loc().z << "\n";
		
        // Save contacts:
        my_contact_rep.mtime = mphysicalSystem.GetChTime();
        my_contact_rep.mstep++;
        mphysicalSystem.GetContactContainer()->ReportAllContacts(&my_contact_rep);

		application.EndScene();

		if (mphysicalSystem.GetChTime() > 1) 
			break;

	}


	return 0;
}
  
