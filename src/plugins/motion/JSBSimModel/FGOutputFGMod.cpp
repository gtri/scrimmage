/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/motion/JSBSimModel/FGOutputFGMod.h>

#include <stdio.h>
#include <string.h>

#include <algorithm>

#include "models/FGAerodynamics.h"
#include "models/FGAuxiliary.h"
#include "models/FGPropulsion.h"
#include "models/FGMassBalance.h"
#include "models/FGPropagate.h"
#include "models/FGGroundReactions.h"
#include "models/FGFCS.h"
#include "models/propulsion/FGPiston.h"
#include "models/propulsion/FGElectric.h"
#include "models/propulsion/FGTank.h"

// NOTE: This file has a couple of quirky ways of converting between data types
// because it's how it is done in FlightGear / JSBSim. That's why there are
// some NOLINT and cppcheck suppressions.

static const int endianTest = 1;
#define isLittleEndian (*((char *) &endianTest ) != 0) // NOLINT

namespace JSBSim {

// TODO: Replace JSBSim's methods for network byte ordering
static void htond(double &x) {
    if (isLittleEndian) {
        int    *Double_Overlay;
        int     Holding_Buffer;

        // Double_Overlay = (int *) &x;
        // cppcheck-suppress invalidPointerCast
        Double_Overlay = reinterpret_cast<int *>(&x);
        Holding_Buffer = Double_Overlay[0];

        Double_Overlay[0] = htonl(Double_Overlay[1]);
        Double_Overlay[1] = htonl(Holding_Buffer);
    } else {
        return;
    }
}

// TODO: Replace JSBSim's methods for network byte ordering
// Float version
static void htonf(float &x) {
    if (isLittleEndian) {
        int    *Float_Overlay;
        int     Holding_Buffer;
				
        // cppcheck-suppress invalidPointerCast
        Float_Overlay = reinterpret_cast<int *>(&x);
        Holding_Buffer = Float_Overlay[0];

        Float_Overlay[0] = htonl(Holding_Buffer);
    } else {
        return;
    }
}

FGOutputFGMod::FGOutputFGMod(FGFDMExec* fdmex) :
		FGOutputFG(fdmex) {
    memset(net1, 0x0, sizeof(s));
		dataLength = sizeof(FGNetFDM1) + sizeof(FGNetFDM3); // FS 2020.2 or earlier does not use FGNetFDM2

		if (fdmex->GetDebugLevel() > 0) {
				 // Engine status
				 if (Propulsion->GetNumEngines() > FG_MAX_ENGINES)
					 cerr << "This vehicle has " << Propulsion->GetNumEngines() << " engines, but the current " << endl
								<< "version of FlightGear's FGNetFDM only supports " << FG_MAX_ENGINES << " engines." << endl
								<< "Only the first " << FG_MAX_ENGINES << " engines will be used." << endl;
			
				 // Consumables
				 if (Propulsion->GetNumTanks() > FG_MAX_TANKS)
					 cerr << "This vehicle has " << Propulsion->GetNumTanks() << " tanks, but the current " << endl
								<< "version of FlightGear's FGNetFDM only supports " << FG_MAX_TANKS << " tanks." << endl
								<< "Only the first " << FG_MAX_TANKS << " tanks will be used." << endl;
			
				 // Gear status
				 if (GroundReactions->GetNumGearUnits() > FG_MAX_WHEELS)
					 cerr << "This vehicle has " << GroundReactions->GetNumGearUnits() << " bogeys, but the current " << endl
								<< "version of FlightGear's FGNetFDM only supports " << FG_MAX_WHEELS << " bogeys." << endl
								<< "Only the first " << FG_MAX_WHEELS << " bogeys will be used." << endl;
			 }
}

void FGOutputFGMod::Print(void) {
  if (socket == 0) return;
  if (!socket->GetConnectStatus()) return;

  SocketDataFillMod();
  socket->Send((char *)net1, dataLength); // NOLINT
}

void FGOutputFGMod::SocketDataFillMod(void) {
  unsigned int i;

  // Version
	// Currently not supporting FlightGear 2020.3 or later
  net1->version = htonl(24);
	net3 = (FGNetFDM3 *)(net1 + 1);
	
  // Positions
  net1->longitude = Propagate->GetLocation().GetLongitude(); //
  net1->latitude  = Propagate->GetLocation().GetLatitude(); // GeodLatitudeRad(); // geodetic (radians)
  net1->altitude  = Propagate->GetAltitudeASL()*0.3048; // altitude, above sea level (meters)
  net1->agl       = static_cast<float>(Propagate->GetDistanceAGL()*0.3048); // altitude, above ground level (meters)

  net1->phi       = static_cast<float>(Propagate->GetEuler(ePhi)); // roll (radians)
  net1->theta     = static_cast<float>(Propagate->GetEuler(eTht)); // pitch (radians)
  net1->psi       = static_cast<float>(Propagate->GetEuler(ePsi)); // yaw or true heading (radians)

  net1->alpha     = static_cast<float>(Auxiliary->Getalpha()); // angle of attack (radians)
  net1->beta      = static_cast<float>(Auxiliary->Getbeta()); // side slip angle (radians)

  // Velocities
  net1->phidot     = static_cast<float>(Auxiliary->GetEulerRates(ePhi)); // roll rate (radians/sec)
  net1->thetadot   = static_cast<float>(Auxiliary->GetEulerRates(eTht)); // pitch rate (radians/sec)
  net1->psidot     = static_cast<float>(Auxiliary->GetEulerRates(ePsi)); // yaw rate (radians/sec)
  net1->vcas       = static_cast<float>(Auxiliary->GetVcalibratedKTS()); // VCAS, knots
  net1->climb_rate = static_cast<float>(Propagate->Gethdot());           // altitude rate, ft/sec
  net1->v_north    = static_cast<float>(Propagate->GetVel(eNorth));      // north vel in NED frame, fps
  net1->v_east     = static_cast<float>(Propagate->GetVel(eEast));       // east vel in NED frame, fps
  net1->v_down     = static_cast<float>(Propagate->GetVel(eDown));       // down vel in NED frame, fps
  // ---ADD METHOD TO CALCULATE THESE TERMS---
  net1->v_body_u = static_cast<float>(Propagate->GetUVW(1)); // ECEF speed in body axis
  net1->v_body_v = static_cast<float>(Propagate->GetUVW(2)); // ECEF speed in body axis
  net1->v_body_w = static_cast<float>(Propagate->GetUVW(3)); // ECEF speed in body axis

  // Accelerations
  net1->A_X_pilot   = static_cast<float>(Auxiliary->GetPilotAccel(1));    // X body accel, ft/s/s
  net1->A_Y_pilot   = static_cast<float>(Auxiliary->GetPilotAccel(2));    // Y body accel, ft/s/s
  net1->A_Z_pilot   = static_cast<float>(Auxiliary->GetPilotAccel(3));    // Z body accel, ft/s/s

  // Stall
  net1->stall_warning = 0.0;  // 0.0 - 1.0 indicating the amount of stall
  net1->slip_deg    = static_cast<float>(Auxiliary->Getbeta(inDegrees));  // slip ball deflection, deg

  net1->num_engines = std::min(static_cast<size_t>(FG_MAX_ENGINES),
                              Propulsion->GetNumEngines()); // Number of valid engines

  for (i=0; i < net1->num_engines; i++) {
		auto engine = Propulsion->GetEngine(i);
    if (engine->GetRunning())
      net1->eng_state[i] = 2;       // Engine state running
    else if (engine->GetCranking())
      net1->eng_state[i] = 1;       // Engine state cranking
    else
      net1->eng_state[i] = 0;       // Engine state off

    switch (engine->GetType()) {
    case (FGEngine::etRocket):
        break;
    case (FGEngine::etPiston): 
				{
				auto piston_engine = static_pointer_cast<FGPiston>(engine);
        net1->rpm[i]       = static_cast<float>(piston_engine->getRPM());
        net1->fuel_flow[i] = static_cast<float>(piston_engine->getFuelFlow_gph());
        net1->fuel_px[i]   = 0; // Fuel pressure, psi  (N/A in current model)
        net1->egt[i]       = static_cast<float>(piston_engine->GetEGT());
        net1->cht[i]       = static_cast<float>(piston_engine->getCylinderHeadTemp_degF());
        net1->mp_osi[i]    = static_cast<float>(piston_engine->getManifoldPressure_inHg());
        net1->oil_temp[i]  = static_cast<float>(piston_engine->getOilTemp_degF());
        net1->oil_px[i]    = static_cast<float>(piston_engine->getOilPressure_psi());
        net1->tit[i]       = 0; // Turbine Inlet Temperature  (N/A for piston)
				}
        break;
    case (FGEngine::etTurbine):
        break;
    case (FGEngine::etTurboprop):
        break;
    case (FGEngine::etElectric):
				net1->rpm[i] = static_cast<float>(static_pointer_cast<FGElectric>(engine)->getRPM());
        break;
    case (FGEngine::etUnknown):
        break;
    }
  }

  net1->num_tanks = std::min(static_cast<size_t>(FG_MAX_TANKS),
                            Propulsion->GetNumTanks());   // Max number of fuel tanks

  for (i=0; i < net1->num_tanks; i++) {
      net1->fuel_quantity[i] = static_cast<float>(Propulsion->GetTank(i)->GetContents());
  }

  net3->num_wheels  = std::min(static_cast<int>(FG_MAX_WHEELS),
                              GroundReactions->GetNumGearUnits());

  for (i=0; i < net3->num_wheels; i++) {
    net3->wow[i] = GroundReactions->GetGearUnit(i)->GetWOW();
    if (GroundReactions->GetGearUnit(i)->GetGearUnitDown())
      net3->gear_pos[i]      = 1;  // gear down, using FCS convention
    else
      net3->gear_pos[i]      = 0;  // gear up, using FCS convention
    net3->gear_steer[i]       = static_cast<float>(GroundReactions->GetGearUnit(i)->GetSteerNorm());
    net3->gear_compression[i] = static_cast<float>(GroundReactions->GetGearUnit(i)->GetCompLen());

    net3->gear_pos[i] = 0; // MOD: Force gear up
  }

  // Environment
  net3->cur_time    = static_cast<uint64_t>(1234567890);    // Friday, Feb 13, 2009, 23:31:30 UTC (not processed by FGFS anyway)
  net3->warp        = 0;                       // offset in seconds to unix time
  net3->visibility  = 25000.0;                 // visibility in meters (for env. effects)

  // Control surface positions (normalized values)
  net3->elevator          = static_cast<float>(FCS->GetDePos(ofNorm));    // Norm Elevator Pos, --
  net3->elevator_trim_tab = static_cast<float>(FCS->GetPitchTrimCmd());   // Norm Elev Trim Tab Pos, --
  net3->left_flap         = static_cast<float>(FCS->GetDfPos(ofNorm));    // Norm Flap Pos, --
  net3->right_flap        = static_cast<float>(FCS->GetDfPos(ofNorm));    // Norm Flap Pos, --
  net3->left_aileron      = static_cast<float>(FCS->GetDaLPos(ofNorm));   // Norm L Aileron Pos, --
  net3->right_aileron     = static_cast<float>(FCS->GetDaRPos(ofNorm));   // Norm R Aileron Pos, --
  net3->rudder            = static_cast<float>(FCS->GetDrPos(ofNorm));    // Norm Rudder Pos, --
  net3->nose_wheel        = static_cast<float>(FCS->GetDrPos(ofNorm));    // *** FIX ***  Using Rudder Pos for NWS, --
  net3->speedbrake        = static_cast<float>(FCS->GetDsbPos(ofNorm));   // Norm Speedbrake Pos, --
  net3->spoilers          = static_cast<float>(FCS->GetDspPos(ofNorm));   // Norm Spoiler Pos, --

  // Convert the net buffer to network format
  if ( isLittleEndian ) {
    htond(net1->longitude);
    htond(net1->latitude);
    htond(net1->altitude);
    htonf(net1->agl);
    htonf(net1->phi);
    htonf(net1->theta);
    htonf(net1->psi);
    htonf(net1->alpha);
    htonf(net1->beta);

    htonf(net1->phidot);
    htonf(net1->thetadot);
    htonf(net1->psidot);
    htonf(net1->vcas);
    htonf(net1->climb_rate);
    htonf(net1->v_north);
    htonf(net1->v_east);
    htonf(net1->v_down);
    htonf(net1->v_body_u);
    htonf(net1->v_body_v);
    htonf(net1->v_body_w);

    htonf(net1->A_X_pilot);
    htonf(net1->A_Y_pilot);
    htonf(net1->A_Z_pilot);

    htonf(net1->stall_warning);
    htonf(net1->slip_deg);

    for (i=0; i < net1->num_engines; ++i) {
      net1->eng_state[i] = htonl(net1->eng_state[i]);
      htonf(net1->rpm[i]);
      htonf(net1->fuel_flow[i]);
      htonf(net1->fuel_px[i]);
      htonf(net1->egt[i]);
      htonf(net1->cht[i]);
      htonf(net1->mp_osi[i]);
      htonf(net1->tit[i]);
      htonf(net1->oil_temp[i]);
      htonf(net1->oil_px[i]);
    }
    net1->num_engines = htonl(net1->num_engines);

    for (i=0; i < net1->num_tanks; ++i) {
      htonf(net1->fuel_quantity[i]);
    }
    net1->num_tanks = htonl(net1->num_tanks);

    for (i=0; i < net3->num_wheels; ++i) {
      net3->wow[i] = htonl(net3->wow[i]);
      htonf(net3->gear_pos[i]);
      htonf(net3->gear_steer[i]);
      htonf(net3->gear_compression[i]);
    }
    net3->num_wheels = htonl(net3->num_wheels);

    net3->cur_time = htonl(net3->cur_time);
    net3->warp = htonl(net3->warp);
    htonf(net3->visibility);

    htonf(net3->elevator);
    htonf(net3->elevator_trim_tab);
    htonf(net3->left_flap);
    htonf(net3->right_flap);
    htonf(net3->left_aileron);
    htonf(net3->right_aileron);
    htonf(net3->rudder);
    htonf(net3->nose_wheel);
    htonf(net3->speedbrake);
    htonf(net3->spoilers);
  }
}
} // namespace JSBSim
