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
    memset(&fgSockBufMod, 0x0, sizeof(fgSockBufMod));
}

void FGOutputFGMod::Print(void) {
  int length = sizeof(fgSockBufMod);

  if (socket == 0) return;
  if (!socket->GetConnectStatus()) return;

  SocketDataFillMod(&fgSockBufMod);
  socket->Send((char *)&fgSockBufMod, length); // NOLINT
}

void FGOutputFGMod::SocketDataFillMod(FGNetFDM* net) {
  unsigned int i;

  // Version
  net->version = FG_NET_FDM_VERSION;

  // Positions
  net->longitude = Propagate->GetLocation().GetLongitude(); //
  net->latitude  = Propagate->GetLocation().GetLatitude(); // GeodLatitudeRad(); // geodetic (radians)
  net->altitude  = Propagate->GetAltitudeASL()*0.3048; // altitude, above sea level (meters)
  net->agl       = static_cast<float>(Propagate->GetDistanceAGL()*0.3048); // altitude, above ground level (meters)

  net->phi       = static_cast<float>(Propagate->GetEuler(ePhi)); // roll (radians)
  net->theta     = static_cast<float>(Propagate->GetEuler(eTht)); // pitch (radians)
  net->psi       = static_cast<float>(Propagate->GetEuler(ePsi)); // yaw or true heading (radians)

  net->alpha     = static_cast<float>(Auxiliary->Getalpha()); // angle of attack (radians)
  net->beta      = static_cast<float>(Auxiliary->Getbeta()); // side slip angle (radians)

  // Velocities
  net->phidot     = static_cast<float>(Auxiliary->GetEulerRates(ePhi)); // roll rate (radians/sec)
  net->thetadot   = static_cast<float>(Auxiliary->GetEulerRates(eTht)); // pitch rate (radians/sec)
  net->psidot     = static_cast<float>(Auxiliary->GetEulerRates(ePsi)); // yaw rate (radians/sec)
  net->vcas       = static_cast<float>(Auxiliary->GetVcalibratedKTS()); // VCAS, knots
  net->climb_rate = static_cast<float>(Propagate->Gethdot());           // altitude rate, ft/sec
  net->v_north    = static_cast<float>(Propagate->GetVel(eNorth));      // north vel in NED frame, fps
  net->v_east     = static_cast<float>(Propagate->GetVel(eEast));       // east vel in NED frame, fps
  net->v_down     = static_cast<float>(Propagate->GetVel(eDown));       // down vel in NED frame, fps
  // ---ADD METHOD TO CALCULATE THESE TERMS---
  net->v_body_u = static_cast<float>(Propagate->GetUVW(1)); // ECEF speed in body axis
  net->v_body_v = static_cast<float>(Propagate->GetUVW(2)); // ECEF speed in body axis
  net->v_body_w = static_cast<float>(Propagate->GetUVW(3)); // ECEF speed in body axis

  // Accelerations
  net->A_X_pilot   = static_cast<float>(Auxiliary->GetPilotAccel(1));    // X body accel, ft/s/s
  net->A_Y_pilot   = static_cast<float>(Auxiliary->GetPilotAccel(2));    // Y body accel, ft/s/s
  net->A_Z_pilot   = static_cast<float>(Auxiliary->GetPilotAccel(3));    // Z body accel, ft/s/s

  // Stall
  net->stall_warning = 0.0;  // 0.0 - 1.0 indicating the amount of stall
  net->slip_deg    = static_cast<float>(Auxiliary->Getbeta(inDegrees));  // slip ball deflection, deg

  // Engine status
  if (Propulsion->GetNumEngines() > FGNetFDM::FG_MAX_ENGINES && FDMExec->GetSimTime() == 0.0)
    cerr << "This vehicle has " << Propulsion->GetNumEngines() << " engines, but the current " << endl
         << "version of FlightGear's FGNetFDM only supports " << FGNetFDM::FG_MAX_ENGINES << " engines." << endl
         << "Only the first " << FGNetFDM::FG_MAX_ENGINES << " engines will be used." << endl;

  net->num_engines = std::min(static_cast<unsigned int>(FGNetFDM::FG_MAX_ENGINES),
                              Propulsion->GetNumEngines()); // Number of valid engines

  for (i=0; i < net->num_engines; i++) {
    if (Propulsion->GetEngine(i)->GetRunning())
      net->eng_state[i] = 2;       // Engine state running
    else if (Propulsion->GetEngine(i)->GetCranking())
      net->eng_state[i] = 1;       // Engine state cranking
    else
      net->eng_state[i] = 0;       // Engine state off

    switch (Propulsion->GetEngine(i)->GetType()) {
    case (FGEngine::etRocket):
        break;
    case (FGEngine::etPiston):
        net->rpm[i]       = static_cast<float>((static_cast<FGPiston *>(Propulsion->GetEngine(i))->getRPM()));
        net->fuel_flow[i] = static_cast<float>(((static_cast<FGPiston *>(Propulsion->GetEngine(i))->getFuelFlow_gph())));
        net->fuel_px[i]   = 0; // Fuel pressure, psi  (N/A in current model)
        net->egt[i]       = static_cast<float>(((static_cast<FGPiston *>(Propulsion->GetEngine(i))->GetEGT())));
        net->cht[i]       = static_cast<float>(((static_cast<FGPiston *>(Propulsion->GetEngine(i))->getCylinderHeadTemp_degF())));
        net->mp_osi[i]    = static_cast<float>(((static_cast<FGPiston *>(Propulsion->GetEngine(i))->getManifoldPressure_inHg())));
        net->oil_temp[i]  = static_cast<float>(((static_cast<FGPiston *>(Propulsion->GetEngine(i))->getOilTemp_degF())));
        net->oil_px[i]    = static_cast<float>(((static_cast<FGPiston *>(Propulsion->GetEngine(i))->getOilPressure_psi())));
        net->tit[i]       = 0; // Turbine Inlet Temperature  (N/A for piston)
        break;
    case (FGEngine::etTurbine):
        break;
    case (FGEngine::etTurboprop):
        break;
    case (FGEngine::etElectric):
        break;
    case (FGEngine::etUnknown):
        break;
    }
  }

  // Consumables
  if (Propulsion->GetNumTanks() > FGNetFDM::FG_MAX_TANKS && FDMExec->GetSimTime() == 0.0)
    cerr << "This vehicle has " << Propulsion->GetNumTanks() << " tanks, but the current " << endl
         << "version of FlightGear's FGNetFDM only supports " << FGNetFDM::FG_MAX_TANKS << " tanks." << endl
         << "Only the first " << FGNetFDM::FG_MAX_TANKS << " tanks will be used." << endl;

  net->num_tanks = std::min(static_cast<unsigned int>(FGNetFDM::FG_MAX_TANKS),
                            Propulsion->GetNumTanks());   // Max number of fuel tanks

  for (i=0; i < net->num_tanks; i++) {
      net->fuel_quantity[i] = static_cast<float>((static_cast<FGTank *>(Propulsion->GetTank(i))->GetContents()));
  }

  // Gear status
  if (GroundReactions->GetNumGearUnits() > FGNetFDM::FG_MAX_WHEELS && FDMExec->GetSimTime() == 0.0)
    cerr << "This vehicle has " << GroundReactions->GetNumGearUnits() << " bogeys, but the current " << endl
         << "version of FlightGear's FGNetFDM only supports " << FGNetFDM::FG_MAX_WHEELS << " bogeys." << endl
         << "Only the first " << FGNetFDM::FG_MAX_WHEELS << " bogeys will be used." << endl;

  net->num_wheels  = std::min(static_cast<int>(FGNetFDM::FG_MAX_WHEELS),
                              GroundReactions->GetNumGearUnits());

  for (i=0; i < net->num_wheels; i++) {
    net->wow[i]              = GroundReactions->GetGearUnit(i)->GetWOW();
    if (GroundReactions->GetGearUnit(i)->GetGearUnitDown())
      net->gear_pos[i]      = 1;  // gear down, using FCS convention
    else
      net->gear_pos[i]      = 0;  // gear up, using FCS convention
    net->gear_steer[i]       = static_cast<float>(GroundReactions->GetGearUnit(i)->GetSteerNorm());
    net->gear_compression[i] = static_cast<float>(GroundReactions->GetGearUnit(i)->GetCompLen());

    net->gear_pos[i] = 0; // MOD: Force gear up
  }

  // Environment
  net->cur_time    = static_cast<uint64_t>(1234567890);    // Friday, Feb 13, 2009, 23:31:30 UTC (not processed by FGFS anyway)
  net->warp        = 0;                       // offset in seconds to unix time
  net->visibility  = 25000.0;                 // visibility in meters (for env. effects)

  // Control surface positions (normalized values)
  net->elevator          = static_cast<float>(FCS->GetDePos(ofNorm));    // Norm Elevator Pos, --
  net->elevator_trim_tab = static_cast<float>(FCS->GetPitchTrimCmd());   // Norm Elev Trim Tab Pos, --
  net->left_flap         = static_cast<float>(FCS->GetDfPos(ofNorm));    // Norm Flap Pos, --
  net->right_flap        = static_cast<float>(FCS->GetDfPos(ofNorm));    // Norm Flap Pos, --
  net->left_aileron      = static_cast<float>(FCS->GetDaLPos(ofNorm));   // Norm L Aileron Pos, --
  net->right_aileron     = static_cast<float>(FCS->GetDaRPos(ofNorm));   // Norm R Aileron Pos, --
  net->rudder            = static_cast<float>(FCS->GetDrPos(ofNorm));    // Norm Rudder Pos, --
  net->nose_wheel        = static_cast<float>(FCS->GetDrPos(ofNorm));    // *** FIX ***  Using Rudder Pos for NWS, --
  net->speedbrake        = static_cast<float>(FCS->GetDsbPos(ofNorm));   // Norm Speedbrake Pos, --
  net->spoilers          = static_cast<float>(FCS->GetDspPos(ofNorm));   // Norm Spoiler Pos, --

  // Convert the net buffer to network format
  if ( isLittleEndian ) {
    net->version = htonl(net->version);

    htond(net->longitude);
    htond(net->latitude);
    htond(net->altitude);
    htonf(net->agl);
    htonf(net->phi);
    htonf(net->theta);
    htonf(net->psi);
    htonf(net->alpha);
    htonf(net->beta);

    htonf(net->phidot);
    htonf(net->thetadot);
    htonf(net->psidot);
    htonf(net->vcas);
    htonf(net->climb_rate);
    htonf(net->v_north);
    htonf(net->v_east);
    htonf(net->v_down);
    htonf(net->v_body_u);
    htonf(net->v_body_v);
    htonf(net->v_body_w);

    htonf(net->A_X_pilot);
    htonf(net->A_Y_pilot);
    htonf(net->A_Z_pilot);

    htonf(net->stall_warning);
    htonf(net->slip_deg);

    for (i=0; i < net->num_engines; ++i) {
      net->eng_state[i] = htonl(net->eng_state[i]);
      htonf(net->rpm[i]);
      htonf(net->fuel_flow[i]);
      htonf(net->fuel_px[i]);
      htonf(net->egt[i]);
      htonf(net->cht[i]);
      htonf(net->mp_osi[i]);
      htonf(net->tit[i]);
      htonf(net->oil_temp[i]);
      htonf(net->oil_px[i]);
    }
    net->num_engines = htonl(net->num_engines);

    for (i=0; i < net->num_tanks; ++i) {
      htonf(net->fuel_quantity[i]);
    }
    net->num_tanks = htonl(net->num_tanks);

    for (i=0; i < net->num_wheels; ++i) {
      net->wow[i] = htonl(net->wow[i]);
      htonf(net->gear_pos[i]);
      htonf(net->gear_steer[i]);
      htonf(net->gear_compression[i]);
    }
    net->num_wheels = htonl(net->num_wheels);

    net->cur_time = htonl(net->cur_time);
    net->warp = htonl(net->warp);
    htonf(net->visibility);

    htonf(net->elevator);
    htonf(net->elevator_trim_tab);
    htonf(net->left_flap);
    htonf(net->right_flap);
    htonf(net->left_aileron);
    htonf(net->right_aileron);
    htonf(net->rudder);
    htonf(net->nose_wheel);
    htonf(net->speedbrake);
    htonf(net->spoilers);
  }
}
} // namespace JSBSim
