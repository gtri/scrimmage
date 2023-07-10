==================================
Mission to Mission XML Generation
==================================

This tutorial covers the process of capturing the end states of entities in
an XML file, which can be used as starting points for future simulations. The 
following must be completed before the mission to mission capabilities can be
utilized:

1. In the mission XML file, the following tag must be included with the value ``true``::

        <mission_to_mission>true</mission_to_mission>

2. In the mission XML file's entity block, if the block should be included in the output
   mission XML file - meaning future simulations will require the block, the following tag
   must be included with the value ``true``. If it is not included, the entity block will be removed
   from the output mission XML file. ::

        <remove_block>true</remove_block>

3. If plugin specific XML tag attributes (applicable to motion, sensor, autonomy, and controller plugins) 
   are expected to be updated while the simulation is running, the following function declaration will need to 
   be added to the plugin's header file::

        std::map<std::string,std::string> mission_xml_get() override;

   The mission_xml_get function must insert plugin specific xml tags as strings to a map. Depending on the variable
   type of the xml tag, extra formatting might be needed - for example: converting  a bool to a string results in 
   "0" or "1," which will need to be converted to "true" or "false."

   Here is an example of the mission_xml_get, implemented in the SimpleAircraft.cpp file::

        std::map<std::string,std::string> SimpleAircraft::mission_xml_get() {
            std::map<std::string,std::string> mission_xml;
        
            mission_xml.insert({"Name","SimpleAircraft"});
            mission_xml.insert({"min_velocity",std::to_string(min_velocity_)});
            mission_xml.insert({"max_velocity",std::to_string(max_velocity_)});
            mission_xml.insert({"max_roll",std::to_string(max_roll_)});
            mission_xml.insert({"max_roll_rate",std::to_string(max_roll_rate_)});
            mission_xml.insert({"max_pitch",std::to_string(max_pitch_)});
            mission_xml.insert({"max_pitch_rate",std::to_string(max_pitch_rate_)});
            mission_xml.insert({"speed_target",std::to_string(speedTarget_)});
            mission_xml.insert({"radius_slope_per_speed",std::to_string(lengthSlopePerSpeed_)});
            mission_xml.insert({"turning_radius",std::to_string(length_)});
        
            return mission_xml;
        }

   Note that the plugin name must be specified as the first entry in the map, with the key ``Name`` and the 
   value ``<plugin_name>``. In the map, each xml specific tag attribute name must be the key of the map and 
   the attribute value must be the value of the map.

There will be 2 output files, which can be found in the simulation's log directory: 
``~/.scrimmage/logs/<scrimmage_run_log_directory>``.

1. The ``mission_to_mission.xml`` file captures the final entity states and formats the data to be used as an
   input file to future SCRIMMAGE simulations.

2. The ``final_ent_states.txt`` file captures the final state information for each entity. These values can be
   used as reference to verify end states and capture values like velocity, which do not have entity specific tags.
   The following entity end states are included in the output text file:

    * Team_ID
    * X_Pos
    * Y_Pos
    * Z_Pos
    * Vel_X
    * Vel_Y
    * Vel_Z
    * Heading
    * Pitch
    * Roll
    * Altitude
    * Health