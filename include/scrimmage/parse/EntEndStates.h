#ifndef INCLUDE_SCRIMMAGE_PARSE_ENTENDSTATES_H_
#define INCLUDE_SCRIMMAGE_PARSE_ENTENDSTATES_H_

   
   struct ent_end_state {
      int team_id;

      double x_pos;
      double y_pos;
      double z_pos;

      double yaw;
      double pitch;
      double roll;

      int health_points;
   };

#endif