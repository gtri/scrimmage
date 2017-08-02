#!/bin/bash 
#-------------------------------------------------------
#  Part 1: Check for and handle command-line arguments
#-------------------------------------------------------
TIME_WARP=1
JUST_MAKE="no"
VARIATION="0"
for ARGI; do
    if [ "${ARGI}" = "--help" -o "${ARGI}" = "-h" ] ; then
	printf "%s [SWITCHES] [time_warp]   \n" $0
	printf "  --just_make, -j    \n" 
	printf "  --help, -h         \n" 
	exit 0;
    elif [ "${ARGI//[^0-9]/}" = "$ARGI" -a "$TIME_WARP" = 1 ]; then 
        TIME_WARP=$ARGI
    elif [ "${ARGI}" = "v1" ] ; then
	VARIATION="1"
    elif [ "${ARGI}" = "v2" ] ; then
	VARIATION="2"
    elif [ "${ARGI}" = "v3" ] ; then
	VARIATION="3"
    elif [ "${ARGI}" = "--just_make" -o "${ARGI}" = "-j" ] ; then
	JUST_MAKE="yes"
    else 
	printf "Bad Argument: %s \n" $ARGI
	exit 0
    fi
done

#-------------------------------------------------------
#  Part 2: Create the .moos and .bhv files. 
#-------------------------------------------------------
VNAME1="henry"           # The first vehicle Community
VNAME2="gilda"           # The second vehicle Community
START_POS1="0,0"         
START_POS2="80,0"        
LOITER_POS1="x=0,y=-75"
LOITER_POS2="x=125,y=-50"
SHORE_LISTEN="9300"

nsplug meta_vehicle.moos targ_henry.moos -f WARP=$TIME_WARP \
    VNAME=$VNAME1          SHARE_LISTEN="9301"              \
    VPORT="9001"           SHORE_LISTEN=$SHORE_LISTEN       \
    START_POS=$START_POS1  VARIATION=$VARIATION             

nsplug meta_vehicle.moos targ_gilda.moos -f WARP=$TIME_WARP \
    VNAME=$VNAME2          SHARE_LISTEN="9302"              \
    VPORT="9002"           SHORE_LISTEN=$SHORE_LISTEN       \
    START_POS=$START_POS2  VARIATION=$VARIATION             

nsplug meta_shoreside.moos targ_shoreside.moos -f WARP=$TIME_WARP \
    SNAME="shoreside"  SHARE_LISTEN=$SHORE_LISTEN                 \
    SPORT="9000"       VARIATION=$VARIATION         

nsplug meta_vehicle.bhv targ_henry.bhv -f VNAME=$VNAME1     \
    START_POS=$START_POS1 LOITER_POS=$LOITER_POS1       

nsplug meta_vehicle.bhv targ_gilda.bhv -f VNAME=$VNAME2     \
    START_POS=$START_POS2 LOITER_POS=$LOITER_POS2       

if [ ! -e targ_henry.moos ]; then echo "no targ_henry.moos"; exit; fi
if [ ! -e targ_henry.bhv  ]; then echo "no targ_henry.bhv "; exit; fi
if [ ! -e targ_gilda.moos ]; then echo "no targ_gilda.moos"; exit; fi
if [ ! -e targ_gilda.bhv  ]; then echo "no targ_gilda.bhv "; exit; fi
if [ ! -e targ_shoreside.moos ]; then echo "no targ_shoreside.moos";  exit; fi

if [ ${JUST_MAKE} = "yes" ] ; then
    exit 0
fi


#-------------------------------------------------------
#  Part 3: Launch the processes
#-------------------------------------------------------
printf "Launching $SNAME MOOS Community (WARP=%s) \n"  $TIME_WARP
pAntler targ_shoreside.moos >& /dev/null &
printf "Launching $VNAME1 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_henry.moos >& /dev/null &
printf "Launching $VNAME2 MOOS Community (WARP=%s) \n" $TIME_WARP
pAntler targ_gilda.moos >& /dev/null &
printf "Done \n"

uMAC targ_shoreside.moos

printf "Killing all processes ... \n"
mykill
printf "Done killing processes.   \n"
