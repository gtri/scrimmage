#!/bin/bash

usage()
{
    cat << EOF

USAGE:
          $0 [OPTIONS]

OPTIONS:
          -h               Display this usage information
          -m               Connect to multiplayer server
          -i [IP_ADDRESS]  Multiplayer IP address
          -p [PORT]        Multiplayer port
          -c [CALLSIGN]    Callsign to use for multiplayer server
          -a [AIRCRAFT]    FlightGear aircraft visual model

EOF
}

ENABLE_MULTIPLAYER=false
SERVER_IP="127.0.0.1"
SERVER_PORT=5000
RECEIVE_PORT=6000
CALLSIGN="player1"
AIRCRAFT="c172p-2dpanel"
while getopts ":p:i:c:a:mh" opt; do
    case $opt in
        h)
            usage
            exit 0
            ;;
        m)
            ENABLE_MULTIPLAYER=true
            MISSION_FILE_DIR=$OPTARG
            ;;
        p)
            SERVER_PORT=$OPTARG
            ;;
        i)
            SERVER_IP=$OPTARG
            ;;
        c)
            CALLSIGN=$OPTARG
            ;;
        a)
            AIRCRAFT=$OPTARG
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            exit 1
            ;;
    esac
done

CMD_STR="fgfs --fg-root=/usr/share/games/flightgear --fg-scenery=/usr/share/games/flightgear/Scenery --airport=KSFO --aircraft=${AIRCRAFT} --native-fdm=socket,out,60,,5500,udp --fdm=null --native-fdm=socket,in,60,,5600,udp --units-meters"

if [ $ENABLE_MULTIPLAYER == true ]; then
    CMD_STR="${CMD_STR} --callsign=${CALLSIGN} --multiplay=out,10,${SERVER_IP},${SERVER_PORT} --multiplay=in,10,,${RECEIVE_PORT}"
fi

echo "Executing command..."
echo ${CMD_STR}

${CMD_STR}

exit 0

#fgfs --airport=ATL \
#     --aircraft=Rascal110-JSBSim \
#     --native-fdm=socket,out,60,,5500,udp \
#     --fdm=null \
#     --native-fdm=socket,in,60,,5600,udp \
#     --units-meters \
#     --callsign=${CALLSIGN} \
#     --multiplay=out,10,${SERVER_IP},${SERVER_PORT} \
#     --multiplay=in,10,,${SERVER_PORT}
