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

EOF
}

ENABLE_MULTIPLAYER=false
SERVER_IP="127.0.0.1"
SERVER_PORT=5000
CALLSIGN="player1"
while getopts ":p:i:c:mh" opt; do
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

CMD_STR="fgfs --airport=ATL --aircraft=Rascal110-JSBSim --native-fdm=socket,out,60,,5500,udp --fdm=null --native-fdm=socket,in,60,,5600,udp --units-meters"

if [ $ENABLE_MULTIPLAYER == true ]; then
    CMD_STR="${CMD_STR} --callsign=${CALLSIGN} --multiplay=out,10,${SERVER_IP},${SERVER_PORT} --multiplay=in,10,,${SERVER_PORT}"
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
