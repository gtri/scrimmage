#!/bin/bash

usage()
{
    cat << EOF
USAGE: 
          $0 [OPTIONS]

NO OPTIONS:

               Without specifying any options, the script displays the
               scrimmage files found on the system.

OPTIONS:
          -r   Remove scrimmage files from system
          -p   Specific the system prefix to search (default: /usr/local)
          -h   Display this help message

EOF
}

PREFIX=/usr/local
REMOVE_FILES=false
while getopts ":rp:h" opt; do
    case $opt in
        h)
            usage
            exit 0
            ;;
        p)
            PREFIX=${OPTARG}
            ;;
        r)
            REMOVE_FILES=true
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            exit 1
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            usage
            exit 1
            ;;
    esac
done

if [ $REMOVE_FILES == true ]; then
    find ${PREFIX}/bin -iname "*scrimmage*" | xargs sudo rm -rf
    find ${PREFIX}/etc -iname "*scrimmage*" | xargs sudo rm -rf
    find ${PREFIX}/include -iname "*scrimmage*" | xargs sudo rm -rf
    find ${PREFIX}/lib -iname "*scrimmage*" | xargs sudo rm -rf        
else
    find ${PREFIX}/bin -iname "*scrimmage*"
    find ${PREFIX}/etc -iname "*scrimmage*"
    find ${PREFIX}/include -iname "*scrimmage*"
    find ${PREFIX}/lib -iname "*scrimmage*"
    #find ${PREFIX}/share -iname "*scrimmage*"
fi
