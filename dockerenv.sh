#!/usr/bin/env bash
# This script helps manage a SSH forwarding session for a Docker socket on the NUC
# To start or stop your session, run ./dockerenv.sh [start|stop]
# This script may also be sourced without arguments for use in other deployment scripts,
# which may depend on having Docker target the NUC

pushd $(dirname $BASH_SOURCE) > /dev/null

source robot.env

# I'm being run directly
if [ "$0" = "$BASH_SOURCE" ]
then
    case $@ in
        start)
            if [ -f ssh.pid ]
            then
                SSH_PID=$(cat ssh.pid)
                if ps -p $SSH_PID > /dev/null
                then
                    exit 1
                    break
                fi
            fi

            echo "Trying to forward Docker socket..."

            rm -f docker.sock
            ssh -nNT -L $(pwd)/docker.sock:/var/run/docker.sock ras@${ROBOT_IP} &
            SSH_PID=$!
            disown
            echo -n $SSH_PID > ssh.pid

            export DOCKER_HOST=unix://$(pwd)/docker.sock

            echo -n "Waiting for Docker socket to appear... "

            while true
            do
                if [ -e docker.sock ]
                then
                    break
                else
                    echo -n "."
                fi

                if ! ps -p $SSH_PID > /dev/null
                then
                    echo ""
                    echo "SSH client exited"
                    rm ssh.pid
                    exit 1
                    break
                fi

                sleep 0.2
            done
            echo " Started"
        ;;

        stop)
            if ! [ -f ssh.pid ]
            then
                echo "ssh.pid does not exist"
                exit 1
            fi

            kill $(cat ssh.pid)
            rm -f ssh.pid
            rm -f docker.sock
            echo "Stopped SSH tunnel"
        ;;

        *)
            echo "Usage: $0 [start|stop]"
            echo "After starting, this script can be sourced to configure DOCKER_HOST"
        ;;
    esac
# I'm being sourced
else
    if ! [ -f ssh.pid ]
    then
        echo "SSH tunnel not running"
        return 1
    fi
    export DOCKER_HOST=unix://$(pwd)/docker.sock
    return 0
fi

popd > /dev/null
