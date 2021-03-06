#! /bin/bash

# go - start the robot control loop
# load calibration data from imt2.cal too

# InMotion2 robot system software

# Copyright 2003-2014 Interactive Motion Technologies, Inc.
# Watertown, MA, USA
# http://www.interactive-motion.com
# All rights reserved

PERSONALITY=$(cat /opt/imt/personality)

if [[ ! -d "$CROB_HOME" ]]; then
    echo "CROB_HOME directory $CROB_HOME does not exist."
    exit 1
fi

zmsg() {
    $CROB_HOME/tools/zenity_wrap "$@"
}

cd $CROB_HOME

go_body() {

    if ! ./checkexist; then
        exit 1
    fi

    if pkill -0 -x robot; then
        zmsg "go: old robot still running, stopping." --warning
        if ! ./stop; then
            zmsg "go: couldn't stop old robot, exiting." --error
            exit 1
        fi
    fi

    if [[ $PERSONALITY == ce ]]; then
        if ! $CROB_HOME/tools/plc check-plc; then
            zmsg "go: PLC is not running, exiting." --error
            exit 1
        fi

        $CROB_HOME/tools/plc set-drive-en
    fi

    if [[ $PERSONALITY == g2 ]]; then
	if lsusb | grep -q "Bus 002 Device 002: ID 8087:0024"; then
	    zmsg "go: BIOS error. USB EHCI 2 must be disabled in the computer BIOS. Check CMOS battery. Call IMT." --error
	    exit 1
	fi

        $CROB_HOME/tools/ucplc -q check-estop
        ret=$?

        case $ret in
            0)
                $CROB_HOME/tools/ucplc -q set-active-en
                ;;
            1)
                zmsg "go: Stop button is set. Release the stop button to use the robot." --error
		exit 1
                ;;
            255)
                zmsg "go: Failed to communicate with microcontroller, exiting." --error
                exit 1
                ;;
            *)
                zmsg "go: Got unknown response from microcontroller, exiting." --error
                exit 1
                ;;
        esac

        surge_ret=$($CROB_HOME/tools/ucplc check-surge)
        if [[ ("$surge_ret" != "1.0") && ("$surge_ret" != "NA") ]]; then
            zmsg "The robot surge protector has failed.\n\nYou may continue to use the robot, but please contact IMT Support." --warning
        fi

        if [[ $($CROB_HOME/tools/ucplc check-warm) != "0" ]]; then
            zmsg "The robot got above normal operating temperature recently.\n\nYou may continue to use the robot, but please check the fans and the room temperature. If the temperature rises too high, the robot and computer may shut down." --warning
        fi
	$CROB_HOME/tools/ucplc clear-warm -q

    fi

    ./robot

    go_wait_for_robot

    ./shm < $IMT_CONFIG/robots/$(cat $IMT_CONFIG/current_robot)/imt2.cal

    sleep 0.1
}

go_uei_check() {
    nboards=$($CROB_HOME/tools/vget n_ueidaq_boards)
}

go_net_ft() {
    sleep 0.1

    # start the atinetft daemon
    if [[ -e $IMT_CONFIG/have_atinetft ]]; then
	CPF=$(curl --connect-timeout 2 -s -o - http://atinetft/netftapi2.xml|xpath -q -e "/netft/cfgcpf/text()[1]" 2> /dev/null)
	if [[ $? != 0 ]]; then
	    zmsg "go: ATI NetFT not found" --error
	    ./stop
	    exit 1
	fi
	CPT=$(curl -s -o - http://atinetft/netftapi2.xml|xpath -q -e "/netft/cfgcpt/text()[1]")

	if ! ./atinetft $CPF $CPT ; then
	    zmsg "go: ATI NetFT failed: $CPF $CPT" --error
	    ./stop
	    exit 1
	fi
    fi
}

go_wait_for_robot() {
    # make sure the robot program starts.
    # if not, try asking 3 times.

    i=0
    # while ! pkill -0 -x robot ; do
    while [[ ! -w /proc/xenomai/registry/native/pipes/crob_in ]]; do
        if [[ $i -gt 20 ]]; then
            zmsg "go: could not start robot" --error
            ./stop
            exit 1
        fi
        ((i++))
        sleep 0.1
    done
}

go_mcc() {
    if [[ ! -e $IMT_CONFIG/have_research_mcc ]]; then
	$ROBOT_HOME/mcc/c/mccd
    fi
}

go_ce() {
    # make sure that the number of uei boards is at least 2 after loading.
    # if not, try 3 times.

    nb_tries=0

    while true; do
        go_body
        go_uei_check
        ((nb_tries++))
        if [[ $nboards -ge 2 ]]; then
            break
        elif [[ $nb_tries -lt 3 ]]; then
            echo go: uei_check nb_tries=$nb_tries trying again
            date
            ./stop
        else
            zmsg "go: uei_check failed 3 times, stopping." --error
            ./stop
            exit 1
        fi
    done
}

go_g2() {
    go_body
    go_net_ft
    go_mcc
    ./notifyerror -d
}

if [[ $PERSONALITY == ce ]]; then
    go_ce
elif [[ $PERSONALITY == g2 ]]; then
    go_g2
else
    zmsg "go: unknown system type: $PERSONALITY" --error
    exit 1
fi

echo $(date -u "+%Y-%m-%d %H:%M:%S (%s go)") >> /var/log/imt/timestamps
