#!/bin/bash

ROBOTS=(
    rmf_obelix_1
    rmf_obelix_2
    rmf_obelix_3
)

for ROBOT in "${ROBOTS[@]}"; do
    echo "Initializing $ROBOT..."
    rosservice call /${ROBOT}/pci_initialization_trigger

    echo "Starting planner for $ROBOT..."
    rosservice call /${ROBOT}/planner_control_interface/std_srvs/automatic_planning
done
