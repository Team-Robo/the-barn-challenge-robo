#!/bin/bash

# Default parameters
DEFAULT_LAUNCH="move_base_DWA.launch"
START_IDX=0
SPACING=6
REPEAT=10         # number of times to run each world
MAX_IDX=359
LAUNCH_FILES=()
MONITOR=false     # resource monitoring flag
MNODES=()
MYAML=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        --launch)
            shift
            # Collect all non-option arguments as launch files
            while [[ $# -gt 0 && "$1" != --* ]]; do
                LAUNCH_FILES+=("$1")
                shift
            done
            ;;
        --start_idx)
            START_IDX="$2"
            shift 2
            ;;
        --spacing)
            SPACING="$2"
            shift 2
            ;;
        --repeat)
            REPEAT="$2"
            shift 2
            ;;
        --monitor)
            MONITOR=true
            shift
            ;;
        --mnodes)
            shift
            while [[ $# -gt 0 && "$1" != --* ]]; do
                MNODES+=("$1")
                shift
            done
            ;;
        --myaml)
            MYAML="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            shift
            ;;
    esac
done

# If no launch file provided, use default
if [ ${#LAUNCH_FILES[@]} -eq 0 ]; then
    echo "No launch file provided, using default: $DEFAULT_LAUNCH"
    LAUNCH_FILES+=("$DEFAULT_LAUNCH")
fi

# Loop over each launch file
for LAUNCH in "${LAUNCH_FILES[@]}"; do
    echo "Running tests with launch file: $LAUNCH"

    # Calculate number of worlds dynamically
    NUM_WORLDS=$(( (MAX_IDX - START_IDX) / SPACING + 1 ))

    for (( i=0; i<NUM_WORLDS; i++ )); do
        n=$(( START_IDX + i * SPACING ))

        for (( j=1; j<=REPEAT; j++ )); do
            echo "Running world index $n, test $j..."

            # Build the command as an array to handle quoting properly
            CMD=(python3 run.py --world_idx "$n" --launch "$LAUNCH")

            if [ "$MONITOR" = true ]; then
                CMD+=(--monitor)
            fi

            if [ ${#MNODES[@]} -gt 0 ]; then
                CMD+=(--mnodes "${MNODES[@]}")
            fi

            if [ -n "$MYAML" ]; then
                CMD+=(--myaml "$MYAML")
            fi

            # Run the command
            "${CMD[@]}"

            # Wait a bit before next run
            sleep 5
        done
    done
done

