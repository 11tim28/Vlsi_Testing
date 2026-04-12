#!/bin/bash

# List of test names (without extensions)
tests=("s5378" "s9234" "s38417")  # Add all your testcases here

# Path to your simulator executable
sim="./3fsim"

# Loop over each test
for test in "${tests[@]}"; do
    echo "Running test: $test"
    
    blif_file="../data/${test}.blif"
    pat_file="../data/${test}.pat"
    out_file="../out/${test}.out"
    golden_file="../data/${test}.gold.out"
    
    # Run the simulator
    $sim "$blif_file" "$pat_file" "$out_file"
    
    # Check if the command succeeded
    if [ $? -eq 0 ]; then
        echo "Test $test finished successfully."
        echo "Comparing with golden..." 
        if diff "$out_file" "$golden_file" > /dev/null; then
            echo "Test $test PASSED: Output matches golden reference."
        else
            echo "Test $test FAILED: Output differs from golden reference."
            diff "$out_file" "$golden_file"  # show differences
        fi
    else
        echo "Test $test FAILED."
    fi

    echo "-----------------------------"
done