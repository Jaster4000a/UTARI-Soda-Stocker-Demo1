#!/bin/bash
python bottle_stocker_v3.py &
python bottle_subscriber_v2.py &
python wave_move.py &
python blinking2.py &
trap 'kill $(jobs -p)' SIGINT
wait




