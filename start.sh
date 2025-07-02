#!/bin/bash

# Start containers in the background
docker-compose up --build -d

# Wait a few seconds to ensure services are up
sleep 3

# Open frontend in browser
xdg-open http://localhost 