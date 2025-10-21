#!/bin/bash

# Activate virtual environment
source .venv/bin/activate

# Install required packages
pip install -r requirements.txt

# Run the custom top-down car demo
python simple_sim.py

