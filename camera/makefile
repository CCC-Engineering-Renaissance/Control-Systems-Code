# Variables
PYTHON = python3

# The 'all' target is what runs when you just type 'make'
all: run

# Target to run the ROV orchestrator
run:
	$(PYTHON) camera.py

# Target to clean up cache
clean:
	rm -rf __pycache__
	rm -f *.pyc

# Diagnostic tool to see connected cameras
list-cams:
	v4l2-ctl --list-devices
