#!/bin/bash

# Script per lanciare la versione SDK del robot Spot
# Attiva il virtual environment 'spot_env' e lancia l'algoritmo di esplorazione

# Colori per output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}╔════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     SPOT SDK EXPLORATION LAUNCHER          ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════╝${NC}\n"

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$SCRIPT_DIR"

echo -e "${YELLOW}[INFO]${NC} Project root: ${PROJECT_ROOT}"

# Define virtual environment path
VENV_PATH="${HOME}/spot_env"

# Check if virtual environment exists
if [ ! -d "$VENV_PATH" ]; then
    echo -e "${YELLOW}[INFO]${NC} Virtual environment 'spot_env' not found at ${VENV_PATH}"
    echo -e "${YELLOW}[INFO]${NC} Creating virtual environment..."

    # Create virtual environment
    python3 -m venv "$VENV_PATH"

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[OK]${NC} Virtual environment created successfully"
    else
        echo -e "${RED}[ERROR]${NC} Failed to create virtual environment"
        exit 1
    fi
fi

# Activate virtual environment
echo -e "${YELLOW}[INFO]${NC} Activating virtual environment..."
source "$VENV_PATH/bin/activate"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}[OK]${NC} Virtual environment activated: $VIRTUAL_ENV"
else
    echo -e "${RED}[ERROR]${NC} Failed to activate virtual environment"
    exit 1
fi

# Check if install flag is passed
INSTALL_DEPS=false
if [ "$1" == "--install" ] || [ "$1" == "-i" ]; then
    INSTALL_DEPS=true
    echo -e "${YELLOW}[INFO]${NC} Installation mode enabled"
fi

# Install/upgrade essential packages only if requested
if [ "$INSTALL_DEPS" = true ]; then
    echo -e "${YELLOW}[INFO]${NC} Upgrading pip, setuptools and wheel..."
    pip install --upgrade pip setuptools wheel --quiet

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[OK]${NC} Essential packages upgraded"
    else
        echo -e "${RED}[ERROR]${NC} Failed to upgrade essential packages"
        exit 1
    fi

    # Install other common dependencies
    echo -e "${YELLOW}[INFO]${NC} Installing additional dependencies..."
    pip install numpy opencv-python pyyaml --quiet

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[OK]${NC} Additional dependencies installed successfully"
    else
        echo -e "${YELLOW}[WARNING]${NC} Some dependencies may have failed to install"
    fi

    # Display instructions for bosdyn
    echo -e "${YELLOW}[INFO]${NC} Boston Dynamics Spot SDK (bosdyn) must be installed manually"
    echo -e "${YELLOW}[INFO]${NC} bosdyn is not available on PyPI and requires special installation"

else
    echo -e "${BLUE}[INFO]${NC} Skipping dependency installation (use --install or -i flag to install)"
fi

# Verify essential packages are available
echo -e "${YELLOW}[INFO]${NC} Verifying required packages..."

# Check for bosdyn
if python3 -c "import bosdyn" 2>/dev/null; then
    echo -e "${GREEN}[OK]${NC} Boston Dynamics Spot SDK (bosdyn) found"
else
    echo -e "${RED}[ERROR]${NC} Boston Dynamics Spot SDK (bosdyn) not found in virtual environment"
    echo -e "${YELLOW}[INFO]${NC} Please install bosdyn manually or consult the project documentation"
    echo -e "${YELLOW}[INFO]${NC} To install: pip install bosdyn-core bosdyn-client bosdyn-mission"
    exit 1
fi

# Check for other key packages
for pkg in numpy yaml; do
    if python3 -c "import $pkg" 2>/dev/null; then
        echo -e "${GREEN}[OK]${NC} Package '$pkg' found"
    else
        echo -e "${YELLOW}[WARNING]${NC} Package '$pkg' not found"
    fi
done

# Change to project directory
cd "$PROJECT_ROOT"

if [ $? -ne 0 ]; then
    echo -e "${RED}[ERROR]${NC} Failed to change to project directory"
    exit 1
fi

echo -e "${GREEN}[OK]${NC} Changed to project directory: $PWD"

# Add project root to PYTHONPATH
export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH}"
echo -e "${YELLOW}[INFO]${NC} PYTHONPATH updated: $PYTHONPATH"

# Display Python and pip information
echo -e "\n${BLUE}════════════════════════════════════════════${NC}"
echo -e "${BLUE}Environment Information:${NC}"
echo -e "${BLUE}════════════════════════════════════════════${NC}"
echo "Python: $(python3 --version)"
echo "Pip: $(pip --version)"
echo "Virtual Env: $VIRTUAL_ENV"
echo "PWD: $PWD"
echo -e "${BLUE}════════════════════════════════════════════${NC}\n"

# Launch the exploration SDK
echo -e "${YELLOW}[INFO]${NC} Launching exploration algorithm (SDK version)..."
echo -e "${BLUE}════════════════════════════════════════════${NC}\n"

# Run the main exploration script with unbuffered output to see prints immediately
python3 -u -m entry_points.run_exploration_sdk

# Capture the exit code
EXIT_CODE=$?

echo -e "\n${BLUE}════════════════════════════════════════════${NC}"
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}[OK]${NC} Exploration completed successfully"
else
    echo -e "${RED}[ERROR]${NC} Exploration exited with code $EXIT_CODE"
fi
echo -e "${BLUE}════════════════════════════════════════════${NC}"

exit $EXIT_CODE

