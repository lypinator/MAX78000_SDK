echo "\nMAX78000_SDK auto-setup script\n"

echo "Installing packages..."
# Install packages
apt update
apt upgrade
apt install --yes make libncurses5 libusb-1.0-0 libusb-0.1-4 libhidapi-libusb0 libhidapi-hidraw0

# Get user's home directory
HOME_DIR=$(getent passwd ${SUDO_USER:-$USER} | cut -d: -f6) # This should work if this script is run with sudo or normally
echo "Located user home directory $HOME_DIR"

# Get location of script
BASEDIR=$(dirname $0)
MAXIM_PATH="$(cd $BASEDIR && pwd)"

# Add MAXIM_PATH to shell startup script
if [ -z "$(grep "export MAXIM_PATH=" $HOME_DIR/.bashrc)" ]; # If grep search for MAXIM_PATH in ~/.bashrc returns empty string (-z)
then
	echo "Setting MAXIM_PATH=$MAXIM_PATH in $HOME_DIR/.bashrc"
	sed -i~ "1iexport MAXIM_PATH=${MAXIM_PATH}" $HOME_DIR/.bashrc # Insert line at beginning (1i) and create backup (-i~)
else
	# Update MAXIM_PATH instead of adding new line
	echo "$HOME_DIR/.bashrc already sets MAXIM_PATH.  Updating to MAXIM_PATH=$MAXIM_PATH just in case"
	sed -i~ "s:.*export MAXIM_PATH=.*:export MAXIM_PATH=$MAXIM_PATH:" $HOME_DIR/.bashrc
fi

# Copy OpenOCD rules file
echo "Copying OpenOCD .rules file to /etc/udev/rules.d/"
cp $MAXIM_PATH/Tools/OpenOCD/60-openocd.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger --attr-match=subsystem=net

# TODO: Call user-side VS Code generator script?
