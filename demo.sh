#!/bin/bash

# Get current IP address of the robot
current_ip=$(hostname -I | sed -r 's/\s+$//' | grep -oE '^[^[:space:]]+')

if [ -z "$current_ip" ]; then
    echo "Failed to get current IP address."
    exit 1
fi

upstart_status=$(ls /lib/systemd/system/ | grep rbl_upstart)

# Check if service exists
if [ -n "$upstart_status" ]; then
    echo "Found an existing upstart service, enabling it..."
    systemctl enable --user rbl_upstart.service > /dev/null 2>&1 && systemctl start --user rbl_upstart.service

    if [ $? -eq 0 ]; then
        echo "Upstart service enabled and started successfully, robot in demonstration mode."
    else
        echo "Failed to start upstart service."
        exit 1
    fi
else
    echo "Did not find an upstart service, creating a new one..."
    sudo cp -f services/rbl_upstart.service /etc/systemd/user/

    if [ $? -ne 0 ]; then
        echo "Failed to copy service file."
        exit 1
    fi

	systemctl --user enable rbl_upstart.service
	systemctl --user start rbl_upstart.service
	sudo systemctl daemon-reload

    if [ $? -eq 0 ]; then
        echo "Upstart service enabled and started successfully, robot in demonstration mode."
    else
        echo "Failed to start upstart service."
        exit 1
    fi
fi

loginctl enable-linger $USER

if [ $? -ne 0 ]; then
    echo "Failed to enable linger for user."
    exit 1
fi
