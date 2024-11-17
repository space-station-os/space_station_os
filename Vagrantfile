# -*- mode: ruby -*-
# vi: set ft=ruby :

Vagrant.configure("2") do |config|
  config.vm.box = "ubuntu/jammy64"
  config.vm.box_version = "20241002.0.0"

  config.vm.provider "virtualbox" do |vb|
    vb.name = "spacestation"
    vb.gui = false
    vb.cpus = 2
    vb.memory = "2048"
    vb.customize [
      "modifyvm", :id,
      "--vram", "256",
      "--clipboard", "bidirectional",
      "--graphicscontroller", "vmsvga",
      "--accelerate3d", "on",
    ]
  end

  config.vm.provision "shell", inline: <<-SHELL
    # Install prerequirements
    apt-get -y update
    apt-get -y install --no-install-recommends linux-image-generic software-properties-common
    apt-get -y install --no-install-recommends ubuntu-desktop-minimal

    # Install ROS2
    add-apt-repository universe -y
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
    apt-get -y update
    apt-get -y install ros-humble-desktop ros-dev-tools python3-colcon-common-extensions
    source /opt/ros/humble/setup.bash

    # Install space station os
    cd /home/vagrant/ && git clone https://github.com/space-station-os/space_station_os.git && cd ./space_station_os/
    colcon build
    source install/setup.bash

    # Setup bash_profile
    echo "source /opt/ros/humble/setup.bash" >> /home/vagrant/.bash_profile
    echo "source /home/vagrant/space_station_os/install/setup.bash" >> /home/vagrant/.bash_profile

    # Update vbguest
    wget http://download.virtualbox.org/virtualbox/7.1.4/VBoxGuestAdditions_7.1.4.iso
    mkdir /media/VBoxGuestAdditions
    mount -o loop,ro VBoxGuestAdditions_7.1.4.iso /media/VBoxGuestAdditions
    sh /media/VBoxGuestAdditions/VBoxLinuxAdditions.run
    rm VBoxGuestAdditions_7.1.4.iso
    umount /media/VBoxGuestAdditions
    rmdir /media/VBoxGuestAdditions

    # Reboot
    shutdown -r now
  SHELL
end
