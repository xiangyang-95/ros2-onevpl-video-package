#!/bin/bash

if [[ $EUID -ne 0 ]]; then
  echo "Please run the script with sudo permissions"
  exit 1
fi

apt update
apt -y install cmake pkg-config build-essential

# use wget to fetch the Intel repository public key
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
# add to your apt sources keyring so that archives signed with this key will be trusted.
apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
# remove the public key
rm GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

add-apt-repository -y "deb https://apt.repos.intel.com/oneapi all main"

apt install -y intel-basekit