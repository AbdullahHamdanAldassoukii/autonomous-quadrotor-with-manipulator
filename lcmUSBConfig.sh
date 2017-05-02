#!/bin/bash
ifconfig usb0 multicast
route add -net 224.0.0.0 netmask 240.0.0.0 dev usb0

