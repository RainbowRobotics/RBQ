#!/usr/bin/env bash
clear
while true; do
    if ping -c 1 -W 1 8.8.8.8 &> /dev/null; then
    	echo "$(date): Internet available, starting VPN ..." >> vpn.log
        if ! ip link show wg0 &> /dev/null; then
            echo "Starting VPN ..."
            sudo wg-quick up wg0 || echo "Failed to start VPN"
        fi
    else
    	echo "$(date): VPN server not reachable " >> vpn.log
        if ip link show wg0 &> /dev/null; then
            echo "Stopping VPN ..."
            sudo wg-quick down wg0
        fi
    fi
    sleep 5
done

