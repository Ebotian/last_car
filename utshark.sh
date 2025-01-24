#!/bin/bash

sudo rm -rf /tmp/pipe
sudo mkfifo /tmp/pipe

#echo the mac address is: $2
echo '''
HOW TO SET WIRESHARK:
    1.Click Edit -> Preferences
    2.Click Protocols -> DLT_USER
    3.Click Edit (Encapsulations Table)
    4.Click New
    5.Under DLT, select "User 0 (DLT=147)" (adjust this selection as appropriate if the error message showed a different DLT number than 147)
    6.Under Payload Protocol, enter: btle
    7.Click OK
    8.Click OK
'''

sudo killall wireshark
sudo wireshark -k -i /tmp/pipe &

#echo "reset ubertooth"
#sudo ubertooth-util -r

#if [ "$2" != "NULL" ];then
#    echo "-t<address> set connection following target (example: -t22:44:66:88:aa:cc/48)"
#    sudo ubertooth-btle -t  $2
#fi

if [ "$1" == "-p" ];then
    echo "-p promiscuous: sniff active connections"
    sudo ubertooth-btle -p -I -c /tmp/pipe
elif [ "$1" == "-n" ];then
    echo "-n don't follow, only print advertisements"
    sudo ubertooth-btle -n -c /tmp/pipe
elif [ "$1" == "-f" ];then
    echo "-f follow connections"
    sudo ubertooth-btle -f -I -c /tmp/pipe
else
    echo "INPUT_ERROR: sudo bash ubertooth_wireshark.sh -f BC:23:4C:00:00:01"
    exit 1
fi

echo the mac address is: $2