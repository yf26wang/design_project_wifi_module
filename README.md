# FYDP - WIFI module

## Overview
This contains the code running on the wifi module of the camera embedded system. The wifi module is an ESP-32 runing FreeRTOS. The code connects to a wifi network, reads data from the camera repeatedly, and configures two identical tasks to send http requests to the cloud server. Two tasks maximizes the two cores of the ESP-32 and improves the framerate of sending. The images sent are tagged with a sequence number to allow the cloud server to reorder them properly.

## Running
- Download ESP-32 visual studio code extension
- Connect host to the ESP-32 wifi module 
- Run ESP-IDF Select port to use
- Run ESP-IDF Build, Flash, and Monitor extension command
