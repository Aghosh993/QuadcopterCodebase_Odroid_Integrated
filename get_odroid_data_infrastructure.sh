#!/bin/bash
rsync -chavzP --stats --delete alarm@192.168.2.201:/home/alarm/data_collection/ ./data_collection
