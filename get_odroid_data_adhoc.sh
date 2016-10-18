#!/bin/bash
rsync -chavzP --stats --delete alarm@140.100.160.2:/home/alarm/data_collection/ ./data_collection
