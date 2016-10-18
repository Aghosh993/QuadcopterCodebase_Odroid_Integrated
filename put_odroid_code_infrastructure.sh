#!/bin/bash
rsync -chavzP --stats --delete ./odroid_code alarm@192.168.2.201:/home/alarm/ 
