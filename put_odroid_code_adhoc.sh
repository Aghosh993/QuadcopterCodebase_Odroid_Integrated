#!/bin/bash
rsync -chavzP --stats --delete ./odroid_code alarm@140.100.160.2:/home/alarm/ 
