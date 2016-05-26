#!/bin/sh
rsync -avz --delete-after --exclude=/ipc --exclude=/env --exclude=.git --exclude=__pycache__ --exclude="*.o" --exclude="*.lst" --exclude="*.d" --exclude="*.egg-info" ./ cvra@192.168.2.10:run/
ssh cvra@192.168.2.10 "cd run; ./deploy-setup.sh"
