#!/bin/sh
rsync -avz --delete-after --exclude=/ipc --exclude=/env --exclude=.git --exclude=__pycache__ --exclude="*.o" --exclude="*.lst" --exclude="*.d" --exclude="*.egg-info" ./ cvra@10.0.10.1:run/
ssh cvra@10.0.10.1 "cd run; ./deploy-setup.sh"
