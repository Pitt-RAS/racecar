Steps to use sim:

1. Set ROBOT_IP to the desired target in robot.env
2. Change noah to the target username everywhere in robot.sh
3. ./robot.sh deploy

Periodically run 
  docker system prune
to delete old images if imageprune.py doesn't do it's job
