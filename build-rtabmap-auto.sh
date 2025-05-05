docker build --build-arg CACHEBUST=$(date +%s) -f Dockerfile-rtabMap-auto -t rtabmap-image-auto .
