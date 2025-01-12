# Docker Image

Welcome to the Docker image source code!

You can create your own Docker image from scratch here. Check out [full Docker docs](https://github.com/armlabstanford/collaborative-robotics/wiki/Docker-Quickstart).



To build the Dockerfile, you can run:


```sh
cd Docker

docker build . -t collaborative 

```

To run it, you simply need to use the tag name (`-t`) that you used previously:

```sh
docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m -v <computer-path>:<docker-path> collaborative
```

Any time you update the Dockerfile, re-run the build!

