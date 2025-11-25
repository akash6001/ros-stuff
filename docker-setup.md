# Setting up ROS 2 using Docker
## What is Docker?

Docker is a tool that enables developers to build software that runs consistently across different platforms (computers with different operating systems, servers, etc.). As a concrete example, a software developer working on macOS might use Docker to build an application that also works on a Linux-based computer - which is similar to what we will be doing for ROS 2.

Docker uses an approach called _containerization_, which differs from virtualization (i.e. using virtual machines). However, instead of emulating an entire operating system, Docker isolates only essential components needed for the software or application to run. This makes it faster and easier to use than a virtual machine.

## Installing Docker

Install Docker Desktop from the official website - there are 2 options for Mac, choose the one that matches your hardware.

![Docker Desktop Installation](https://github.com/user-attachments/assets/87a20c6a-ba3a-4e45-9d57-66ed074b8253)

Once you have completed the installation, run the following command in terminal to check:

```
docker --version
```

## Installing and using ROS 2

1. Ensure that Docker Desktop is open (the terminal command won't work unless the Docker engine is running) and run the following command:

   ```
   docker pull osrf/ros:humble-desktop
   ```

   or if you want to use a different distribution replace `humble` with `jazzy` or `kilted` (the other two distributions which are currently supported.

2. Run a new container from the `osrf/ros:humble-desktop` image.

   ```
   docker run -it --rm osrf/ros:humble-desktop
   ```

   You might get a warning about platform compatability, but this does not affect the functionality of the container.

(to insert)
- display access (xquartz)
- tmux
- customisation (Dockerfile)
- final test
- saving files (bind mounts)

   
